#include "subroot_node.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "flow_packet.h"
#include "flow_power.h"
#include "flow_sync.h"
#include "lora_test_config.h"
#include "mesh_dedup.h"
#include "e220_lora.h"

#if CONFIG_FLOW_SUBROOT_LIGHT_SLEEP && CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

static const char *TAG = "FLOW_SUBROOT";

#define FLOW_ESPNOW_CHANNEL 1
#define FLOW_LORA_MAX_ROUTE_LEN 4

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_self_mac[6] = {0};
static mesh_dedup_table_t s_dedup = {0};
#if CONFIG_FLOW_SUBROOT_SELF_TEST
static uint32_t s_probe_sequence = 0;
#endif

static QueueHandle_t s_lora_tx_queue = NULL;

static inline bool mac_eq6(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

static int8_t clamp_i16_to_i8(int16_t value)
{
    if (value > 127) {
        return 127;
    }
    if (value < -128) {
        return -128;
    }
    return (int8_t)value;
}

static void __attribute__((unused)) mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static esp_err_t init_wifi_espnow(void);
static esp_err_t init_lora_radio(void);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
}
#else
static void on_espnow_send(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    (void)status;
}
#endif

static void enqueue_lora_tx(const flow_packet_t *pkt)
{
    if (s_lora_tx_queue == NULL || pkt == NULL) {
        return;
    }

    (void)xQueueSend(s_lora_tx_queue, pkt, 0);
}

static void on_espnow_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len != (int)sizeof(flow_packet_t)) {
        return;
    }

    flow_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (!flow_packet_validate_crc32(&pkt)) {
        return; // drop silently
    }

    if (pkt.type != FLOW_PKT_TYPE_METER_DATA) {
        return;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
        return;
    }

    // SUBROOT que ingere do ESP-NOW.
    memcpy(pkt.subroot_mac, s_self_mac, sizeof(pkt.subroot_mac));
    pkt.espnow_rssi = (recv_info->rx_ctrl != NULL) ? (int8_t)recv_info->rx_ctrl->rssi : INT8_MIN;

    // Conta este forward (SUBROOT -> LoRa)
    pkt.hop_count++;

    flow_packet_update_crc32(&pkt);
    ESP_LOGI(TAG,
             "ENCAMINHANDO ESPNOW->LoRa seq=%lu espnow_rssi=%d",
             (unsigned long)pkt.sequence,
             (int)pkt.espnow_rssi);
    enqueue_lora_tx(&pkt);
}

static void lora_tx_task(void *arg)
{
    (void)arg;

    flow_packet_t pkt;
    while (1) {
        if (xQueueReceive(s_lora_tx_queue, &pkt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        esp_err_t err = e220_lora_transmit((const uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha TX LoRa tipo=%u seq=%lu (%s)",
                     (unsigned)pkt.type,
                     (unsigned long)pkt.sequence,
                     esp_err_to_name(err));
        }
    }
}

#if CONFIG_FLOW_SUBROOT_SELF_TEST
static void subroot_probe_task(void *arg)
{
    (void)arg;

    char self_mac_str[18] = {0};
    mac_to_str(s_self_mac, self_mac_str, sizeof(self_mac_str));

    while (1) {
        flow_packet_t pkt;
        flow_packet_init_empty(&pkt);

        pkt.type = FLOW_PKT_TYPE_METER_DATA;
        pkt.sequence = ++s_probe_sequence;
        memcpy(pkt.meter_id, s_self_mac, sizeof(pkt.meter_id));
        memcpy(pkt.subroot_mac, s_self_mac, sizeof(pkt.subroot_mac));
        pkt.volume_liters = (uint32_t)CONFIG_FLOW_SUBROOT_SELF_TEST_VOLUME_LITERS;
        pkt.delta_liters = (uint32_t)CONFIG_FLOW_SUBROOT_SELF_TEST_DELTA_LITERS;
        pkt.timestamp_ms = esp_timer_get_time() / 1000LL;
        pkt.battery_pct = 100;
        pkt.hop_count = 1;
        pkt.espnow_rssi = INT8_MIN;
        pkt.lora_rssi = INT8_MIN;
        pkt.lora_snr = INT8_MIN;

        flow_packet_update_crc32(&pkt);

        ESP_LOGI(TAG,
                 "TX SONDA SUBROOT->ROOT meter=%s seq=%lu vol=%luL delta=%luL",
                 self_mac_str,
                 (unsigned long)pkt.sequence,
                 (unsigned long)pkt.volume_liters,
                 (unsigned long)pkt.delta_liters);
        enqueue_lora_tx(&pkt);

        vTaskDelay(pdMS_TO_TICKS(CONFIG_FLOW_SUBROOT_SELF_TEST_INTERVAL_MS));
    }
}
#endif

static void maybe_forward_lora_meter_data(const e220_lora_event_t *event, const flow_packet_t *rx_pkt)
{
    if (event == NULL || rx_pkt == NULL) {
        return;
    }

    flow_packet_t pkt = *rx_pkt;

    if (!flow_packet_validate_crc32(&pkt)) {
        return; // drop silently
    }

    if (pkt.type != FLOW_PKT_TYPE_METER_DATA) {
        return;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
        return;
    }

    if (pkt.route_len >= FLOW_LORA_MAX_ROUTE_LEN) {
        return;
    }

    // Atualiza metricas do ultimo hop LoRa (para este nó).
    pkt.lora_rssi = clamp_i16_to_i8(event->rssi_dbm);
    pkt.lora_snr = event->snr_db;

    // Este nó vai forwardar via LoRa: append no caminho.
    memcpy(pkt.route_path[pkt.route_len], s_self_mac, 6);
    pkt.route_len++;

    // Conta este forward (SUBROOT -> LoRa)
    pkt.hop_count++;

    flow_packet_update_crc32(&pkt);
    enqueue_lora_tx(&pkt);
}

static void maybe_forward_lora_ack(const e220_lora_event_t *event, const flow_packet_t *rx_pkt)
{
    if (event == NULL || rx_pkt == NULL) {
        return;
    }

    flow_packet_t pkt = *rx_pkt;

    if (!flow_packet_validate_crc32(&pkt)) {
        return; // drop silently
    }

    if (pkt.type != FLOW_PKT_TYPE_ACK) {
        return;
    }

    ESP_LOGI(TAG,
             "ACK LoRa RX seq=%lu rssi=%d snr=%d route_len=%u",
             (unsigned long)pkt.sequence,
             (int)clamp_i16_to_i8(event->rssi_dbm),
             (int)event->snr_db,
             (unsigned)pkt.route_len);

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
        return;
    }

    // Se este for o SUBROOT de ingestao, ACK chegou ao destino.
    if (mac_eq6(pkt.subroot_mac, s_self_mac)) {
        ESP_LOGI(TAG, "ACK recebido para meter (seq=%lu)", (unsigned long)pkt.sequence);
        return;
    }

    // Encaminhamento em direcao ao subroot de origem seguindo o caminho reverso.
    if (pkt.route_len == 0) {
        return;
    }

    const uint8_t *expected = pkt.route_path[pkt.route_len - 1];
    if (!mac_eq6(expected, s_self_mac)) {
        return;
    }

    // Pop deste hop.
    pkt.route_len--;

    // Atualiza metricas do ultimo hop LoRa (para este nó).
    pkt.lora_rssi = clamp_i16_to_i8(event->rssi_dbm);
    pkt.lora_snr = event->snr_db;

    flow_packet_update_crc32(&pkt);
    enqueue_lora_tx(&pkt);
}

// ---------------------------------------------------------------------------
// Beacon: selecao de pai (menor rank; empate por RSSI), dedup por sync_seq
// (1 rebroadcast por burst) e repasse em burst nos DOIS radios: ESP-NOW para
// os meters do cluster e LoRa para subroots de rank maior (cascata multi-hop).
// ---------------------------------------------------------------------------

static uint8_t s_my_rank = FLOW_RANK_INVALID;
static flow_parent_t s_parent;
static portMUX_TYPE s_parent_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_last_rebroadcast_seq = 0;

// Avalia um beacon LoRa. Retorna true se e um burst NOVO do pai adotado
// (=> o chamador deve medir deriva e rebroadcastar).
static bool subroot_handle_beacon(const flow_beacon_t *in, int16_t rssi)
{
    if (in == NULL || in->hop_from_root >= (FLOW_RANK_INVALID - 1U)) {
        return false;
    }

    portENTER_CRITICAL(&s_parent_mux);
    (void)flow_sync_consider_beacon(&s_parent, in, FLOW_UPLINK_LORA, rssi);
    const flow_parent_t p = s_parent;
    portEXIT_CRITICAL(&s_parent_mux);

    if (!p.valid || memcmp(p.parent_id, in->sender_id, 6) != 0) {
        return false; // beacon de um vizinho que nao e o pai adotado
    }
    s_my_rank = p.my_rank;

    if (in->sync_seq == s_last_rebroadcast_seq) {
        return false; // mesmo burst (ou beacon de descoberta): ja repassado
    }
    s_last_rebroadcast_seq = in->sync_seq;
    return true;
}

// Repassa o beacon capturado aos filhos: N copias espacadas, com a agenda
// (next_sync/next_collect) descontada do tempo decorrido desde a captura.
// LoRa so ate a profundidade maxima da arvore.
static void subroot_rebroadcast_burst(const flow_beacon_t *in, int64_t rx_at_ms)
{
    for (int i = 0; i < CONFIG_FLOW_REBROADCAST_COPIES; ++i) {
        const int64_t elapsed = esp_timer_get_time() / 1000LL - rx_at_ms;
        const uint32_t sync_in = (in->next_sync_in_ms > (uint32_t)elapsed)
                                     ? in->next_sync_in_ms - (uint32_t)elapsed : 0U;
        const uint32_t collect_in = (in->next_collect_in_ms > (uint32_t)elapsed)
                                        ? in->next_collect_in_ms - (uint32_t)elapsed : 0U;
        flow_beacon_t out;
        flow_sync_build_beacon(&out, s_self_mac, in->root_id, s_my_rank,
                               in->sync_seq, in->root_epoch_ms, collect_in, sync_in);

        esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&out, sizeof(out));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha rebroadcast BEACON ESP-NOW (%s)", esp_err_to_name(err));
        }
        if (s_my_rank < FLOW_LORA_MAX_ROUTE_LEN) {
            err = e220_lora_transmit((const uint8_t *)&out, sizeof(out));
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Falha rebroadcast BEACON LoRa (%s)", esp_err_to_name(err));
            }
        }
        if (i + 1 < CONFIG_FLOW_REBROADCAST_COPIES) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_FLOW_REBROADCAST_SPACING_MS));
        }
    }
    ESP_LOGI(TAG, "BEACON repassado rank=%u seq=%lu copias=%d (ESP-NOW%s)",
             (unsigned)s_my_rank, (unsigned long)in->sync_seq,
             CONFIG_FLOW_REBROADCAST_COPIES,
             (s_my_rank < FLOW_LORA_MAX_ROUTE_LEN) ? "+LoRa" : "");
}

static void lora_rx_task(void *arg)
{
    (void)arg;

    while (1) {
        e220_lora_event_t event = {0};
        esp_err_t err = e220_lora_receive_event(&event, pdMS_TO_TICKS(200));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            continue;
        }
        if (event.type != E220_LORA_EVENT_RX_DONE) {
            continue;
        }
        if (event.payload_len == sizeof(flow_beacon_t)) {
            flow_beacon_t b;
            memcpy(&b, event.payload, sizeof(b));
            if (flow_beacon_validate_crc32(&b) &&
                subroot_handle_beacon(&b, event.rssi_dbm)) {
                subroot_rebroadcast_burst(&b, esp_timer_get_time() / 1000LL);
            }
            continue;
        }
        if (event.payload_len != sizeof(flow_packet_t)) {
            continue;
        }

        flow_packet_t pkt;
        memcpy(&pkt, event.payload, sizeof(pkt));

        if (!flow_packet_header_ok(&pkt)) {
            continue;
        }

        if (pkt.type == FLOW_PKT_TYPE_METER_DATA) {
            maybe_forward_lora_meter_data(&event, &pkt);
        } else if (pkt.type == FLOW_PKT_TYPE_ACK) {
            maybe_forward_lora_ack(&event, &pkt);
        }
    }
}

static esp_err_t init_wifi_espnow(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(FLOW_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp_now init");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(on_espnow_send), TAG, "esp_now send cb");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_espnow_recv), TAG, "esp_now recv cb");
    flow_power_mark_wifi_up();

    // Peer broadcast (necessario para forward via ESP-NOW caso seja desejado no futuro)
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t init_lora_radio_ex(bool skip_radio_config)
{
    e220_lora_config_t config = {
        .skip_radio_config = skip_radio_config,
        .uart_port = CONFIG_FLOW_E220_UART_PORT,
        .pin_tx = E220_PIN_TX,
        .pin_rx = E220_PIN_RX,
        .pin_m0 = E220_PIN_M0,
        .pin_m1 = E220_PIN_M1,
        .pin_aux = E220_PIN_AUX,
        .baud = CONFIG_FLOW_E220_BAUD,
        .address = (uint16_t)CONFIG_FLOW_E220_ADDRESS,
        .channel = (uint8_t)CONFIG_FLOW_E220_CHANNEL,
        .air_data_rate = (uint8_t)CONFIG_FLOW_E220_AIR_DATA_RATE,
        .tx_power_dbm = (int8_t)CONFIG_FLOW_E220_TX_POWER_DBM,
        .wor_period_ms = (uint16_t)CONFIG_FLOW_E220_WOR_PERIOD_MS,
        .fixed_mode = CONFIG_FLOW_E220_FIXED_MODE,
        .rssi_byte = CONFIG_FLOW_E220_RSSI_BYTE,
        .frequency_hz = LORA_TEST_FREQUENCY_HZ,
    };

    ESP_RETURN_ON_ERROR(e220_lora_init(&config), TAG, "e220 init");
    e220_lora_flush_events();
    return e220_lora_start_rx_continuous();
}

static esp_err_t init_lora_radio(void)
{
    return init_lora_radio_ex(false);
}

#if CONFIG_FLOW_SUBROOT_LIGHT_SLEEP
static void subroot_enable_power_save(void)
{
    // Mesmo duty-cycle connectionless do METER, para captar os bursts ESP-NOW
    // enquanto dorme em light sleep. Chamar DEPOIS de esp_wifi_start().
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps falhou: %s", esp_err_to_name(err));
    }
    err = esp_now_set_wake_window(CONFIG_FLOW_LS_WAKE_WINDOW_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_set_wake_window falhou: %s", esp_err_to_name(err));
    }
    err = esp_wifi_connectionless_module_set_wake_interval(CONFIG_FLOW_LS_WAKE_INTERVAL_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_wake_interval falhou: %s (habilitar connectionless PS?)",
                 esp_err_to_name(err));
    }
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 10,
        .light_sleep_enable = true,
    };
    err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_pm_configure falhou: %s", esp_err_to_name(err));
    }
#else
    ESP_LOGW(TAG, "CONFIG_PM_ENABLE desligado: sem automatic light sleep");
#endif
    ESP_LOGW(TAG,
             "SUBROOT light sleep: window=%dms interval=%dms (LoRa RX continuo ainda mascara a economia ate WOR/Fase 2)",
             CONFIG_FLOW_LS_WAKE_WINDOW_MS, CONFIG_FLOW_LS_WAKE_INTERVAL_MS);
}
#endif /* CONFIG_FLOW_SUBROOT_LIGHT_SLEEP */

#if CONFIG_FLOW_SUBROOT_DEEP_SLEEP
// ---------------------------------------------------------------------------
// Modo PRODUCAO: sessao agendada com deep sleep.
//
// Ciclo: acorda guard antes do burst do pai -> escuta LoRa (Wi-Fi DESLIGADO)
// -> capta beacon -> mede deriva -> liga Wi-Fi -> rebroadcast burst (ESP-NOW
// + LoRa) -> decide proximo evento (sync ou coleta) -> E220 em sleep -> deep
// sleep. Na coleta: acorda antes da janela, ESP-NOW RX dos meters -> LoRa TX
// ao pai -> repassa ACKs -> dorme ate o proximo sync.
// ---------------------------------------------------------------------------
#include "esp_sleep.h"
#include "flow_power.h"

enum { WAKE_SCAN = 0, WAKE_SYNC = 1, WAKE_COLLECT = 2 };

RTC_DATA_ATTR static bool         s_rtc_cold = true;
RTC_DATA_ATTR static uint32_t     s_rtc_wakes = 0;
RTC_DATA_ATTR static uint8_t      s_rtc_kind = WAKE_SCAN;
RTC_DATA_ATTR static uint8_t      s_rtc_misses = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_guard_ms = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_slept_ms = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_sync_after_collect_ms = 0;
RTC_DATA_ATTR static flow_drift_t s_rtc_drift = {0};

// Abaixo deste alvo nao vale pagar boot+reinit: espera acordado.
// Break-even: acordar custa ~400 ms de boot @ ~30 mA (~3 uAh); ficar acordado
// custa ~32 mA. Dormir compensa para qualquer intervalo > ~0,5 s, entao 2 s da
// folga. NUNCA subir isto para dezenas de s: com resync curto (bancada) o no
// nunca dormiria — ficaria 100% acordado (~32 mA) e a deriva nunca seria medida
// (o esp_timer so zera no deep sleep).
#define FLOW_STAY_AWAKE_THRESHOLD_MS 2000

static bool s_wifi_up = false;

static void subroot_ensure_wifi(void)
{
    if (!s_wifi_up) {
        ESP_ERROR_CHECK(init_wifi_espnow());
        s_wifi_up = true;
    }
}

// Agenda o proximo evento e dorme. Retorna apenas no caminho "curto demais
// p/ deep sleep" (fica acordado esperando).
static void subroot_schedule_and_sleep(uint8_t kind, uint32_t event_in_ms,
                                       uint32_t sync_after_collect_ms)
{
    const uint32_t unc = flow_drift_uncertainty_ppm(
        &s_rtc_drift, CONFIG_FLOW_DRIFT_INIT_PPM, CONFIG_FLOW_DRIFT_RESIDUAL_PPM);
    const uint8_t rank_for_guard = (s_my_rank == FLOW_RANK_INVALID) ? 1U : s_my_rank;
    const uint32_t guard = flow_sync_compute_guard(
        unc, event_in_ms, rank_for_guard, s_rtc_misses,
        CONFIG_FLOW_HOP_JITTER_MS, CONFIG_FLOW_GUARD_MIN_MS, CONFIG_FLOW_GUARD_MAX_MS);
    const uint32_t target = (event_in_ms > guard) ? event_in_ms - guard : 0U;
    const uint32_t corrected = flow_drift_apply(&s_rtc_drift, target);

    s_rtc_kind = kind;
    s_rtc_guard_ms = guard;
    s_rtc_slept_ms = corrected;
    s_rtc_sync_after_collect_ms = sync_after_collect_ms;

    ESP_LOGI(TAG,
             "Proximo evento=%s em %lums (guard=%lums deriva=%ldppm[%u] misses=%u sleep=%lums)",
             (kind == WAKE_COLLECT) ? "COLETA" : (kind == WAKE_SYNC) ? "SYNC" : "SCAN",
             (unsigned long)event_in_ms, (unsigned long)guard,
             (long)s_rtc_drift.ppm, (unsigned)s_rtc_drift.samples,
             (unsigned)s_rtc_misses, (unsigned long)corrected);

    if (corrected < FLOW_STAY_AWAKE_THRESHOLD_MS) {
        ESP_LOGI(TAG, "Alvo curto: esperando acordado %lums", (unsigned long)corrected);
        vTaskDelay(pdMS_TO_TICKS(corrected));
        return;
    }
    (void)e220_lora_prepare_deep_sleep();
    flow_power_deep_sleep_ms(corrected); // nao retorna
}

// Escuta LoRa ate captar um burst NOVO do pai (early-exit) ou estourar budget.
static bool subroot_listen_beacon(uint32_t budget_ms, flow_beacon_t *out,
                                  int64_t *rx_at_ms)
{
    const int64_t start = esp_timer_get_time() / 1000LL;
    const int64_t end = start + budget_ms;

    while (esp_timer_get_time() / 1000LL < end) {
        e220_lora_event_t event = {0};
        if (e220_lora_receive_event(&event, pdMS_TO_TICKS(200)) != ESP_OK ||
            event.type != E220_LORA_EVENT_RX_DONE) {
            continue;
        }
        if (event.payload_len == sizeof(flow_beacon_t)) {
            flow_beacon_t b;
            memcpy(&b, event.payload, sizeof(b));
            if (flow_beacon_validate_crc32(&b) &&
                subroot_handle_beacon(&b, event.rssi_dbm)) {
                *out = b;
                *rx_at_ms = esp_timer_get_time() / 1000LL;
                return true;
            }
        } else if (event.payload_len == sizeof(flow_packet_t)) {
            // Dados/ACK fora de hora ainda sao repassados (fila LoRa).
            flow_packet_t pkt;
            memcpy(&pkt, event.payload, sizeof(pkt));
            if (flow_packet_header_ok(&pkt)) {
                if (pkt.type == FLOW_PKT_TYPE_METER_DATA) {
                    maybe_forward_lora_meter_data(&event, &pkt);
                } else if (pkt.type == FLOW_PKT_TYPE_ACK) {
                    maybe_forward_lora_ack(&event, &pkt);
                }
            }
        }
    }
    return false;
}

// Janela de coleta: ESP-NOW RX dos meters (callback -> fila -> lora_tx_task)
// + LoRa RX inline p/ dados de subroots mais fundos e ACKs de volta.
static void subroot_run_collect_window(void)
{
    subroot_ensure_wifi();
    const int64_t end = esp_timer_get_time() / 1000LL +
                        CONFIG_FLOW_COLLECT_WINDOW_MS + 2 * CONFIG_FLOW_GUARD_MIN_MS;
    ESP_LOGI(TAG, "Janela de COLETA aberta (%dms)", CONFIG_FLOW_COLLECT_WINDOW_MS);
    while (esp_timer_get_time() / 1000LL < end) {
        e220_lora_event_t event = {0};
        if (e220_lora_receive_event(&event, pdMS_TO_TICKS(200)) != ESP_OK ||
            event.type != E220_LORA_EVENT_RX_DONE ||
            event.payload_len != sizeof(flow_packet_t)) {
            continue;
        }
        flow_packet_t pkt;
        memcpy(&pkt, event.payload, sizeof(pkt));
        if (!flow_packet_header_ok(&pkt)) {
            continue;
        }
        if (pkt.type == FLOW_PKT_TYPE_METER_DATA) {
            maybe_forward_lora_meter_data(&event, &pkt);
        } else if (pkt.type == FLOW_PKT_TYPE_ACK) {
            maybe_forward_lora_ack(&event, &pkt);
        }
    }
    ESP_LOGI(TAG, "Janela de COLETA encerrada");
}

static void subroot_scheduled_run(void)
{
    flow_power_stay_awake_hatch();

    const uint8_t kind_at_wake = s_rtc_cold ? WAKE_SCAN : s_rtc_kind;
    if (s_rtc_cold) {
        flow_drift_reset(&s_rtc_drift);
        s_rtc_misses = 0;
    }
    s_rtc_cold = false;
    s_rtc_wakes++;

    mesh_dedup_init(&s_dedup);
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);
    flow_sync_reset_parent(&s_parent);

    // LoRa sobe sempre (escuta/encaminha); registradores so no cold boot.
    // Falha (ex.: brownout com o radio ligando) NAO pode virar loop de panico:
    // dorme 60 s e tenta de novo (a fonte pode se recuperar).
    esp_err_t lora_err =
        init_lora_radio_ex(kind_at_wake != WAKE_SCAN || s_rtc_wakes > 1);
    if (lora_err != ESP_OK) {
        ESP_LOGE(TAG, "Init LoRa falhou (%s); retry em 60s via deep sleep",
                 esp_err_to_name(lora_err));
        flow_power_deep_sleep_ms(60000);
    }

    s_lora_tx_queue = xQueueCreate(8, sizeof(flow_packet_t));
    xTaskCreate(lora_tx_task, "flow_lora_tx", 4096, NULL, 8, NULL);

    ESP_LOGW(TAG, "MODO FLOW MESH: SUBROOT agendado, wake #%lu tipo=%d guard=%lums",
             (unsigned long)s_rtc_wakes, (int)kind_at_wake,
             (unsigned long)s_rtc_guard_ms);

    uint8_t kind = kind_at_wake;
    while (1) {
        const int64_t wake_at = esp_timer_get_time() / 1000LL;

        if (kind == WAKE_COLLECT) {
            subroot_run_collect_window();
            const int64_t elapsed_evt =
                esp_timer_get_time() / 1000LL - wake_at - s_rtc_guard_ms;
            uint32_t sync_in = s_rtc_sync_after_collect_ms;
            sync_in = (sync_in > (uint32_t)elapsed_evt) ? sync_in - (uint32_t)elapsed_evt
                                                        : CONFIG_FLOW_GUARD_MIN_MS;
            subroot_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
            kind = s_rtc_kind;
            continue;
        }

        // SYNC ou SCAN: escutar o beacon do pai.
        const uint32_t budget = (kind == WAKE_SCAN)
            ? CONFIG_FLOW_SCAN_LISTEN_MS
            : 2U * s_rtc_guard_ms + CONFIG_FLOW_BEACON_BURST_MS;

        flow_beacon_t b;
        int64_t rx_at = 0;
        if (subroot_listen_beacon(budget, &b, &rx_at)) {
            if (kind == WAKE_SYNC) {
                const int32_t err_ms =
                    (int32_t)(rx_at - wake_at) - (int32_t)s_rtc_guard_ms;
                flow_drift_update(&s_rtc_drift, err_ms, s_rtc_slept_ms);
                ESP_LOGI(TAG, "Deriva: err=%ldms sleep=%lums -> %ldppm",
                         (long)err_ms, (unsigned long)s_rtc_slept_ms,
                         (long)s_rtc_drift.ppm);
            }
            s_rtc_misses = 0;

            subroot_ensure_wifi();
            subroot_rebroadcast_burst(&b, rx_at);

            // Decide o proximo evento pela agenda fresca do beacon.
            const int64_t since_rx = esp_timer_get_time() / 1000LL - rx_at;
            uint32_t sync_in = (b.next_sync_in_ms > (uint32_t)since_rx)
                                   ? b.next_sync_in_ms - (uint32_t)since_rx : 0U;
            uint32_t collect_in = (b.next_collect_in_ms > (uint32_t)since_rx)
                                      ? b.next_collect_in_ms - (uint32_t)since_rx : 0U;
            if (collect_in > 0U && collect_in < sync_in) {
                subroot_schedule_and_sleep(WAKE_COLLECT, collect_in,
                                           sync_in - collect_in);
            } else {
                subroot_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
            }
        } else {
            // MISS: escalada de guarda; apos N, SCAN duty-cycled.
            s_rtc_misses++;
            ESP_LOGW(TAG, "BEACON perdido (misses=%u)", (unsigned)s_rtc_misses);
            if (s_rtc_misses >= CONFIG_FLOW_SCAN_AFTER_MISSES) {
                uint32_t retry = (uint32_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000U / 4U;
                if (retry < 10000U) {
                    retry = 10000U;
                }
                subroot_schedule_and_sleep(WAKE_SCAN, retry, 0);
            } else {
                const int64_t elapsed_evt =
                    esp_timer_get_time() / 1000LL - wake_at - s_rtc_guard_ms;
                uint32_t sync_in = (uint32_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000U;
                sync_in = (sync_in > (uint32_t)elapsed_evt)
                              ? sync_in - (uint32_t)elapsed_evt : 10000U;
                subroot_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
            }
        }
        kind = s_rtc_kind;
    }
}
#endif /* CONFIG_FLOW_SUBROOT_DEEP_SLEEP */

void subroot_node_run(void)
{
#if CONFIG_FLOW_SUBROOT_DEEP_SLEEP
    subroot_scheduled_run();
    return;
#endif
    mesh_dedup_init(&s_dedup);
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    ESP_ERROR_CHECK(init_wifi_espnow());
    ESP_ERROR_CHECK(init_lora_radio());

#if CONFIG_FLOW_SUBROOT_LIGHT_SLEEP
    subroot_enable_power_save();
#endif

    s_lora_tx_queue = xQueueCreate(8, sizeof(flow_packet_t));
    if (s_lora_tx_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila de TX LoRa");
        return;
    }

    ESP_LOGW(TAG, "MODO FLOW MESH: SUBROOT ativo (ESP-NOW ch=%d / LoRa %uMHz)",
             FLOW_ESPNOW_CHANNEL, (unsigned)CONFIG_LORA_TEST_FREQUENCY_MHZ);

    xTaskCreate(lora_tx_task, "flow_lora_tx", 4096, NULL, 8, NULL);
    xTaskCreate(lora_rx_task, "flow_lora_rx", 4096, NULL, 8, NULL);

#if CONFIG_FLOW_SUBROOT_SELF_TEST
    xTaskCreate(subroot_probe_task, "flow_probe_tx", 4096, NULL, 6, NULL);
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
