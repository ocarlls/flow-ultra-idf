#include "sdkconfig.h"
#include "meter_node.h"

#if CONFIG_FLOW_METER_TX_TEST

// ---------------------------------------------------------------------------
// Modo de BANCADA: relay ESP-NOW em light sleep (para medir consumo no PPK2).
// Implementacao em meter_tx_test.c.
// ---------------------------------------------------------------------------
#include "meter_tx_test.h"

void meter_node_run(void)
{
    meter_tx_test_run();
}

#else /* !CONFIG_FLOW_METER_TX_TEST */

// ---------------------------------------------------------------------------
// Modo de PRODUCAO: sessao agendada com deep sleep.
//
// Ciclo SYNC: acorda guard antes do burst do pai -> escuta ESP-NOW com
// early-exit -> mede deriva (guard adaptativo, converge de minutos p/
// segundos) -> decide proximo evento (sync ou coleta) -> deep sleep.
// Ciclo COLETA: acorda no inicio da janela (relogio fresco: a coleta e sempre
// precedida de um burst de sync "lead" do root) -> transmite a leitura
// (placeholder sintetico ate o sensor real) em N copias -> dorme ate o sync.
// Fallback: miss escala a guarda 2x; N misses -> SCAN; cold boot -> SCAN.
// ---------------------------------------------------------------------------
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "flow_packet.h"
#include "flow_power.h"
#include "flow_sync.h"

static const char *TAG = "FLOW_METER";

#define FLOW_ESPNOW_CHANNEL 1
#define FLOW_METER_NVS_NS "flow_mesh"
#define FLOW_METER_NVS_KEY_SEQ "seq"
#define FLOW_METER_NVS_SEQ_EVERY 10
#define FLOW_METER_SYNTH_DELTA_L 25U

enum { WAKE_SCAN = 0, WAKE_SYNC = 1, WAKE_COLLECT = 2 };

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estado que sobrevive ao deep sleep (RTC RAM).
RTC_DATA_ATTR static bool         s_rtc_cold = true;
RTC_DATA_ATTR static uint32_t     s_rtc_wakes = 0;
RTC_DATA_ATTR static uint8_t      s_rtc_kind = WAKE_SCAN;
RTC_DATA_ATTR static uint8_t      s_rtc_misses = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_guard_ms = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_slept_ms = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_sync_after_collect_ms = 0;
RTC_DATA_ATTR static flow_drift_t s_rtc_drift = {0};
RTC_DATA_ATTR static uint8_t      s_rtc_parent[6] = {0};
RTC_DATA_ATTR static uint8_t      s_rtc_my_rank = FLOW_RANK_INVALID;
// Leitura sintetica (placeholder ate o sensor real).
RTC_DATA_ATTR static uint32_t     s_rtc_volume_l = 0;
RTC_DATA_ATTR static uint32_t     s_rtc_sequence = 0;

static uint8_t s_self_mac[6] = {0};

// Pai descoberto nesta janela (escrito no callback de RX, lido depois).
static flow_parent_t s_parent;
static int64_t s_parent_rx_ms = 0; // uptime da captura do beacon vencedor
static portMUX_TYPE s_parent_mux = portMUX_INITIALIZER_UNLOCKED;

static void mac_to_str(const uint8_t mac[6], char *buf, size_t len)
{
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void on_espnow_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (data == NULL || len != (int)sizeof(flow_beacon_t)) {
        return;
    }
    flow_beacon_t b;
    memcpy(&b, data, sizeof(b));
    if (!flow_beacon_validate_crc32(&b)) {
        return;
    }
    const int16_t rssi = (info != NULL && info->rx_ctrl != NULL)
                             ? (int16_t)info->rx_ctrl->rssi
                             : INT16_MIN;
    const int64_t now = esp_timer_get_time() / 1000LL;
    portENTER_CRITICAL(&s_parent_mux);
    if (flow_sync_consider_beacon(&s_parent, &b, FLOW_UPLINK_ESPNOW, rssi)) {
        s_parent_rx_ms = now; // beacon que (re)definiu o pai: ancora da agenda
    }
    portEXIT_CRITICAL(&s_parent_mux);
}

static esp_err_t init_espnow(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(FLOW_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;
    if ((err = esp_now_init()) != ESP_OK) return err;
    if ((err = esp_now_register_recv_cb(on_espnow_recv)) != ESP_OK) return err;
    flow_power_mark_wifi_up();

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) return err;
    return ESP_OK;
}

// --- Persistencia da sequencia (NVS a cada N; RTC no dia-a-dia) -----------
static void nvs_load_sequence(void)
{
    nvs_handle_t h;
    if (nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    uint32_t seq = 0;
    esp_err_t err = nvs_get_u32(h, FLOW_METER_NVS_KEY_SEQ, &seq);
    if (err == ESP_OK) {
        // Retoma adiante do persistido (pode estar ate N-1 atras do real).
        s_rtc_sequence = seq + FLOW_METER_NVS_SEQ_EVERY;
    }
    nvs_close(h);
}

static void nvs_store_sequence(void)
{
    nvs_handle_t h;
    if (nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_set_u32(h, FLOW_METER_NVS_KEY_SEQ, s_rtc_sequence);
    (void)nvs_commit(h);
    nvs_close(h);
}

// --- Agenda + deep sleep ----------------------------------------------------
static void meter_schedule_and_sleep(uint8_t kind, uint32_t event_in_ms,
                                     uint32_t sync_after_collect_ms)
{
    const uint32_t unc = flow_drift_uncertainty_ppm(
        &s_rtc_drift, CONFIG_FLOW_DRIFT_INIT_PPM, CONFIG_FLOW_DRIFT_RESIDUAL_PPM);
    const uint8_t rank_for_guard =
        (s_rtc_my_rank == FLOW_RANK_INVALID) ? 1U : s_rtc_my_rank;
    // Para COLETA a guarda e minima: houve sync fresco logo antes (lead burst).
    const uint32_t guard = (kind == WAKE_COLLECT)
        ? CONFIG_FLOW_GUARD_MIN_MS
        : flow_sync_compute_guard(unc, event_in_ms, rank_for_guard, s_rtc_misses,
                                  CONFIG_FLOW_HOP_JITTER_MS,
                                  CONFIG_FLOW_GUARD_MIN_MS, CONFIG_FLOW_GUARD_MAX_MS);
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

    flow_power_deep_sleep_ms(corrected); // teardown wifi + deep sleep; nao retorna
}

// --- Coleta: origina a leitura (sintetica) em burst de N copias -------------
static void meter_send_reading(void)
{
    s_rtc_volume_l += FLOW_METER_SYNTH_DELTA_L;
    s_rtc_sequence++;
    if ((s_rtc_sequence % FLOW_METER_NVS_SEQ_EVERY) == 0U) {
        nvs_store_sequence();
    }

    flow_packet_t pkt;
    flow_packet_init_empty(&pkt);
    pkt.type = FLOW_PKT_TYPE_METER_DATA;
    pkt.sequence = s_rtc_sequence;
    memcpy(pkt.meter_id, s_self_mac, sizeof(pkt.meter_id));
    pkt.volume_liters = s_rtc_volume_l;
    pkt.delta_liters = FLOW_METER_SYNTH_DELTA_L;
    pkt.timestamp_ms = esp_timer_get_time() / 1000LL;
    pkt.battery_pct = 100; // placeholder ate ADC real
    pkt.hop_count = 0;
    flow_packet_update_crc32(&pkt);

    // Jitter inicial anti-colisao entre meters do mesmo cluster.
    vTaskDelay(pdMS_TO_TICKS(esp_random() % (CONFIG_FLOW_RETRY_JITTER_MS + 1U)));

    for (int i = 0; i < CONFIG_FLOW_REBROADCAST_COPIES; ++i) {
        esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha TX DATA (%s)", esp_err_to_name(err));
        }
        if (i + 1 < CONFIG_FLOW_REBROADCAST_COPIES) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_FLOW_REBROADCAST_SPACING_MS));
        }
    }
    ESP_LOGI(TAG, "DATA TX seq=%lu vol=%luL (%d copias)",
             (unsigned long)s_rtc_sequence, (unsigned long)s_rtc_volume_l,
             CONFIG_FLOW_REBROADCAST_COPIES);
}

// --- Escuta de beacon com early-exit ----------------------------------------
static bool meter_listen_beacon(uint32_t budget_ms, flow_parent_t *out,
                                int64_t *rx_at_ms)
{
    portENTER_CRITICAL(&s_parent_mux);
    flow_sync_reset_parent(&s_parent);
    s_parent_rx_ms = 0;
    portEXIT_CRITICAL(&s_parent_mux);

    const int64_t end = esp_timer_get_time() / 1000LL + budget_ms;
    while (esp_timer_get_time() / 1000LL < end) {
        vTaskDelay(pdMS_TO_TICKS(50));
        portENTER_CRITICAL(&s_parent_mux);
        const bool got = s_parent.valid;
        *out = s_parent;
        *rx_at_ms = s_parent_rx_ms;
        portEXIT_CRITICAL(&s_parent_mux);
        if (got) {
            return true; // early-exit: 1 beacon basta p/ alinhar
        }
    }
    return false;
}

void meter_node_run(void)
{
#if CONFIG_FLOW_TEST_ESPNOW_BEACON_ORIGIN
    // ===== TESTE meter-only: emissor de BEACON por ESP-NOW (simula o root) =====
    ESP_ERROR_CHECK(init_espnow());
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);
    const uint32_t origin_next_sync_ms = (uint32_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000U;
    ESP_LOGW(TAG, "MODO TESTE: emissor BEACON ESP-NOW rank0 next_sync=%lums",
             (unsigned long)origin_next_sync_ms);
    uint32_t origin_seq = 0;
    while (1) {
        flow_beacon_t b;
        flow_sync_build_beacon(&b, s_self_mac, s_self_mac, 0U, ++origin_seq,
                               esp_timer_get_time() / 1000LL,
                               origin_next_sync_ms, origin_next_sync_ms);
        esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&b, sizeof(b));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha TX BEACON ESP-NOW (%s)", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "BEACON ESP-NOW TX seq=%lu rank=0", (unsigned long)origin_seq);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
#else
    flow_power_stay_awake_hatch();

    const uint8_t kind = s_rtc_cold ? WAKE_SCAN : s_rtc_kind;
    if (s_rtc_cold) {
        flow_drift_reset(&s_rtc_drift);
        s_rtc_misses = 0;
        nvs_load_sequence();
    }
    s_rtc_cold = false;
    s_rtc_wakes++;

    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);
    ESP_ERROR_CHECK(init_espnow());

    const int64_t wake_at = esp_timer_get_time() / 1000LL;
    char pstr[18] = {0};
    mac_to_str(s_rtc_parent, pstr, sizeof(pstr));
    ESP_LOGI(TAG, "WAKE #%lu tipo=%d pai=%s rank=%u guard=%lums",
             (unsigned long)s_rtc_wakes, (int)kind, pstr,
             (unsigned)s_rtc_my_rank, (unsigned long)s_rtc_guard_ms);

    if (kind == WAKE_COLLECT) {
        // Relogio fresco (lead burst): transmite direto, sem escutar.
        meter_send_reading();
        const int64_t elapsed_evt =
            esp_timer_get_time() / 1000LL - wake_at - s_rtc_guard_ms;
        uint32_t sync_in = s_rtc_sync_after_collect_ms;
        sync_in = (sync_in > (uint32_t)elapsed_evt)
                      ? sync_in - (uint32_t)elapsed_evt : 60000U;
        meter_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
    }

    // SYNC ou SCAN: escutar o burst do pai.
    const uint32_t budget = (kind == WAKE_SCAN)
        ? CONFIG_FLOW_SCAN_LISTEN_MS
        : 2U * s_rtc_guard_ms + CONFIG_FLOW_BEACON_BURST_MS;
    ESP_LOGI(TAG, "Escutando BEACON por ate %lums...", (unsigned long)budget);

    flow_parent_t p;
    int64_t rx_at = 0;
    if (meter_listen_beacon(budget, &p, &rx_at)) {
        if (kind == WAKE_SYNC) {
            const int32_t err_ms = (int32_t)(rx_at - wake_at) - (int32_t)s_rtc_guard_ms;
            flow_drift_update(&s_rtc_drift, err_ms, s_rtc_slept_ms);
            ESP_LOGI(TAG, "Deriva: err=%ldms sleep=%lums -> %ldppm",
                     (long)err_ms, (unsigned long)s_rtc_slept_ms,
                     (long)s_rtc_drift.ppm);
        }
        s_rtc_misses = 0;
        memcpy(s_rtc_parent, p.parent_id, 6);
        s_rtc_my_rank = p.my_rank;

        mac_to_str(p.parent_id, pstr, sizeof(pstr));
        ESP_LOGI(TAG, "PAI=%s rank=%u rssi=%d next_sync=%lums next_collect=%lums",
                 pstr, (unsigned)p.my_rank, (int)p.best_rssi,
                 (unsigned long)p.next_sync_in_ms,
                 (unsigned long)p.next_collect_in_ms);

        const int64_t since_rx = esp_timer_get_time() / 1000LL - rx_at;
        uint32_t sync_in = (p.next_sync_in_ms > (uint32_t)since_rx)
                               ? p.next_sync_in_ms - (uint32_t)since_rx : 0U;
        uint32_t collect_in = (p.next_collect_in_ms > (uint32_t)since_rx)
                                  ? p.next_collect_in_ms - (uint32_t)since_rx : 0U;
        if (collect_in > 0U && collect_in < sync_in) {
            meter_schedule_and_sleep(WAKE_COLLECT, collect_in, sync_in - collect_in);
        } else {
            meter_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
        }
    } else {
        s_rtc_misses++;
        ESP_LOGW(TAG, "Nenhum BEACON captado (misses=%u)", (unsigned)s_rtc_misses);
        if (s_rtc_misses >= CONFIG_FLOW_SCAN_AFTER_MISSES) {
            uint32_t retry = (uint32_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000U / 4U;
            if (retry < 60000U) {
                retry = 60000U;
            }
            meter_schedule_and_sleep(WAKE_SCAN, retry, 0);
        } else {
            const int64_t elapsed_evt =
                esp_timer_get_time() / 1000LL - wake_at - s_rtc_guard_ms;
            uint32_t sync_in = (uint32_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000U;
            sync_in = (sync_in > (uint32_t)elapsed_evt)
                          ? sync_in - (uint32_t)elapsed_evt : 60000U;
            meter_schedule_and_sleep(WAKE_SYNC, sync_in, 0);
        }
    }
#endif /* CONFIG_FLOW_TEST_ESPNOW_BEACON_ORIGIN */
}

#endif /* CONFIG_FLOW_METER_TX_TEST */
