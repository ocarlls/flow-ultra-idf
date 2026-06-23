#include "meter_tx_test.h"

#if CONFIG_FLOW_METER_TX_TEST

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif
#if !CONFIG_FLOW_METER_LIGHT_SLEEP
#include "esp_sleep.h"
#endif

#include "flow_packet.h"
#include "mesh_dedup.h"

static const char *TAG = "FLOW_TXTEST";

#define FLOW_ESPNOW_CHANNEL 1
#define FLOW_METER_MAX_FORWARD_HOPS 4
#define FLOW_TX_QUEUE_LEN 8

// Wake window / interval do duty-cycle connectionless do WiFi.
// NOTA: a unidade do esp_now_set_wake_window / connectionless_wake_interval e
// proxima de 1 ms (1024 us); para o teste tratamos os valores como ms.
#define FLOW_LS_WINDOW_MS   CONFIG_FLOW_LS_WAKE_WINDOW_MS
#define FLOW_LS_INTERVAL_MS CONFIG_FLOW_LS_WAKE_INTERVAL_MS

static const uint8_t s_bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_self_mac[6] = {0};

static struct {
    uint32_t tx_originated;
    uint32_t tx_sends_total;
    uint32_t tx_send_err;
    uint32_t rx_valid;
    uint32_t rx_relayed;
    uint32_t dups;
} s_stats;

#if CONFIG_FLOW_METER_LIGHT_SLEEP
static mesh_dedup_table_t s_dedup = {0};
static QueueHandle_t s_tx_queue = NULL;
#endif

// ===========================================================================
// Callbacks ESP-NOW
// ===========================================================================
static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
}

#if CONFIG_FLOW_METER_LIGHT_SLEEP
static void on_espnow_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    (void)recv_info;
    if (!data || len != (int)sizeof(flow_packet_t)) {
        return;
    }

    flow_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (!flow_packet_validate_crc32(&pkt)) {
        return;
    }
    if (pkt.type != FLOW_PKT_TYPE_METER_DATA) {
        return;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
        s_stats.dups++;
        return;
    }

    s_stats.rx_valid++;

    if (pkt.hop_count >= FLOW_METER_MAX_FORWARD_HOPS) {
        return;
    }

    // Encaminha em burst (defere para a task de TX; NAO bloquear no callback).
    pkt.hop_count++;
    flow_packet_update_crc32(&pkt);
    if (s_tx_queue != NULL && xQueueSend(s_tx_queue, &pkt, 0) == pdTRUE) {
        s_stats.rx_relayed++;
    }
}
#endif

// ===========================================================================
// Inicializacao Wi-Fi + ESP-NOW (sem deinit no loop)
// ===========================================================================
static esp_err_t init_wifi_espnow(bool register_recv)
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
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(FLOW_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;

    if ((err = esp_now_init()) != ESP_OK) return err;
    if ((err = esp_now_register_send_cb(on_espnow_send)) != ESP_OK) return err;
#if CONFIG_FLOW_METER_LIGHT_SLEEP
    if (register_recv) {
        if ((err = esp_now_register_recv_cb(on_espnow_recv)) != ESP_OK) return err;
    }
#else
    (void)register_recv;
#endif

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_bcast, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        return err;
    }
    return ESP_OK;
}

// ===========================================================================
// Pacote sintetico + burst de transmissao
// ===========================================================================
static void build_synthetic(flow_packet_t *pkt, uint32_t seq)
{
    flow_packet_init_empty(pkt);
    pkt->type = FLOW_PKT_TYPE_METER_DATA;
    pkt->sequence = seq;
    memcpy(pkt->meter_id, s_self_mac, sizeof(pkt->meter_id));
    pkt->volume_liters = seq;   // dummy crescente
    pkt->delta_liters = 1;      // dummy
    pkt->timestamp_ms = esp_timer_get_time() / 1000LL;
    pkt->battery_pct = 100;
    pkt->hop_count = 0;
    flow_packet_update_crc32(pkt);
}

// Repete o mesmo pacote ao longo de >= 1 intervalo de escuta do receptor, para
// cair em alguma janela sem depender de sincronia de relogio entre os nos.
static void transmit_burst(const flow_packet_t *pkt)
{
    int copies = CONFIG_FLOW_LS_TX_BURST_MS / CONFIG_FLOW_LS_TX_SPACING_MS;
    if (copies < 1) {
        copies = 1;
    }
    for (int i = 0; i < copies; ++i) {
        esp_err_t err = esp_now_send(s_bcast, (const uint8_t *)pkt, sizeof(*pkt));
        s_stats.tx_sends_total++;
        if (err != ESP_OK) {
            s_stats.tx_send_err++;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FLOW_LS_TX_SPACING_MS));
    }
}

#if CONFIG_FLOW_LS_VERBOSE
static void log_stats(int64_t *last_us)
{
    const int64_t now = esp_timer_get_time();
    if ((now - *last_us) < ((int64_t)CONFIG_FLOW_LS_STATS_INTERVAL_S * 1000000LL)) {
        return;
    }
    *last_us = now;
    ESP_LOGI(TAG,
             "STATS orig=%lu sends=%lu err=%lu rx=%lu relay=%lu dups=%lu up=%llds",
             (unsigned long)s_stats.tx_originated,
             (unsigned long)s_stats.tx_sends_total,
             (unsigned long)s_stats.tx_send_err,
             (unsigned long)s_stats.rx_valid,
             (unsigned long)s_stats.rx_relayed,
             (unsigned long)s_stats.dups,
             (long long)(now / 1000000LL));
}
#endif

// ===========================================================================
// Variante LIGHT SLEEP: WiFi vivo + connectionless duty-cycle + burst
// ===========================================================================
#if CONFIG_FLOW_METER_LIGHT_SLEEP
static void run_light_sleep(void)
{
    mesh_dedup_init(&s_dedup);
    ESP_ERROR_CHECK(init_wifi_espnow(true));
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    // Power save connectionless: radio acorda FLOW_LS_WINDOW_MS a cada
    // FLOW_LS_INTERVAL_MS (chamar DEPOIS de esp_wifi_start()).
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps falhou: %s", esp_err_to_name(err));
    }
    err = esp_now_set_wake_window(FLOW_LS_WINDOW_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_set_wake_window falhou: %s", esp_err_to_name(err));
    }
    err = esp_wifi_connectionless_module_set_wake_interval(FLOW_LS_INTERVAL_MS);
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
    ESP_LOGW(TAG, "CONFIG_PM_ENABLE desligado: sem automatic light sleep (so modem-sleep)");
#endif

    s_tx_queue = xQueueCreate(FLOW_TX_QUEUE_LEN, sizeof(flow_packet_t));
    if (s_tx_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila de TX");
        return;
    }

    ESP_LOGW(TAG,
             "TXTEST LIGHT SLEEP ativo: window=%dms interval=%dms burst=%dms spacing=%dms period=%dms",
             FLOW_LS_WINDOW_MS, FLOW_LS_INTERVAL_MS,
             CONFIG_FLOW_LS_TX_BURST_MS, CONFIG_FLOW_LS_TX_SPACING_MS,
             CONFIG_FLOW_LS_TX_PERIOD_MS);

    uint32_t seq = 0;
    int64_t last_stats_us = esp_timer_get_time();
    flow_packet_t pkt;

    // Em idle (bloqueado na fila), o FreeRTOS tickless entra em light sleep e o
    // WiFi faz o duty-cycle das janelas de escuta automaticamente.
    while (1) {
        flow_packet_t relay;
        if (xQueueReceive(s_tx_queue, &relay, pdMS_TO_TICKS(CONFIG_FLOW_LS_TX_PERIOD_MS)) == pdTRUE) {
            transmit_burst(&relay);
        } else {
            build_synthetic(&pkt, ++seq);
            // Registra o proprio pacote no dedup para nao reencaminhar o eco.
            (void)mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence,
                                          pkt.type, pkt.timestamp_ms);
            transmit_burst(&pkt);
            s_stats.tx_originated++;
        }
#if CONFIG_FLOW_LS_VERBOSE
        log_stats(&last_stats_us);
#else
        (void)last_stats_us;
#endif
    }
}
#endif /* CONFIG_FLOW_METER_LIGHT_SLEEP */

// ===========================================================================
// Variante BASELINE: deep sleep so-transmissao (transmissor cego)
// ===========================================================================
#if !CONFIG_FLOW_METER_LIGHT_SLEEP
RTC_DATA_ATTR static uint32_t s_rtc_seq = 0;

static void run_deep_sleep_baseline(void)
{
    ESP_ERROR_CHECK(init_wifi_espnow(false));
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    flow_packet_t pkt;
    build_synthetic(&pkt, ++s_rtc_seq);

    // Burst cobrindo um intervalo, para um receptor duty-cycled captar.
    transmit_burst(&pkt);
    s_stats.tx_originated++;

#if CONFIG_FLOW_LS_VERBOSE
    ESP_LOGI(TAG, "TXTEST DEEP baseline: seq=%lu sends=%lu err=%lu -> sleep %dms",
             (unsigned long)s_rtc_seq,
             (unsigned long)s_stats.tx_sends_total,
             (unsigned long)s_stats.tx_send_err,
             CONFIG_FLOW_LS_TX_PERIOD_MS);
#endif

    esp_wifi_stop();
    esp_wifi_deinit();

    esp_sleep_enable_timer_wakeup((uint64_t)CONFIG_FLOW_LS_TX_PERIOD_MS * 1000ULL);
    esp_deep_sleep_start();
    // nunca retorna
}
#endif /* !CONFIG_FLOW_METER_LIGHT_SLEEP */

// ===========================================================================
// Entry point
// ===========================================================================
void meter_tx_test_run(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
#if CONFIG_FLOW_METER_LIGHT_SLEEP
    run_light_sleep();
#else
    run_deep_sleep_baseline();
#endif
}

#endif /* CONFIG_FLOW_METER_TX_TEST */
