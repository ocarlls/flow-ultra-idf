#include "sdkconfig.h"
#include "meter_tx_test.h"

#if CONFIG_FLOW_METER_TX_TEST

#include <string.h>

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

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "flow_packet.h"
#include "mesh_dedup.h"

static const char *TAG = "FLOW_NODE";

#define FLOW_ESPNOW_CHANNEL         1
#define FLOW_METER_MAX_FORWARD_HOPS 4
#define FLOW_TX_QUEUE_LEN           8

// Wake-on-radio por WiFi: o radio acorda FLOW_LS_WINDOW_MS a cada
// FLOW_LS_INTERVAL_MS para escutar ESP-NOW connectionless. A unidade real do
// esp_now_set_wake_window / connectionless_wake_interval e proxima de 1 ms
// (1024 us); aqui tratamos os valores como ms.
#define FLOW_LS_WINDOW_MS   CONFIG_FLOW_LS_WAKE_WINDOW_MS
#define FLOW_LS_INTERVAL_MS CONFIG_FLOW_LS_WAKE_INTERVAL_MS

static const uint8_t s_bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t s_self_mac[6] = {0};

static struct {
    uint32_t tx_sends_total;   // copias enviadas (relay em burst)
    uint32_t tx_send_err;
    uint32_t rx_valid;         // pacotes validos recebidos
    uint32_t rx_relayed;       // pacotes enfileirados para reencaminhamento
    uint32_t dups;             // descartados por dedup
} s_stats;

static mesh_dedup_table_t s_dedup = {0};
static QueueHandle_t s_tx_queue = NULL;

// ===========================================================================
// Callbacks ESP-NOW
// ===========================================================================
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

// Chamado quando o radio capta um pacote durante a janela de escuta. NAO
// bloquear aqui: valida, deduplica e defere o relay para a task principal.
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

    // Reencaminha em burst (defere para a task de relay; NAO bloquear no callback).
    pkt.hop_count++;
    flow_packet_update_crc32(&pkt);
    if (s_tx_queue != NULL && xQueueSend(s_tx_queue, &pkt, 0) == pdTRUE) {
        s_stats.rx_relayed++;
    }
}

// ===========================================================================
// Inicializacao Wi-Fi + ESP-NOW (sobe uma vez e fica vivo o tempo todo)
// ===========================================================================
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
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(FLOW_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;

    if ((err = esp_now_init()) != ESP_OK) return err;
    if ((err = esp_now_register_send_cb(on_espnow_send)) != ESP_OK) return err;
    if ((err = esp_now_register_recv_cb(on_espnow_recv)) != ESP_OK) return err;

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
// Relay em burst: repete o pacote ao longo de >= 1 intervalo de escuta do
// proximo node, para cair em alguma janela sem depender de sincronia de relogio.
// ===========================================================================
static void relay_burst(const flow_packet_t *pkt)
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

// ===========================================================================
// Entry point — node em LIGHT SLEEP com WiFi ligado.
// Fica dormindo ate o radio captar um pacote; ao receber, acorda, reencaminha
// para o proximo node e volta a dormir. Nunca origina dados proprios.
// ===========================================================================
void meter_tx_test_run(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    mesh_dedup_init(&s_dedup);

    s_tx_queue = xQueueCreate(FLOW_TX_QUEUE_LEN, sizeof(flow_packet_t));
    if (s_tx_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila de relay");
        return;
    }

    ESP_ERROR_CHECK(init_wifi_espnow());
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    // Power save connectionless: o radio acorda FLOW_LS_WINDOW_MS a cada
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
    // Light sleep automatico: no idle do FreeRTOS (task bloqueada na fila), a CPU
    // entra em light sleep; o WiFi faz o duty-cycle das janelas de escuta sozinho.
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
    ESP_LOGE(TAG, "CONFIG_PM_ENABLE desligado: o node NAO entra em light sleep "
                  "(so modem-sleep). Habilite CONFIG_PM_ENABLE + "
                  "CONFIG_FREERTOS_USE_TICKLESS_IDLE.");
#endif

    ESP_LOGW(TAG, "NODE relay em light sleep: window=%dms interval=%dms burst=%dms spacing=%dms",
             FLOW_LS_WINDOW_MS, FLOW_LS_INTERVAL_MS,
             CONFIG_FLOW_LS_TX_BURST_MS, CONFIG_FLOW_LS_TX_SPACING_MS);

    // Bloqueia indefinidamente na fila: sem pacote, o tickless idle mantem a CPU
    // em light sleep. Ao chegar um relay (enfileirado pelo callback de RX), acorda,
    // retransmite em burst e volta a dormir.
    flow_packet_t relay;
    while (1) {
        if (xQueueReceive(s_tx_queue, &relay, portMAX_DELAY) == pdTRUE) {
            relay_burst(&relay);
#if CONFIG_FLOW_LS_VERBOSE
            ESP_LOGI(TAG,
                     "RELAY meter=%02X%02X%02X%02X%02X%02X seq=%lu hop=%u | "
                     "rx=%lu relay=%lu dups=%lu sends=%lu err=%lu",
                     relay.meter_id[0], relay.meter_id[1], relay.meter_id[2],
                     relay.meter_id[3], relay.meter_id[4], relay.meter_id[5],
                     (unsigned long)relay.sequence, (unsigned)relay.hop_count,
                     (unsigned long)s_stats.rx_valid,
                     (unsigned long)s_stats.rx_relayed,
                     (unsigned long)s_stats.dups,
                     (unsigned long)s_stats.tx_sends_total,
                     (unsigned long)s_stats.tx_send_err);
#endif
        }
    }
}

#endif /* CONFIG_FLOW_METER_TX_TEST */
