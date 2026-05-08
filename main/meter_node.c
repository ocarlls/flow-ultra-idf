#include "meter_node.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "flow_packet.h"
#include "mesh_dedup.h"

static const char *TAG = "FLOW_METER";

#define FLOW_ESPNOW_CHANNEL 1
#define FLOW_METER_SEND_INTERVAL_MS 5000
#define FLOW_METER_MAX_FORWARD_HOPS 4
#define FLOW_METER_FIXED_VOLUME_LITERS 1234U
#define FLOW_METER_FIXED_DELTA_LITERS 7U
#define FLOW_METER_DEFAULT_BATTERY_PCT 100U

#define FLOW_METER_NVS_NS       "flow_mesh"
#define FLOW_METER_NVS_KEY_SEQ  "seq"

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_self_mac[6] = {0};
static mesh_dedup_table_t s_dedup = {0};
static uint32_t s_sequence = 0;

static esp_err_t nvs_load_sequence(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t seq = 0;
    err = nvs_get_u32(h, FLOW_METER_NVS_KEY_SEQ, &seq);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        seq = 0;
        err = ESP_OK;
    }

    if (err == ESP_OK) {
        s_sequence = seq;
    }

    nvs_close(h);
    return err;
}

static esp_err_t nvs_store_sequence(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(FLOW_METER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u32(h, FLOW_METER_NVS_KEY_SEQ, s_sequence);
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

static esp_err_t init_wifi_espnow(void);

static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
}

static void on_espnow_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    (void)recv_info;

    if (data == NULL || len != (int)sizeof(flow_packet_t)) {
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

    if (pkt.hop_count >= FLOW_METER_MAX_FORWARD_HOPS) {
        return;
    }

    pkt.hop_count++;
    flow_packet_update_crc32(&pkt);
    (void)esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
}

static void meter_broadcast_task(void *arg)
{
    (void)arg;

    while (1) {
        flow_packet_t pkt;
        flow_packet_init_empty(&pkt);

        pkt.type = FLOW_PKT_TYPE_METER_DATA;
        // A sequencia continua monotônica para a malha não descartar os envios como duplicados.
        pkt.sequence = s_sequence + 1U;
        memcpy(pkt.meter_id, s_self_mac, sizeof(pkt.meter_id));
        pkt.volume_liters = FLOW_METER_FIXED_VOLUME_LITERS;
        pkt.delta_liters = FLOW_METER_FIXED_DELTA_LITERS;
        pkt.timestamp_ms = esp_timer_get_time() / 1000LL;
        pkt.battery_pct = FLOW_METER_DEFAULT_BATTERY_PCT;
        pkt.hop_count = 0;

        flow_packet_update_crc32(&pkt);

        // Registra como visto para evitar re-forward do proprio broadcast.
        const int64_t now_ms = pkt.timestamp_ms;
        (void)mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms);

        esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao enviar ESP-NOW (%s)", esp_err_to_name(err));
        } else {
            s_sequence = pkt.sequence;
            if (nvs_store_sequence() != ESP_OK) {
                ESP_LOGW(TAG, "Falha ao persistir sequencia do medidor");
            }
            ESP_LOGI(TAG,
                     "TX TESTE seq=%lu vol=%luL delta=%luL",
                     (unsigned long)pkt.sequence,
                     (unsigned long)pkt.volume_liters,
                     (unsigned long)pkt.delta_liters);
        }

        vTaskDelay(pdMS_TO_TICKS(FLOW_METER_SEND_INTERVAL_MS));
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

    // Peer broadcast
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

void meter_node_run(void)
{
    mesh_dedup_init(&s_dedup);

    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    if (nvs_load_sequence() != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao carregar sequencia do medidor; iniciando do zero");
        s_sequence = 0;
    }

    ESP_ERROR_CHECK(init_wifi_espnow());

    ESP_LOGW(TAG,
             "MODO FLOW MESH: METER ativo em teste de conectividade (canal=%d, payload fixo vol=%uL delta=%uL)",
             FLOW_ESPNOW_CHANNEL,
             (unsigned)FLOW_METER_FIXED_VOLUME_LITERS,
             (unsigned)FLOW_METER_FIXED_DELTA_LITERS);

    xTaskCreate(meter_broadcast_task, "flow_meter_tx", 4096, NULL, 6, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
