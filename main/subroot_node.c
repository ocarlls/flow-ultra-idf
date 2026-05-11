#include "subroot_node.h"

#include <stdio.h>
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
#include "freertos/queue.h"
#include "freertos/task.h"

#include "flow_packet.h"
#include "lora_test_config.h"
#include "mesh_dedup.h"
#include "sx1276_lora.h"

static const char *TAG = "FLOW_SUBROOT";

#define FLOW_ESPNOW_CHANNEL 1
#define FLOW_LORA_MAX_ROUTE_LEN 4

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_self_mac[6] = {0};
static mesh_dedup_table_t s_dedup = {0};
static uint32_t s_probe_sequence = 0;

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

static void mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static esp_err_t init_wifi_espnow(void);
static esp_err_t init_lora_radio(void);

static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;
}

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

        esp_err_t err = sx1276_lora_transmit((const uint8_t *)&pkt, sizeof(pkt));
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

static void maybe_forward_lora_meter_data(const sx1276_lora_event_t *event, const flow_packet_t *rx_pkt)
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

static void maybe_forward_lora_ack(const sx1276_lora_event_t *event, const flow_packet_t *rx_pkt)
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

static void lora_rx_task(void *arg)
{
    (void)arg;

    while (1) {
        sx1276_lora_event_t event = {0};
        esp_err_t err = sx1276_lora_receive_event(&event, pdMS_TO_TICKS(200));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            continue;
        }
        if (event.type != SX1276_LORA_EVENT_RX_DONE) {
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

static esp_err_t init_lora_radio(void)
{
    sx1276_lora_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_sck = LORA_TEST_PIN_SCK,
        .pin_miso = LORA_TEST_PIN_MISO,
        .pin_mosi = LORA_TEST_PIN_MOSI,
        .pin_cs = LORA_TEST_PIN_CS,
        .pin_rst = LORA_TEST_PIN_RST,
        .pin_dio0 = LORA_TEST_PIN_DIO0,
        .frequency_hz = LORA_TEST_FREQUENCY_HZ,
        .bandwidth_hz = LORA_TEST_BANDWIDTH_HZ,
        .spreading_factor = CONFIG_LORA_TEST_SPREADING_FACTOR,
        .tx_power_dbm = CONFIG_LORA_TEST_TX_POWER_DBM,
        .coding_rate = CONFIG_LORA_TEST_CODING_RATE,
        .preamble_len = CONFIG_LORA_TEST_PREAMBLE_LEN,
    };

    ESP_RETURN_ON_ERROR(sx1276_lora_init(&config), TAG, "sx1276 init");
    sx1276_lora_flush_events();
    return sx1276_lora_start_rx_continuous();
}

void subroot_node_run(void)
{
    mesh_dedup_init(&s_dedup);
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);

    ESP_ERROR_CHECK(init_wifi_espnow());
    ESP_ERROR_CHECK(init_lora_radio());

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
