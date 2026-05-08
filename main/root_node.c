#include "root_node.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "flow_packet.h"
#include "lora_test_config.h"
#include "mesh_dedup.h"
#include "sx1276_lora.h"

static const char *TAG = "FLOW_ROOT";

#define FLOW_LORA_MAX_ROUTE_LEN 4

static mesh_dedup_table_t s_dedup = {0};

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

static void log_packet(const flow_packet_t *pkt, bool crc_ok)
{
    char meter[18] = {0};
    char subroot[18] = {0};
    mac_to_str(pkt->meter_id, meter, sizeof(meter));
    mac_to_str(pkt->subroot_mac, subroot, sizeof(subroot));

    char route_buf[128] = {0};
    size_t off = 0;
    for (uint8_t i = 0; i < pkt->route_len && i < FLOW_LORA_MAX_ROUTE_LEN; ++i) {
        char hop[18] = {0};
        mac_to_str(pkt->route_path[i], hop, sizeof(hop));
        int wrote = snprintf(&route_buf[off], sizeof(route_buf) - off,
                             "%s%s", (i == 0) ? "" : "->", hop);
        if (wrote <= 0) {
            break;
        }
        off += (size_t)wrote;
        if (off >= sizeof(route_buf)) {
            break;
        }
    }

    ESP_LOGI(TAG,
             "RX meter=%s seq=%lu vol=%luL delta=%luL ts=%lld bat=%u%% hop=%u espnow_rssi=%d lora_rssi=%d lora_snr=%d subroot=%s route_len=%u route=%s crc=%s",
             meter,
             (unsigned long)pkt->sequence,
             (unsigned long)pkt->volume_liters,
             (unsigned long)pkt->delta_liters,
             (long long)pkt->timestamp_ms,
             (unsigned)pkt->battery_pct,
             (unsigned)pkt->hop_count,
             (int)pkt->espnow_rssi,
             (int)pkt->lora_rssi,
             (int)pkt->lora_snr,
             subroot,
             (unsigned)pkt->route_len,
             route_buf,
             crc_ok ? "OK" : "BAD");
}

static void send_ack(const flow_packet_t *rx_pkt)
{
    flow_packet_t ack = *rx_pkt;
    ack.type = FLOW_PKT_TYPE_ACK;
    ack.timestamp_ms = esp_timer_get_time() / 1000LL;
    ack.espnow_rssi = INT8_MIN;
    ack.lora_rssi = INT8_MIN;
    ack.lora_snr = INT8_MIN;

    flow_packet_update_crc32(&ack);
    (void)sx1276_lora_transmit((const uint8_t *)&ack, sizeof(ack));
}

void root_node_run(void)
{
    mesh_dedup_init(&s_dedup);
    ESP_ERROR_CHECK(init_lora_radio());

    ESP_LOGW(TAG, "MODO FLOW MESH: ROOT ativo (LoRa %uMHz)", (unsigned)CONFIG_LORA_TEST_FREQUENCY_MHZ);

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

        const bool crc_ok = flow_packet_validate_crc32(&pkt);
        if (!crc_ok) {
            continue;
        }

        if (pkt.type != FLOW_PKT_TYPE_METER_DATA) {
            continue;
        }

        const int64_t now_ms = esp_timer_get_time() / 1000LL;
        if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
            continue;
        }

        // Atualiza metricas do ultimo hop LoRa para log.
        pkt.lora_rssi = clamp_i16_to_i8(event.rssi_dbm);
        pkt.lora_snr = event.snr_db;

        log_packet(&pkt, true);
        send_ack(&pkt);
    }
}
