#include "root_node.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "flow_packet.h"
#include "flow_sync.h"
#include "lora_test_config.h"
#include "mesh_dedup.h"
#include "e220_lora.h"

static const char *TAG = "FLOW_ROOT";

#define FLOW_LORA_MAX_ROUTE_LEN 4
#define FLOW_ACK_DELAY_MS 150

// A coleta e sempre precedida de um burst de sync ("lead"): quem captar esse
// burst chega a janela de coleta com relogio fresco (deriva de segundos, nao
// de horas), permitindo janela curta mesmo sem cristal.
#define FLOW_COLLECT_LEAD_MS ((int64_t)CONFIG_FLOW_BEACON_BURST_MS + 5000)

static mesh_dedup_table_t s_dedup = {0};
static uint8_t s_self_mac[6] = {0};
static uint32_t s_sync_seq = 0;

// Agenda (uptime ms; RTC/GPS futuramente ancora o 16:30 civil).
static int64_t s_next_sync_at = 0;
static int64_t s_next_collect_at = 0;
static int64_t s_burst_until = 0;
static int64_t s_last_beacon_at = 0;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000LL;
}

// Emite 1 BEACON (rank 0) por LoRa com a agenda corrente embutida.
// root_epoch_ms e placeholder (esp_timer) ate a integracao de RTC/GPS.
static void send_beacon(void)
{
    const int64_t now = now_ms();

    // Proximo burst na visao dos filhos: o sync regular ou o lead da coleta,
    // o que vier antes.
    int64_t next_burst_at = s_next_sync_at;
    const int64_t lead_at = s_next_collect_at - FLOW_COLLECT_LEAD_MS;
    if (lead_at > now && lead_at < next_burst_at) {
        next_burst_at = lead_at;
    }

    flow_beacon_t b;
    flow_sync_build_beacon(&b, s_self_mac, s_self_mac, /*my_rank=*/0,
                           s_sync_seq, /*root_epoch_ms=*/now,
                           (uint32_t)(s_next_collect_at - now),
                           (uint32_t)(next_burst_at - now));
    esp_err_t err = e220_lora_transmit((const uint8_t *)&b, sizeof(b));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha TX BEACON (%s)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "BEACON TX seq=%lu rank=0 next_sync=%llds next_collect=%llds",
                 (unsigned long)s_sync_seq,
                 (long long)((next_burst_at - now) / 1000),
                 (long long)((s_next_collect_at - now) / 1000));
    }
    s_last_beacon_at = now;
}

// Agendador de beacon: bursts densos nas janelas de sync (+ lead da coleta),
// beacon esparso de descoberta fora delas, silencio na janela de coleta.
static void beacon_scheduler_tick(void)
{
    const int64_t now = now_ms();
    const bool in_collect_window =
        (now >= s_next_collect_at) &&
        (now < s_next_collect_at + (int64_t)CONFIG_FLOW_COLLECT_WINDOW_MS);

    if (now >= s_next_collect_at + (int64_t)CONFIG_FLOW_COLLECT_WINDOW_MS) {
        s_next_collect_at += (int64_t)CONFIG_FLOW_COLLECT_PERIOD_S * 1000;
        while (s_next_collect_at <= now) {
            s_next_collect_at += (int64_t)CONFIG_FLOW_COLLECT_PERIOD_S * 1000;
        }
    }

    if (now < s_burst_until) {
        // Burst em andamento: repete a cada SPACING.
        if (now - s_last_beacon_at >= CONFIG_FLOW_BEACON_SPACING_MS) {
            send_beacon();
        }
        return;
    }

    // Inicia burst? (nunca dentro da janela de coleta)
    const int64_t lead_at = s_next_collect_at - FLOW_COLLECT_LEAD_MS;
    const bool lead_burst_fits =
        (now >= lead_at) && (now + CONFIG_FLOW_BEACON_BURST_MS <= s_next_collect_at);
    if (!in_collect_window && (now >= s_next_sync_at || lead_burst_fits)) {
        s_burst_until = now + CONFIG_FLOW_BEACON_BURST_MS;
        s_sync_seq++;  // 1 seq por burst: filhos deduplicam o rebroadcast
        if (now >= s_next_sync_at) {
            s_next_sync_at += (int64_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000;
            while (s_next_sync_at <= now) {
                s_next_sync_at += (int64_t)CONFIG_FLOW_RESYNC_PERIOD_S * 1000;
            }
        }
        ESP_LOGI(TAG, "SYNC BURST inicio (seq=%lu, %dms)",
                 (unsigned long)s_sync_seq, CONFIG_FLOW_BEACON_BURST_MS);
        send_beacon();
        return;
    }

    // Descoberta esparsa (mesmo seq: subroots nao rebroadcastam).
    if (!in_collect_window &&
        now - s_last_beacon_at >= (int64_t)CONFIG_FLOW_DISCOVERY_PERIOD_S * 1000) {
        send_beacon();
    }
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

static esp_err_t init_lora_radio(void)
{
    e220_lora_config_t config = {
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
    vTaskDelay(pdMS_TO_TICKS(FLOW_ACK_DELAY_MS));
    esp_err_t err = e220_lora_transmit((const uint8_t *)&ack, sizeof(ack));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ACK TX seq=%lu subroot=%02X:%02X:%02X:%02X:%02X:%02X",
                 (unsigned long)ack.sequence,
                 ack.subroot_mac[0], ack.subroot_mac[1], ack.subroot_mac[2],
                 ack.subroot_mac[3], ack.subroot_mac[4], ack.subroot_mac[5]);
    } else {
        ESP_LOGW(TAG, "Falha ao iniciar ACK seq=%lu (%s)",
                 (unsigned long)ack.sequence,
                 esp_err_to_name(err));
    }
}

void root_node_run(void)
{
    mesh_dedup_init(&s_dedup);
    (void)esp_read_mac(s_self_mac, ESP_MAC_WIFI_STA);
    ESP_ERROR_CHECK(init_lora_radio());

    ESP_LOGW(TAG,
             "MODO FLOW MESH: ROOT ativo (LoRa %uMHz) — sync a cada %ds, coleta a cada %ds, descoberta a cada %ds",
             (unsigned)CONFIG_LORA_TEST_FREQUENCY_MHZ,
             CONFIG_FLOW_RESYNC_PERIOD_S, CONFIG_FLOW_COLLECT_PERIOD_S,
             CONFIG_FLOW_DISCOVERY_PERIOD_S);

    {
        const int64_t now = now_ms();
        s_next_sync_at = now + 5000;  // primeiro burst logo apos o boot
        s_next_collect_at = now + (int64_t)CONFIG_FLOW_COLLECT_PERIOD_S * 1000;
        s_last_beacon_at = now - (int64_t)CONFIG_FLOW_DISCOVERY_PERIOD_S * 1000;
    }

    while (1) {
        beacon_scheduler_tick();

        // Timeout curto durante burst p/ respeitar o espacamento dos beacons.
        const TickType_t rx_timeout =
            (now_ms() < s_burst_until) ? pdMS_TO_TICKS(50) : pdMS_TO_TICKS(200);

        e220_lora_event_t event = {0};
        esp_err_t err = e220_lora_receive_event(&event, rx_timeout);
        if (err == ESP_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (event.type != E220_LORA_EVENT_RX_DONE) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (event.payload_len != sizeof(flow_packet_t)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        flow_packet_t pkt;
        memcpy(&pkt, event.payload, sizeof(pkt));

        if (!flow_packet_header_ok(&pkt)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const bool crc_ok = flow_packet_validate_crc32(&pkt);
        if (!crc_ok) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (pkt.type != FLOW_PKT_TYPE_METER_DATA) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const int64_t now_ms = esp_timer_get_time() / 1000LL;
        if (mesh_dedup_is_duplicate(&s_dedup, pkt.meter_id, pkt.sequence, pkt.type, now_ms)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Atualiza metricas do ultimo hop LoRa para log.
        pkt.lora_rssi = clamp_i16_to_i8(event.rssi_dbm);
        pkt.lora_snr = event.snr_db;

        log_packet(&pkt, true);
        send_ack(&pkt);
    }
}
