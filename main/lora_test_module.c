#include "lora_test_module.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "e220_lora.h"

#define LORA_TEST_TX_DONE_TIMEOUT_MS 6000U
#define LORA_TEST_ACK_TIMEOUT_MS     (LORA_TEST_TX_DONE_TIMEOUT_MS + 2000U)
#define LORA_TEST_MAX_NODES          32

typedef enum {
    LORA_TEST_FRAME_TYPE_PROBE = 0xA1,
    LORA_TEST_FRAME_TYPE_ACK   = 0xA2,
} lora_test_frame_type_t;

typedef enum {
    LORA_TEST_NODE_ACK_STATUS_IDLE    = 0,
    LORA_TEST_NODE_ACK_STATUS_TXWAIT,
    LORA_TEST_NODE_ACK_STATUS_WAITACK,
    LORA_TEST_NODE_ACK_STATUS_OK,
    LORA_TEST_NODE_ACK_STATUS_TXFAIL,
    LORA_TEST_NODE_ACK_STATUS_TDOUT,
    LORA_TEST_NODE_ACK_STATUS_TIMEOUT,
    LORA_TEST_NODE_ACK_STATUS_ERROR,
} lora_test_node_ack_status_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  mac[6];
    uint32_t seq;
    uint32_t uptime_ms;
} lora_probe_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  mac[6];
    uint32_t seq;
    int16_t  probe_rssi_dbm;
    int8_t   probe_snr_db;
    int8_t   reserved;
    uint32_t uptime_ms;
} lora_ack_frame_t;

typedef struct {
    bool     in_use;
    bool     has_first_seq;
    bool     has_last_seq;
    uint8_t  mac[6];
    uint32_t first_seq;
    uint32_t last_seq;
    uint32_t rx_count;
    uint32_t unique_rx_count;
    uint32_t duplicate_count;
    uint32_t ack_ok_count;
    uint32_t ack_fail_count;
    int16_t  last_rssi_dbm;
    int8_t   last_snr_db;
} lora_node_stats_t;

typedef struct {
    uint32_t                    tx_count;
    uint32_t                    ack_ok_count;
    uint32_t                    ack_fail_count;
    uint32_t                    last_seq;
    int16_t                     last_tx_rssi_dbm;
    int16_t                     last_rx_rssi_dbm;
    int8_t                      last_snr_db;
    lora_test_node_ack_status_t last_ack_status;
} lora_node_view_t;

static const char       *TAG  = "LORA_TEST";
static lora_node_stats_t s_nodes[LORA_TEST_MAX_NODES] = {0};
static lora_node_stats_t *s_last_active_node          = NULL;
static lora_node_view_t   s_node_view = {
    .last_tx_rssi_dbm = INT16_MIN,
    .last_rx_rssi_dbm = INT16_MIN,
    .last_snr_db      = INT8_MIN,
    .last_ack_status  = LORA_TEST_NODE_ACK_STATUS_IDLE,
};

static void mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool mac_equal(const uint8_t left[6], const uint8_t right[6])
{
    return memcmp(left, right, 6) == 0;
}

static void reset_runtime_state(void)
{
    memset(s_nodes, 0, sizeof(s_nodes));
    s_last_active_node = NULL;
    memset(&s_node_view, 0, sizeof(s_node_view));
    s_node_view.last_tx_rssi_dbm = INT16_MIN;
    s_node_view.last_rx_rssi_dbm = INT16_MIN;
    s_node_view.last_snr_db      = INT8_MIN;
    s_node_view.last_ack_status  = LORA_TEST_NODE_ACK_STATUS_IDLE;
}

static uint32_t compute_percent_tenths(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U) {
        return 0U;
    }
    uint64_t scaled = (((uint64_t)numerator * 1000ULL) + ((uint64_t)denominator / 2ULL)) /
                      (uint64_t)denominator;
    if (scaled > 1000ULL) {
        scaled = 1000ULL;
    }
    return (uint32_t)scaled;
}

static uint32_t root_expected_packets(const lora_node_stats_t *node)
{
    if (node == NULL || !node->has_first_seq || !node->has_last_seq ||
        node->last_seq < node->first_seq) {
        return 0U;
    }
    return node->last_seq - node->first_seq + 1U;
}

static void format_dbm_value(int16_t value, bool valid, char *buffer, size_t buffer_len)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--");
        return;
    }
    snprintf(buffer, buffer_len, "%ddBm", (int)value);
}

static void format_db_value(int value, bool valid, char *buffer, size_t buffer_len)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--");
        return;
    }
    snprintf(buffer, buffer_len, "%ddB", value);
}

static void format_percent_value(uint32_t numerator, uint32_t denominator,
                                 char *buffer, size_t buffer_len)
{
    uint32_t pt = compute_percent_tenths(numerator, denominator);
    snprintf(buffer, buffer_len, "%lu.%lu%%",
             (unsigned long)(pt / 10U),
             (unsigned long)(pt % 10U));
}

static lora_node_stats_t *find_or_allocate_node(const uint8_t mac[6])
{
    for (size_t i = 0; i < LORA_TEST_MAX_NODES; ++i) {
        if (s_nodes[i].in_use && mac_equal(s_nodes[i].mac, mac)) {
            return &s_nodes[i];
        }
    }
    for (size_t i = 0; i < LORA_TEST_MAX_NODES; ++i) {
        if (!s_nodes[i].in_use) {
            memset(&s_nodes[i], 0, sizeof(s_nodes[i]));
            s_nodes[i].in_use = true;
            memcpy(s_nodes[i].mac, mac, sizeof(s_nodes[i].mac));
            return &s_nodes[i];
        }
    }
    return NULL;
}

static uint32_t uptime_ms_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint32_t next_delay_ms(uint32_t base_ms, uint32_t jitter_ms)
{
    if (jitter_ms == 0U) {
        return base_ms;
    }
    uint32_t span     = (jitter_ms * 2U) + 1U;
    int32_t  offset   = (int32_t)(esp_random() % span) - (int32_t)jitter_ms;
    int32_t  candidate = (int32_t)base_ms + offset;
    if (candidate < 100) {
        candidate = 100;
    }
    return (uint32_t)candidate;
}

static esp_err_t wait_for_tx_done(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);

    while (esp_timer_get_time() < deadline_us) {
        e220_lora_event_t event    = {0};
        int64_t           rem_ms   = (deadline_us - esp_timer_get_time()) / 1000LL;
        TickType_t        wait     = pdMS_TO_TICKS((rem_ms > 50) ? 50 : (rem_ms > 0 ? rem_ms : 1));
        esp_err_t         err      = e220_lora_receive_event(&event, wait);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }
        if (event.type == E220_LORA_EVENT_TX_DONE) {
            return ESP_OK;
        }
    }

    (void)e220_lora_start_rx_continuous();
    return ESP_ERR_TIMEOUT;
}

static void log_config_banner(const uint8_t self_mac[6])
{
    char mac_buffer[18] = {0};
    mac_to_str(self_mac, mac_buffer, sizeof(mac_buffer));

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "MODO TESTE LORA ATIVO (EBYTE E220)");
    ESP_LOGW(TAG, "Role: %s", CONFIG_LORA_TEST_ROLE_ROOT ? "ROOT" : "NODE");
    ESP_LOGW(TAG, "MAC local: %s", mac_buffer);
    ESP_LOGW(TAG, "Addr=0x%04X Canal=%u AirRate=%u TX=%d dBm WOR=%u ms RSSI_byte=%s",
             (unsigned)CONFIG_FLOW_E220_ADDRESS,
             (unsigned)CONFIG_FLOW_E220_CHANNEL,
             (unsigned)CONFIG_FLOW_E220_AIR_DATA_RATE,
             (int)CONFIG_FLOW_E220_TX_POWER_DBM,
             (unsigned)CONFIG_FLOW_E220_WOR_PERIOD_MS,
             CONFIG_FLOW_E220_RSSI_BYTE ? "on" : "off");
    ESP_LOGW(TAG, "UART%d TX=%d RX=%d M0=%d M1=%d AUX=%d baud=%d",
             CONFIG_FLOW_E220_UART_PORT,
             E220_PIN_TX, E220_PIN_RX,
             E220_PIN_M0, E220_PIN_M1, E220_PIN_AUX,
             CONFIG_FLOW_E220_BAUD);
    ESP_LOGW(TAG, "========================================");
}

static void log_root_summary(void)
{
    ESP_LOGI(TAG, "Resumo ROOT:");
    for (size_t i = 0; i < LORA_TEST_MAX_NODES; ++i) {
        if (!s_nodes[i].in_use) {
            continue;
        }
        char     mac_buffer[18] = {0};
        char     pdr_buffer[16] = {0};
        uint32_t expected       = root_expected_packets(&s_nodes[i]);
        mac_to_str(s_nodes[i].mac, mac_buffer, sizeof(mac_buffer));
        format_percent_value(s_nodes[i].unique_rx_count, expected, pdr_buffer, sizeof(pdr_buffer));
        ESP_LOGI(TAG,
                 "  mac=%s pkt=%lu dup=%lu ack_ok=%lu ack_fail=%lu seq=%lu pdr=%s rssi=%d snr=%d",
                 mac_buffer,
                 (unsigned long)s_nodes[i].unique_rx_count,
                 (unsigned long)s_nodes[i].duplicate_count,
                 (unsigned long)s_nodes[i].ack_ok_count,
                 (unsigned long)s_nodes[i].ack_fail_count,
                 (unsigned long)s_nodes[i].last_seq,
                 pdr_buffer,
                 (int)s_nodes[i].last_rssi_dbm,
                 (int)s_nodes[i].last_snr_db);
    }
}

static esp_err_t init_lora_radio(void)
{
    e220_lora_config_t config = {
        .uart_port    = CONFIG_FLOW_E220_UART_PORT,
        .pin_tx       = E220_PIN_TX,
        .pin_rx       = E220_PIN_RX,
        .pin_m0       = E220_PIN_M0,
        .pin_m1       = E220_PIN_M1,
        .pin_aux      = E220_PIN_AUX,
        .baud         = CONFIG_FLOW_E220_BAUD,
        .address      = (uint16_t)CONFIG_FLOW_E220_ADDRESS,
        .channel      = (uint8_t)CONFIG_FLOW_E220_CHANNEL,
        .air_data_rate = (uint8_t)CONFIG_FLOW_E220_AIR_DATA_RATE,
        .tx_power_dbm = (int8_t)CONFIG_FLOW_E220_TX_POWER_DBM,
        .wor_period_ms = (uint16_t)CONFIG_FLOW_E220_WOR_PERIOD_MS,
        .fixed_mode   = CONFIG_FLOW_E220_FIXED_MODE,
        .rssi_byte    = CONFIG_FLOW_E220_RSSI_BYTE,
        .frequency_hz = LORA_TEST_FREQUENCY_HZ,
    };

    esp_err_t err = e220_lora_init(&config);
    if (err != ESP_OK) {
        return err;
    }
    e220_lora_debug_dump();
    return e220_lora_start_rx_continuous();
}

static void __attribute__((unused)) run_root(const uint8_t self_mac[6])
{
    (void)self_mac;
    int64_t next_summary_us = esp_timer_get_time() +
                              ((int64_t)CONFIG_LORA_TEST_ROOT_SUMMARY_INTERVAL_MS * 1000LL);

    while (1) {
        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_summary_us) {
            log_root_summary();
            next_summary_us = now_us + ((int64_t)CONFIG_LORA_TEST_ROOT_SUMMARY_INTERVAL_MS * 1000LL);
        }

        int64_t    rem_ms = (next_summary_us - now_us) / 1000LL;
        TickType_t wait   = pdMS_TO_TICKS((rem_ms > 200) ? 200 : (rem_ms > 0 ? rem_ms : 1));

        e220_lora_event_t event = {0};
        esp_err_t         err   = e220_lora_receive_event(&event, wait);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: erro ao aguardar evento: %s", esp_err_to_name(err));
            continue;
        }
        if (event.type != E220_LORA_EVENT_RX_DONE) {
            if (event.type == E220_LORA_EVENT_RX_ERROR) {
                ESP_LOGW(TAG, "ROOT: frame descartado por erro de RX");
            }
            continue;
        }
        if (event.payload_len != sizeof(lora_probe_frame_t)) {
            ESP_LOGW(TAG, "ROOT: payload inesperado len=%u", (unsigned)event.payload_len);
            continue;
        }

        lora_probe_frame_t probe = {0};
        memcpy(&probe, event.payload, sizeof(probe));
        if (probe.type != LORA_TEST_FRAME_TYPE_PROBE) {
            continue;
        }

        lora_node_stats_t *node      = find_or_allocate_node(probe.mac);
        bool               duplicate = false;
        if (node != NULL) {
            node->rx_count++;
            duplicate = node->has_last_seq && (probe.seq <= node->last_seq);
            if (duplicate) {
                node->duplicate_count++;
            } else {
                if (!node->has_first_seq) {
                    node->has_first_seq = true;
                    node->first_seq     = probe.seq;
                }
                node->has_last_seq = true;
                node->last_seq     = probe.seq;
                node->unique_rx_count++;
            }
            node->last_rssi_dbm  = event.rssi_dbm;
            node->last_snr_db    = event.snr_db;
            s_last_active_node   = node;
        }

        char mac_buffer[18] = {0};
        mac_to_str(probe.mac, mac_buffer, sizeof(mac_buffer));
        ESP_LOGI(TAG, "ROOT RX mac=%s seq=%lu dup=%s rssi=%d snr=%d",
                 mac_buffer, (unsigned long)probe.seq,
                 duplicate ? "sim" : "nao",
                 (int)event.rssi_dbm, (int)event.snr_db);

        lora_ack_frame_t ack = {
            .type           = LORA_TEST_FRAME_TYPE_ACK,
            .seq            = probe.seq,
            .probe_rssi_dbm = event.rssi_dbm,
            .probe_snr_db   = event.snr_db,
            .reserved       = 0,
            .uptime_ms      = uptime_ms_now(),
        };
        memcpy(ack.mac, probe.mac, sizeof(ack.mac));

        e220_lora_flush_events();
        err = e220_lora_transmit((const uint8_t *)&ack, sizeof(ack));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: falha ao iniciar ACK para %s: %s",
                     mac_buffer, esp_err_to_name(err));
            if (node != NULL) {
                node->ack_fail_count++;
            }
            continue;
        }

        err = wait_for_tx_done(LORA_TEST_TX_DONE_TIMEOUT_MS);
        if (err == ESP_OK) {
            if (node != NULL) {
                node->ack_ok_count++;
            }
            ESP_LOGI(TAG, "ROOT ACK mac=%s seq=%lu probe_rssi=%d probe_snr=%d",
                     mac_buffer, (unsigned long)probe.seq,
                     (int)ack.probe_rssi_dbm, (int)ack.probe_snr_db);
            vTaskDelay(pdMS_TO_TICKS(500));
            e220_lora_flush_events();
        } else {
            ESP_LOGW(TAG, "ROOT: timeout aguardando TX_DONE do ACK para %s", mac_buffer);
            if (node != NULL) {
                node->ack_fail_count++;
            }
        }
    }
}

static void __attribute__((unused)) run_node(const uint8_t self_mac[6])
{
    uint32_t sequence        = 0;
    uint32_t startup_delay   = next_delay_ms(CONFIG_LORA_TEST_NODE_SEND_INTERVAL_MS,
                                             CONFIG_LORA_TEST_NODE_SEND_JITTER_MS);
    ESP_LOGI(TAG, "NODE: atraso inicial aleatorio de %lu ms", (unsigned long)startup_delay);
    vTaskDelay(pdMS_TO_TICKS(startup_delay));

    while (1) {
        lora_probe_frame_t probe = {
            .type      = LORA_TEST_FRAME_TYPE_PROBE,
            .seq       = ++sequence,
            .uptime_ms = uptime_ms_now(),
        };
        memcpy(probe.mac, self_mac, sizeof(probe.mac));

        s_node_view.tx_count++;
        s_node_view.last_seq          = probe.seq;
        s_node_view.last_tx_rssi_dbm  = INT16_MIN;
        s_node_view.last_rx_rssi_dbm  = INT16_MIN;
        s_node_view.last_snr_db       = INT8_MIN;
        s_node_view.last_ack_status   = LORA_TEST_NODE_ACK_STATUS_TXWAIT;

        e220_lora_flush_events();
        esp_err_t err = e220_lora_transmit((const uint8_t *)&probe, sizeof(probe));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NODE: falha ao iniciar PROBE seq=%lu: %s",
                     (unsigned long)probe.seq, esp_err_to_name(err));
            s_node_view.ack_fail_count++;
            s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_TXFAIL;
        } else {
            err = wait_for_tx_done(LORA_TEST_TX_DONE_TIMEOUT_MS);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "NODE: timeout aguardando TX_DONE seq=%lu",
                         (unsigned long)probe.seq);
                s_node_view.ack_fail_count++;
                s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_TDOUT;
            } else {
                bool    ack_received  = false;
                bool    ack_wait_error = false;
                int64_t ack_deadline  = esp_timer_get_time() +
                                        ((int64_t)LORA_TEST_ACK_TIMEOUT_MS * 1000LL);

                s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_WAITACK;

                while (esp_timer_get_time() < ack_deadline) {
                    e220_lora_event_t event  = {0};
                    int64_t           rem_ms = (ack_deadline - esp_timer_get_time()) / 1000LL;
                    TickType_t        wait   = pdMS_TO_TICKS((rem_ms > 100) ? 100 :
                                                             (rem_ms > 0 ? rem_ms : 1));
                    err = e220_lora_receive_event(&event, wait);
                    if (err == ESP_ERR_TIMEOUT) {
                        continue;
                    }
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "NODE: erro ao aguardar ACK seq=%lu: %s",
                                 (unsigned long)probe.seq, esp_err_to_name(err));
                        ack_wait_error = true;
                        break;
                    }
                    if (event.type != E220_LORA_EVENT_RX_DONE) {
                        continue;
                    }
                    if (event.payload_len != sizeof(lora_ack_frame_t)) {
                        continue;
                    }

                    lora_ack_frame_t ack = {0};
                    memcpy(&ack, event.payload, sizeof(ack));
                    if (ack.type != LORA_TEST_FRAME_TYPE_ACK ||
                        !mac_equal(ack.mac, self_mac) ||
                        ack.seq != probe.seq) {
                        continue;
                    }

                    char rssi_tx[12] = {0}, rssi_rx[12] = {0}, snr[12] = {0};
                    format_dbm_value(ack.probe_rssi_dbm, true, rssi_tx, sizeof(rssi_tx));
                    format_dbm_value(event.rssi_dbm,     true, rssi_rx, sizeof(rssi_rx));
                    format_db_value((int)event.snr_db,   true, snr,     sizeof(snr));
                    ESP_LOGI(TAG,
                             "NODE ACK seq=%lu rssi_no_root=%s rssi_ack=%s snr=%s",
                             (unsigned long)probe.seq, rssi_tx, rssi_rx, snr);

                    s_node_view.ack_ok_count++;
                    s_node_view.last_tx_rssi_dbm = ack.probe_rssi_dbm;
                    s_node_view.last_rx_rssi_dbm = event.rssi_dbm;
                    s_node_view.last_snr_db       = event.snr_db;
                    s_node_view.last_ack_status   = LORA_TEST_NODE_ACK_STATUS_OK;
                    ack_received = true;
                    break;
                }

                if (!ack_received) {
                    ESP_LOGW(TAG, "NODE: ACK timeout seq=%lu", (unsigned long)probe.seq);
                    s_node_view.ack_fail_count++;
                    s_node_view.last_ack_status = ack_wait_error ?
                        LORA_TEST_NODE_ACK_STATUS_ERROR :
                        LORA_TEST_NODE_ACK_STATUS_TIMEOUT;
                }
            }
        }

        uint32_t delay_ms = next_delay_ms(CONFIG_LORA_TEST_NODE_SEND_INTERVAL_MS,
                                          CONFIG_LORA_TEST_NODE_SEND_JITTER_MS);
        ESP_LOGI(TAG, "NODE: proximo envio em %lu ms", (unsigned long)delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void lora_test_run(void)
{
    uint8_t self_mac[6] = {0};

    reset_runtime_state();
    ESP_ERROR_CHECK(esp_read_mac(self_mac, ESP_MAC_WIFI_STA));
    log_config_banner(self_mac);
    ESP_ERROR_CHECK(init_lora_radio());

#if CONFIG_LORA_TEST_ROLE_ROOT
    run_root(self_mac);
#else
    run_node(self_mac);
#endif
}
