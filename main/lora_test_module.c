#include "lora_test_module.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sx1276_lora.h"

#define LORA_TEST_ACK_TIMEOUT_MS 1200
#define LORA_TEST_TX_DONE_TIMEOUT_MS 1000
#define LORA_TEST_MAX_NODES 32

typedef enum {
    LORA_TEST_FRAME_TYPE_PROBE = 0xA1,
    LORA_TEST_FRAME_TYPE_ACK = 0xA2,
} lora_test_frame_type_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t mac[6];
    uint32_t seq;
    uint32_t uptime_ms;
} lora_probe_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t mac[6];
    uint32_t seq;
    int16_t probe_rssi_dbm;
    int8_t probe_snr_db;
    int8_t reserved;
    uint32_t uptime_ms;
} lora_ack_frame_t;

typedef struct {
    bool in_use;
    bool has_last_seq;
    uint8_t mac[6];
    uint32_t last_seq;
    uint32_t rx_count;
    uint32_t duplicate_count;
    uint32_t ack_ok_count;
    uint32_t ack_fail_count;
    int16_t last_rssi_dbm;
    int8_t last_snr_db;
} lora_node_stats_t;

static const char *TAG = "LORA_TEST";
static lora_node_stats_t s_nodes[LORA_TEST_MAX_NODES] = {0};

static void mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool mac_equal(const uint8_t left[6], const uint8_t right[6])
{
    return memcmp(left, right, 6) == 0;
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

    uint32_t span = (jitter_ms * 2U) + 1U;
    int32_t offset = (int32_t)(esp_random() % span) - (int32_t)jitter_ms;
    int32_t candidate = (int32_t)base_ms + offset;
    if (candidate < 100) {
        candidate = 100;
    }
    return (uint32_t)candidate;
}

static esp_err_t wait_for_tx_done(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);

    while (esp_timer_get_time() < deadline_us) {
        sx1276_lora_event_t event = {0};
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000LL;
        TickType_t wait = pdMS_TO_TICKS((remaining_ms > 50) ? 50 : (remaining_ms > 0 ? remaining_ms : 1));
        esp_err_t err = sx1276_lora_receive_event(&event, wait);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }
        if (event.type == SX1276_LORA_EVENT_TX_DONE) {
            return ESP_OK;
        }
    }

    (void)sx1276_lora_start_rx_continuous();
    return ESP_ERR_TIMEOUT;
}

static void log_config_banner(const uint8_t self_mac[6])
{
    char mac_buffer[18] = {0};
    mac_to_str(self_mac, mac_buffer, sizeof(mac_buffer));

    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "MODO TESTE LORA ATIVO");
    ESP_LOGW(TAG, "Role: %s", CONFIG_LORA_TEST_ROLE_ROOT ? "ROOT" : "NODE");
    ESP_LOGW(TAG, "MAC local: %s", mac_buffer);
    ESP_LOGW(TAG, "Freq=%u MHz BW=%u kHz SF=%u TX=%d dBm",
             (unsigned)CONFIG_LORA_TEST_FREQUENCY_MHZ,
             (unsigned)CONFIG_LORA_TEST_BANDWIDTH_KHZ,
             (unsigned)CONFIG_LORA_TEST_SPREADING_FACTOR,
             (int)CONFIG_LORA_TEST_TX_POWER_DBM);
    ESP_LOGW(TAG, "Pinos SX1276 CLK=%d MISO=%d MOSI=%d CS=%d RST=%d DIO0=%d",
             LORA_TEST_PIN_SCK, LORA_TEST_PIN_MISO, LORA_TEST_PIN_MOSI,
             LORA_TEST_PIN_CS, LORA_TEST_PIN_RST, LORA_TEST_PIN_DIO0);
    ESP_LOGW(TAG, "========================================");
}

static void log_root_summary(void)
{
    ESP_LOGI(TAG, "Resumo ROOT:");
    for (size_t i = 0; i < LORA_TEST_MAX_NODES; ++i) {
        if (!s_nodes[i].in_use) {
            continue;
        }

        char mac_buffer[18] = {0};
        mac_to_str(s_nodes[i].mac, mac_buffer, sizeof(mac_buffer));
        ESP_LOGI(TAG,
                 "  mac=%s pkts=%lu dup=%lu ack_ok=%lu ack_fail=%lu last_seq=%lu rssi=%d snr=%d",
                 mac_buffer,
                 (unsigned long)s_nodes[i].rx_count,
                 (unsigned long)s_nodes[i].duplicate_count,
                 (unsigned long)s_nodes[i].ack_ok_count,
                 (unsigned long)s_nodes[i].ack_fail_count,
                 (unsigned long)s_nodes[i].last_seq,
                 (int)s_nodes[i].last_rssi_dbm,
                 (int)s_nodes[i].last_snr_db);
    }
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
    };

    esp_err_t err = sx1276_lora_init(&config);
    if (err != ESP_OK) {
        return err;
    }

    sx1276_lora_debug_dump();
    return sx1276_lora_start_rx_continuous();
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

        int64_t remaining_summary_ms = (next_summary_us - now_us) / 1000LL;
        TickType_t wait = pdMS_TO_TICKS((remaining_summary_ms > 200) ? 200 :
                                        (remaining_summary_ms > 0 ? remaining_summary_ms : 1));

        sx1276_lora_event_t event = {0};
        esp_err_t err = sx1276_lora_receive_event(&event, wait);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: erro ao aguardar evento: %s", esp_err_to_name(err));
            continue;
        }
        if (event.type != SX1276_LORA_EVENT_RX_DONE) {
            if (event.type == SX1276_LORA_EVENT_RX_ERROR) {
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

        lora_node_stats_t *node = find_or_allocate_node(probe.mac);
        bool duplicate = false;
        if (node != NULL) {
            node->rx_count++;
            duplicate = node->has_last_seq && (probe.seq <= node->last_seq);
            if (duplicate) {
                node->duplicate_count++;
            } else {
                node->has_last_seq = true;
                node->last_seq = probe.seq;
            }
            node->last_rssi_dbm = event.rssi_dbm;
            node->last_snr_db = event.snr_db;
        }

        char mac_buffer[18] = {0};
        mac_to_str(probe.mac, mac_buffer, sizeof(mac_buffer));
        ESP_LOGI(TAG,
                 "ROOT RX mac=%s seq=%lu dup=%s rssi=%d snr=%d",
                 mac_buffer,
                 (unsigned long)probe.seq,
                 duplicate ? "sim" : "nao",
                 (int)event.rssi_dbm,
                 (int)event.snr_db);

        lora_ack_frame_t ack = {
            .type = LORA_TEST_FRAME_TYPE_ACK,
            .seq = probe.seq,
            .probe_rssi_dbm = event.rssi_dbm,
            .probe_snr_db = event.snr_db,
            .reserved = 0,
            .uptime_ms = uptime_ms_now(),
        };
        memcpy(ack.mac, probe.mac, sizeof(ack.mac));

        sx1276_lora_flush_events();
        err = sx1276_lora_transmit((const uint8_t *)&ack, sizeof(ack));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: falha ao iniciar ACK para %s: %s", mac_buffer, esp_err_to_name(err));
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
            ESP_LOGI(TAG,
                     "ROOT ACK mac=%s seq=%lu probe_rssi=%d probe_snr=%d",
                     mac_buffer,
                     (unsigned long)probe.seq,
                     (int)ack.probe_rssi_dbm,
                     (int)ack.probe_snr_db);
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
    uint32_t sequence = 0;
    uint32_t startup_delay_ms = next_delay_ms(CONFIG_LORA_TEST_NODE_SEND_INTERVAL_MS,
                                              CONFIG_LORA_TEST_NODE_SEND_JITTER_MS);
    ESP_LOGI(TAG, "NODE: atraso inicial aleatorio de %lu ms", (unsigned long)startup_delay_ms);
    vTaskDelay(pdMS_TO_TICKS(startup_delay_ms));

    while (1) {
        lora_probe_frame_t probe = {
            .type = LORA_TEST_FRAME_TYPE_PROBE,
            .seq = ++sequence,
            .uptime_ms = uptime_ms_now(),
        };
        memcpy(probe.mac, self_mac, sizeof(probe.mac));

        sx1276_lora_flush_events();
        esp_err_t err = sx1276_lora_transmit((const uint8_t *)&probe, sizeof(probe));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NODE: falha ao iniciar PROBE seq=%lu: %s",
                     (unsigned long)probe.seq,
                     esp_err_to_name(err));
        } else {
            err = wait_for_tx_done(LORA_TEST_TX_DONE_TIMEOUT_MS);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "NODE: timeout aguardando TX_DONE seq=%lu", (unsigned long)probe.seq);
            } else {
                bool ack_received = false;
                int64_t ack_deadline_us = esp_timer_get_time() +
                                          ((int64_t)LORA_TEST_ACK_TIMEOUT_MS * 1000LL);

                while (esp_timer_get_time() < ack_deadline_us) {
                    sx1276_lora_event_t event = {0};
                    int64_t remaining_ms = (ack_deadline_us - esp_timer_get_time()) / 1000LL;
                    TickType_t wait = pdMS_TO_TICKS((remaining_ms > 100) ? 100 :
                                                    (remaining_ms > 0 ? remaining_ms : 1));
                    err = sx1276_lora_receive_event(&event, wait);
                    if (err == ESP_ERR_TIMEOUT) {
                        continue;
                    }
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "NODE: erro ao aguardar ACK seq=%lu: %s",
                                 (unsigned long)probe.seq,
                                 esp_err_to_name(err));
                        break;
                    }
                    if (event.type != SX1276_LORA_EVENT_RX_DONE) {
                        continue;
                    }
                    if (event.payload_len != sizeof(lora_ack_frame_t)) {
                        continue;
                    }

                    lora_ack_frame_t ack = {0};
                    memcpy(&ack, event.payload, sizeof(ack));
                    if (ack.type != LORA_TEST_FRAME_TYPE_ACK || !mac_equal(ack.mac, self_mac) || ack.seq != probe.seq) {
                        continue;
                    }

                    ESP_LOGI(TAG,
                             "NODE ACK seq=%lu root_observed_rssi=%d root_observed_snr=%d ack_rssi=%d ack_snr=%d",
                             (unsigned long)probe.seq,
                             (int)ack.probe_rssi_dbm,
                             (int)ack.probe_snr_db,
                             (int)event.rssi_dbm,
                             (int)event.snr_db);
                    ack_received = true;
                    break;
                }

                if (!ack_received) {
                    ESP_LOGW(TAG, "NODE: ACK timeout seq=%lu", (unsigned long)probe.seq);
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
    ESP_ERROR_CHECK(esp_read_mac(self_mac, ESP_MAC_WIFI_STA));
    log_config_banner(self_mac);

    ESP_ERROR_CHECK(init_lora_radio());

#if CONFIG_LORA_TEST_ROLE_ROOT
    run_root(self_mac);
#else
    run_node(self_mac);
#endif
}