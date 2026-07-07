#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

#include "sdkconfig.h"
#include "espnow_test_module.h"

#if CONFIG_ESPNOW_TEST_MODE

#define ESPNOW_TEST_TYPE_PING    1U
#define ESPNOW_TEST_TYPE_ACK     2U
#define ESPNOW_TEST_ACK_TIMEOUT_MS 300

static const char *TAG = "ESPNOW_TEST";

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint32_t seq;
    uint32_t sender_id;
    uint32_t uptime_ms;
} espnow_test_frame_t;

static volatile bool                 g_tx_done          = false;
static volatile esp_now_send_status_t g_tx_status       = ESP_NOW_SEND_FAIL;
static volatile bool                 g_ack_received      = false;
static volatile uint32_t             g_ack_seq           = 0;
static volatile int16_t              g_last_ack_rssi_dbm = INT16_MIN;
static uint32_t                      g_sequence          = 0;

static esp_err_t ensure_peer_registered(const uint8_t peer_mac[6])
{
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_now_peer_info_t peer = {
        .channel = CONFIG_ESPNOW_TEST_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, peer_mac, sizeof(peer.peer_addr));

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;
    }

    return err;
}

static bool parse_mac_str(const char *mac_str, uint8_t out_mac[6])
{
    if (mac_str == NULL || out_mac == NULL) {
        return false;
    }

    int values[6] = {0};
    int parsed = sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);
    if (parsed != 6) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        if (values[i] < 0 || values[i] > 0xFF) {
            return false;
        }
        out_mac[i] = (uint8_t)values[i];
    }

    return true;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    g_tx_status = status;
    g_tx_done   = true;
}
#else
static void on_espnow_send(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    g_tx_status = status;
    g_tx_done   = true;
}
#endif

static void on_espnow_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len != (int)sizeof(espnow_test_frame_t)) {
        return;
    }

    espnow_test_frame_t frame;
    memcpy(&frame, data, sizeof(frame));

    int8_t rssi = recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0;

#if CONFIG_ESPNOW_TEST_ROLE_RX
    if (frame.type == ESPNOW_TEST_TYPE_PING) {
        ESP_LOGI(TAG, "RX PING seq=%lu sender=0x%08lX rssi=%d",
                 (unsigned long)frame.seq, (unsigned long)frame.sender_id, (int)rssi);

        esp_err_t add_peer_err = ensure_peer_registered(recv_info->src_addr);
        if (add_peer_err != ESP_OK) {
            ESP_LOGW(TAG,
                     "Falha ao cadastrar peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                     recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                     recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
                     esp_err_to_name(add_peer_err));
            return;
        }

        espnow_test_frame_t ack = {
            .type      = ESPNOW_TEST_TYPE_ACK,
            .seq       = frame.seq,
            .sender_id = frame.sender_id,
            .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };

        esp_err_t err = esp_now_send(recv_info->src_addr, (const uint8_t *)&ack, sizeof(ack));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao enviar ACK: %s", esp_err_to_name(err));
        }
    }
#endif

#if CONFIG_ESPNOW_TEST_ROLE_TX
    if (frame.type == ESPNOW_TEST_TYPE_ACK) {
        g_ack_seq           = frame.seq;
        g_ack_received      = true;
        g_last_ack_rssi_dbm = rssi;
        ESP_LOGI(TAG, "RX ACK seq=%lu rssi=%d", (unsigned long)frame.seq, (int)rssi);
    }
#endif
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
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_channel(CONFIG_ESPNOW_TEST_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_now_register_send_cb(on_espnow_send);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_now_register_recv_cb(on_espnow_recv);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

void espnow_test_run(void)
{
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "MODO TESTE ESPNOW ATIVO");
    ESP_LOGW(TAG, "Canal: %d", CONFIG_ESPNOW_TEST_CHANNEL);
#if CONFIG_ESPNOW_TEST_ROLE_TX
    ESP_LOGW(TAG, "Role: TX (envia ping e espera ACK)");
#else
    ESP_LOGW(TAG, "Role: RX (recebe ping e responde ACK)");
#endif
    ESP_LOGW(TAG, "========================================");

    ESP_ERROR_CHECK(init_wifi_espnow());

    uint8_t self_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(self_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "MAC local STA: %02X:%02X:%02X:%02X:%02X:%02X",
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);

#if CONFIG_ESPNOW_TEST_ROLE_TX
    uint8_t peer_mac[6] = {0};
    if (!parse_mac_str(CONFIG_ESPNOW_TEST_PEER_MAC, peer_mac)) {
        ESP_LOGE(TAG, "MAC invalido em CONFIG_ESPNOW_TEST_PEER_MAC: %s",
                 CONFIG_ESPNOW_TEST_PEER_MAC);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    esp_err_t add_peer_err = ensure_peer_registered(peer_mac);
    if (add_peer_err != ESP_OK && add_peer_err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Falha ao adicionar peer: %s", esp_err_to_name(add_peer_err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uint32_t sender_id = ((uint32_t)self_mac[2] << 24) |
                         ((uint32_t)self_mac[3] << 16) |
                         ((uint32_t)self_mac[4] << 8)  |
                         (uint32_t)self_mac[5];

    while (1) {
        espnow_test_frame_t frame = {
            .type      = ESPNOW_TEST_TYPE_PING,
            .seq       = ++g_sequence,
            .sender_id = sender_id,
            .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };

        g_tx_done           = false;
        g_tx_status         = ESP_NOW_SEND_FAIL;
        g_ack_received      = false;
        g_last_ack_rssi_dbm = INT16_MIN;

        esp_err_t err = esp_now_send(peer_mac, (const uint8_t *)&frame, sizeof(frame));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_send falhou: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
            continue;
        }

        int wait_steps = ESPNOW_TEST_ACK_TIMEOUT_MS / 10;
        for (int i = 0; i < wait_steps && !g_tx_done; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!g_tx_done || g_tx_status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGW(TAG, "TX sem confirmacao MAC (seq=%lu)", (unsigned long)frame.seq);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
            continue;
        }

        for (int i = 0; i < wait_steps && !g_ack_received; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (g_ack_received && g_ack_seq == frame.seq) {
            ESP_LOGI(TAG, "PING OK seq=%lu ack_rssi=%d",
                     (unsigned long)frame.seq, (int)g_last_ack_rssi_dbm);
        } else {
            ESP_LOGW(TAG, "ACK timeout seq=%lu", (unsigned long)frame.seq);
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
    }
#else
    ESP_LOGI(TAG, "RX aguardando pings...");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

#else

void espnow_test_run(void)
{
}

#endif
