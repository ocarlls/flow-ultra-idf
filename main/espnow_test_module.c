#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "sdkconfig.h"
#include "espnow_test_module.h"
#include "lora_test_config.h"

#if CONFIG_ESPNOW_TEST_MODE

#define ESPNOW_TEST_TYPE_PING 1U
#define ESPNOW_TEST_TYPE_ACK  2U
#define ESPNOW_TEST_ACK_TIMEOUT_MS 300
#define ESPNOW_TEST_OLED_CMD_BITS 8
#define ESPNOW_TEST_OLED_PARAM_BITS 8
#define ESPNOW_TEST_OLED_FLUSH_TIMEOUT_MS 250
#define ESPNOW_TEST_OLED_TEXT_COLS (LORA_TEST_OLED_WIDTH / 6)
#define ESPNOW_TEST_OLED_TEXT_ROWS (LORA_TEST_OLED_HEIGHT / 8)
#define ESPNOW_TEST_OLED_FB_SIZE ((LORA_TEST_OLED_WIDTH * LORA_TEST_OLED_HEIGHT) / 8)

static const char *TAG = "ESPNOW_TEST";

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint32_t seq;
    uint32_t sender_id;
    uint32_t uptime_ms;
} espnow_test_frame_t;

static volatile bool g_tx_done = false;
static volatile esp_now_send_status_t g_tx_status = ESP_NOW_SEND_FAIL;
static volatile bool g_ack_received = false;
static volatile uint32_t g_ack_seq = 0;
static volatile int16_t g_last_ack_rssi_dbm = INT16_MIN;
static uint32_t g_sequence = 0;

typedef struct {
    char ch;
    uint8_t columns[5];
} espnow_oled_glyph_t;

typedef struct {
    bool available;
    i2c_master_bus_handle_t i2c_bus;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    SemaphoreHandle_t flush_done_sem;
    uint8_t framebuffer[ESPNOW_TEST_OLED_FB_SIZE];
} espnow_oled_ctx_t;

typedef struct {
    uint8_t peer_mac[6];
    bool peer_known;
    uint32_t last_seq;
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t ack_ok_count;
    uint32_t ack_fail_count;
    int16_t last_rssi_dbm;
    char last_status[10];
} espnow_display_state_t;

static espnow_oled_ctx_t s_oled = {0};
static espnow_display_state_t s_display = {
    .last_rssi_dbm = INT16_MIN,
    .last_status = "IDLE",
};

static const espnow_oled_glyph_t s_oled_font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'%', {0x63, 0x13, 0x08, 0x64, 0x63}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

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

static void format_dbm_value(int16_t value, bool valid, char *buffer, size_t buffer_len)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--");
        return;
    }
    snprintf(buffer, buffer_len, "%dDBM", (int)value);
}

static void format_percent_value(uint32_t numerator, uint32_t denominator, char *buffer, size_t buffer_len)
{
    uint32_t percent_tenths = compute_percent_tenths(numerator, denominator);
    snprintf(buffer, buffer_len, "%lu.%lu%%",
             (unsigned long)(percent_tenths / 10U),
             (unsigned long)(percent_tenths % 10U));
}

static void mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len)
{
    snprintf(buffer, buffer_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static const uint8_t *find_glyph(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - ('a' - 'A'));
    }

    for (size_t i = 0; i < sizeof(s_oled_font) / sizeof(s_oled_font[0]); ++i) {
        if (s_oled_font[i].ch == ch) {
            return s_oled_font[i].columns;
        }
    }

    return s_oled_font[0].columns;
}

static bool oled_flush_done_cb(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_io_event_data_t *edata,
                               void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    espnow_oled_ctx_t *oled = (espnow_oled_ctx_t *)user_ctx;
    BaseType_t task_woken = pdFALSE;
    if (oled != NULL && oled->flush_done_sem != NULL) {
        xSemaphoreGiveFromISR(oled->flush_done_sem, &task_woken);
    }

    return task_woken == pdTRUE;
}

static void oled_cleanup(void)
{
    if (s_oled.panel_handle != NULL) {
        (void)esp_lcd_panel_del(s_oled.panel_handle);
        s_oled.panel_handle = NULL;
    }
    if (s_oled.io_handle != NULL) {
        (void)esp_lcd_panel_io_del(s_oled.io_handle);
        s_oled.io_handle = NULL;
    }
    if (s_oled.i2c_bus != NULL) {
        (void)i2c_del_master_bus(s_oled.i2c_bus);
        s_oled.i2c_bus = NULL;
    }
    if (s_oled.flush_done_sem != NULL) {
        vSemaphoreDelete(s_oled.flush_done_sem);
        s_oled.flush_done_sem = NULL;
    }

    s_oled.available = false;
    memset(s_oled.framebuffer, 0, sizeof(s_oled.framebuffer));
}

static esp_err_t oled_init(void)
{
    if (s_oled.available) {
        return ESP_OK;
    }

    // Heltec LoRa32 v2: Vext em LOW para energizar o OLED
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_level(21, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    s_oled.flush_done_sem = xSemaphoreCreateBinary();
    if (s_oled.flush_done_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = LORA_TEST_OLED_I2C_PORT,
        .sda_io_num = LORA_TEST_OLED_PIN_SDA,
        .scl_io_num = LORA_TEST_OLED_PIN_SCL,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_oled.i2c_bus);
    if (err != ESP_OK) {
        oled_cleanup();
        return err;
    }

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = LORA_TEST_OLED_I2C_ADDR,
        .scl_speed_hz = LORA_TEST_OLED_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = ESPNOW_TEST_OLED_CMD_BITS,
        .lcd_param_bits = ESPNOW_TEST_OLED_PARAM_BITS,
    };

    err = esp_lcd_new_panel_io_i2c(s_oled.i2c_bus, &io_config, &s_oled.io_handle);
    if (err != ESP_OK) {
        oled_cleanup();
        return err;
    }

    const esp_lcd_panel_io_callbacks_t callbacks = {
        .on_color_trans_done = oled_flush_done_cb,
    };
    err = esp_lcd_panel_io_register_event_callbacks(s_oled.io_handle, &callbacks, &s_oled);
    if (err != ESP_OK) {
        oled_cleanup();
        return err;
    }

    esp_lcd_panel_ssd1306_config_t vendor_config = {
        .height = LORA_TEST_OLED_HEIGHT,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = LORA_TEST_OLED_PIN_RST,
        .vendor_config = &vendor_config,
    };

    err = esp_lcd_new_panel_ssd1306(s_oled.io_handle, &panel_config, &s_oled.panel_handle);
    if (err != ESP_OK) {
        oled_cleanup();
        return err;
    }

    err = esp_lcd_panel_reset(s_oled.panel_handle);
    if (err == ESP_OK) {
        err = esp_lcd_panel_init(s_oled.panel_handle);
    }
    if (err == ESP_OK) {
        err = esp_lcd_panel_disp_on_off(s_oled.panel_handle, true);
    }
    if (err != ESP_OK) {
        oled_cleanup();
        return err;
    }

    memset(s_oled.framebuffer, 0, sizeof(s_oled.framebuffer));
    s_oled.available = true;
    return ESP_OK;
}

static void oled_disable_after_error(esp_err_t err, const char *stage)
{
    ESP_LOGW(TAG, "OLED indisponivel em %s: %s", stage, esp_err_to_name(err));
    oled_cleanup();
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (!s_oled.available || x < 0 || x >= LORA_TEST_OLED_WIDTH || y < 0 || y >= LORA_TEST_OLED_HEIGHT) {
        return;
    }

    size_t index = ((size_t)(y / 8) * (size_t)LORA_TEST_OLED_WIDTH) + (size_t)x;
    uint8_t mask = (uint8_t)(1U << (y % 8));

    if (on) {
        s_oled.framebuffer[index] |= mask;
    } else {
        s_oled.framebuffer[index] &= (uint8_t)~mask;
    }
}

static void oled_clear(void)
{
    if (!s_oled.available) {
        return;
    }
    memset(s_oled.framebuffer, 0, sizeof(s_oled.framebuffer));
}

static void oled_draw_char(int col, int row, char ch)
{
    if (!s_oled.available || col < 0 || col >= ESPNOW_TEST_OLED_TEXT_COLS || row < 0 || row >= ESPNOW_TEST_OLED_TEXT_ROWS) {
        return;
    }

    int base_x = col * 6;
    int base_y = row * 8;
    const uint8_t *glyph = find_glyph(ch);

    for (int dx = 0; dx < 5; ++dx) {
        uint8_t column_bits = glyph[dx];
        for (int dy = 0; dy < 7; ++dy) {
            if ((column_bits & (1U << dy)) != 0U) {
                oled_set_pixel(base_x + dx, base_y + dy, true);
            }
        }
    }
}

static void oled_draw_text(int row, const char *text)
{
    if (!s_oled.available || row < 0 || row >= ESPNOW_TEST_OLED_TEXT_ROWS || text == NULL) {
        return;
    }

    for (int col = 0; col < ESPNOW_TEST_OLED_TEXT_COLS && text[col] != '\0'; ++col) {
        oled_draw_char(col, row, text[col]);
    }
}

static esp_err_t oled_flush(void)
{
    if (!s_oled.available) {
        return ESP_OK;
    }

    while (xSemaphoreTake(s_oled.flush_done_sem, 0) == pdTRUE) {
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(s_oled.panel_handle,
                                              0,
                                              0,
                                              LORA_TEST_OLED_WIDTH,
                                              LORA_TEST_OLED_HEIGHT,
                                              s_oled.framebuffer);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(s_oled.flush_done_sem, pdMS_TO_TICKS(ESPNOW_TEST_OLED_FLUSH_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void oled_render_status(void)
{
    if (!s_oled.available) {
        return;
    }

    char line[ESPNOW_TEST_OLED_TEXT_ROWS][ESPNOW_TEST_OLED_TEXT_COLS + 1] = {{0}};
    char rssi_text[16] = {0};
    char pdr_text[16] = {0};
    char mac_tail_text[10] = "--:--:--";
    char mac_text[18] = "--:--:--:--:--:--";

    format_dbm_value(s_display.last_rssi_dbm,
                     s_display.last_rssi_dbm != INT16_MIN,
                     rssi_text,
                     sizeof(rssi_text));
    format_percent_value(s_display.ack_ok_count,
                         s_display.tx_count,
                         pdr_text,
                         sizeof(pdr_text));
    if (s_display.peer_known) {
        mac_to_str(s_display.peer_mac, mac_text, sizeof(mac_text));
        memcpy(mac_tail_text, &mac_text[9], 8);
        mac_tail_text[8] = '\0';
    }

#if CONFIG_ESPNOW_TEST_ROLE_TX
    snprintf(line[0], sizeof(line[0]), "ESPNOW TX");
    snprintf(line[1], sizeof(line[1]), "STS:%s", s_display.last_status);
    snprintf(line[2], sizeof(line[2]), "AR:%s", rssi_text);
    snprintf(line[3], sizeof(line[3]), "TX:%lu", (unsigned long)s_display.tx_count);
    snprintf(line[4], sizeof(line[4]), "ACKOK:%lu", (unsigned long)s_display.ack_ok_count);
    snprintf(line[5], sizeof(line[5]), "ACKFL:%lu", (unsigned long)s_display.ack_fail_count);
    snprintf(line[6], sizeof(line[6]), "PDR:%s", pdr_text);
    snprintf(line[7], sizeof(line[7]), "SEQ:%lu", (unsigned long)s_display.last_seq);
#else
    snprintf(line[0], sizeof(line[0]), "ESPNOW RX");
    snprintf(line[1], sizeof(line[1]), "PEER:*:*:*:%s", mac_tail_text);
    snprintf(line[2], sizeof(line[2]), "RSSI:%s", rssi_text);
    snprintf(line[3], sizeof(line[3]), "PKT:%lu", (unsigned long)s_display.rx_count);
    snprintf(line[4], sizeof(line[4]), "ACKOK:%lu", (unsigned long)s_display.ack_ok_count);
    snprintf(line[5], sizeof(line[5]), "ACKFL:%lu", (unsigned long)s_display.ack_fail_count);
    snprintf(line[6], sizeof(line[6]), "SEQ:%lu", (unsigned long)s_display.last_seq);
    snprintf(line[7], sizeof(line[7]), "STS:%s", s_display.last_status);
#endif

    oled_clear();
    for (int i = 0; i < ESPNOW_TEST_OLED_TEXT_ROWS; ++i) {
        oled_draw_text(i, line[i]);
    }

    esp_err_t err = oled_flush();
    if (err != ESP_OK) {
        oled_disable_after_error(err, "espnow_flush");
    }
}

static esp_err_t ensure_peer_registered(const uint8_t peer_mac[6])
{
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_now_peer_info_t peer = {
        .channel = CONFIG_ESPNOW_TEST_CHANNEL,
        .ifidx = WIFI_IF_STA,
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

static void on_espnow_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    g_tx_status = status;
    g_tx_done = true;
}

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
        s_display.rx_count++;
        s_display.last_seq = frame.seq;
        s_display.last_rssi_dbm = rssi;
        memcpy((void *)s_display.peer_mac, recv_info->src_addr, sizeof(s_display.peer_mac));
        s_display.peer_known = true;
        snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "RXPING");

        ESP_LOGI(TAG,
                 "RX PING seq=%lu sender=0x%08lX rssi=%d",
                 (unsigned long)frame.seq,
                 (unsigned long)frame.sender_id,
                 (int)rssi);

        esp_err_t add_peer_err = ensure_peer_registered(recv_info->src_addr);
        if (add_peer_err != ESP_OK) {
            s_display.ack_fail_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "PEERERR");
            ESP_LOGW(TAG,
                     "Falha ao cadastrar peer %02X:%02X:%02X:%02X:%02X:%02X: %s",
                     recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                     recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
                     esp_err_to_name(add_peer_err));
            return;
        }

        espnow_test_frame_t ack = {
            .type = ESPNOW_TEST_TYPE_ACK,
            .seq = frame.seq,
            .sender_id = frame.sender_id,
            .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };

        esp_err_t err = esp_now_send(recv_info->src_addr, (const uint8_t *)&ack, sizeof(ack));
        if (err != ESP_OK) {
            s_display.ack_fail_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "ACKFAIL");
            ESP_LOGW(TAG, "Falha ao enviar ACK: %s", esp_err_to_name(err));
        } else {
            s_display.ack_ok_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "ACKOK");
        }
    }
#endif

#if CONFIG_ESPNOW_TEST_ROLE_TX
    if (frame.type == ESPNOW_TEST_TYPE_ACK) {
        g_ack_seq = frame.seq;
        g_ack_received = true;
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

#if CONFIG_ESPNOW_TEST_ROLE_RX
    esp_err_t oled_err = oled_init();
    if (oled_err != ESP_OK) {
        ESP_LOGW(TAG, "OLED indisponivel no teste ESPNOW: %s", esp_err_to_name(oled_err));
    }
#endif

    uint8_t self_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(self_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG,
             "MAC local STA: %02X:%02X:%02X:%02X:%02X:%02X",
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);

#if CONFIG_ESPNOW_TEST_ROLE_TX
    uint8_t peer_mac[6] = {0};
    if (!parse_mac_str(CONFIG_ESPNOW_TEST_PEER_MAC, peer_mac)) {
        ESP_LOGE(TAG, "MAC invalido em CONFIG_ESPNOW_TEST_PEER_MAC: %s", CONFIG_ESPNOW_TEST_PEER_MAC);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG,
             "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    esp_err_t add_peer_err = ensure_peer_registered(peer_mac);
    if (add_peer_err != ESP_OK && add_peer_err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Falha ao adicionar peer: %s", esp_err_to_name(add_peer_err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    memcpy((void *)s_display.peer_mac, peer_mac, sizeof(s_display.peer_mac));
    s_display.peer_known = true;
    snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "READY");

    uint32_t sender_id = ((uint32_t)self_mac[2] << 24) |
                         ((uint32_t)self_mac[3] << 16) |
                         ((uint32_t)self_mac[4] << 8) |
                         (uint32_t)self_mac[5];

    while (1) {
        espnow_test_frame_t frame = {
            .type = ESPNOW_TEST_TYPE_PING,
            .seq = ++g_sequence,
            .sender_id = sender_id,
            .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
        };

        g_tx_done = false;
        g_tx_status = ESP_NOW_SEND_FAIL;
        g_ack_received = false;
        g_last_ack_rssi_dbm = INT16_MIN;

        s_display.tx_count++;
        s_display.last_seq = frame.seq;
        snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "TX");

        esp_err_t err = esp_now_send(peer_mac, (const uint8_t *)&frame, sizeof(frame));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_send falhou: %s", esp_err_to_name(err));
            s_display.ack_fail_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "SENDERR");
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
            continue;
        }

        int wait_steps = ESPNOW_TEST_ACK_TIMEOUT_MS / 10;
        for (int i = 0; i < wait_steps && !g_tx_done; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!g_tx_done || g_tx_status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGW(TAG, "TX sem confirmacao MAC (seq=%lu)", (unsigned long)frame.seq);
            s_display.ack_fail_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "TXFAIL");
            vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
            continue;
        }

        for (int i = 0; i < wait_steps && !g_ack_received; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (g_ack_received && g_ack_seq == frame.seq) {
            ESP_LOGI(TAG, "PING OK seq=%lu", (unsigned long)frame.seq);
            s_display.ack_ok_count++;
            s_display.last_rssi_dbm = g_last_ack_rssi_dbm;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "ACKOK");
        } else {
            ESP_LOGW(TAG, "ACK timeout seq=%lu", (unsigned long)frame.seq);
            s_display.ack_fail_count++;
            snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "ACKTO");
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_TEST_SEND_INTERVAL_MS));
    }
#else
    snprintf((char *)s_display.last_status, sizeof(s_display.last_status), "WAIT");
    oled_render_status();
    while (1) {
        oled_render_status();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

#else

void espnow_test_run(void)
{
}

#endif
