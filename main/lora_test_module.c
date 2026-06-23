#include "lora_test_module.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "e220_lora.h"

/* TX_DONE e sintetizado a partir do pino AUX do E220 (modo transparente nao tem
 * IRQ de TX_DONE). Limite superior generoso para cobrir o air-time do pacote no
 * menor air data rate; o evento normalmente chega bem antes. */
#define LORA_TEST_TX_DONE_TIMEOUT_MS 6000U
#define LORA_TEST_ACK_TIMEOUT_MS (LORA_TEST_TX_DONE_TIMEOUT_MS + 2000U)
#define LORA_TEST_MAX_NODES 32
#define LORA_TEST_OLED_CMD_BITS 8
#define LORA_TEST_OLED_PARAM_BITS 8
#define LORA_TEST_OLED_FLUSH_TIMEOUT_MS 250
#define LORA_TEST_OLED_TEXT_COLS (LORA_TEST_OLED_WIDTH / 6)
#define LORA_TEST_OLED_TEXT_ROWS (LORA_TEST_OLED_HEIGHT / 8)
#define LORA_TEST_OLED_FB_SIZE ((LORA_TEST_OLED_WIDTH * LORA_TEST_OLED_HEIGHT) / 8)

typedef enum {
    LORA_TEST_FRAME_TYPE_PROBE = 0xA1,
    LORA_TEST_FRAME_TYPE_ACK = 0xA2,
} lora_test_frame_type_t;

typedef enum {
    LORA_TEST_NODE_ACK_STATUS_IDLE = 0,
    LORA_TEST_NODE_ACK_STATUS_TXWAIT,
    LORA_TEST_NODE_ACK_STATUS_WAITACK,
    LORA_TEST_NODE_ACK_STATUS_OK,
    LORA_TEST_NODE_ACK_STATUS_TXFAIL,
    LORA_TEST_NODE_ACK_STATUS_TDOUT,
    LORA_TEST_NODE_ACK_STATUS_TIMEOUT,
    LORA_TEST_NODE_ACK_STATUS_ERROR,
} lora_test_node_ack_status_t;

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
    bool has_first_seq;
    bool has_last_seq;
    uint8_t mac[6];
    uint32_t first_seq;
    uint32_t last_seq;
    uint32_t rx_count;
    uint32_t unique_rx_count;
    uint32_t duplicate_count;
    uint32_t ack_ok_count;
    uint32_t ack_fail_count;
    int16_t last_rssi_dbm;
    int8_t last_snr_db;
} lora_node_stats_t;

typedef struct {
    uint32_t tx_count;
    uint32_t ack_ok_count;
    uint32_t ack_fail_count;
    uint32_t last_seq;
    int16_t last_tx_rssi_dbm;
    int16_t last_rx_rssi_dbm;
    int8_t last_snr_db;
    lora_test_node_ack_status_t last_ack_status;
} lora_node_view_t;

typedef struct {
    char ch;
    uint8_t columns[5];
} lora_oled_glyph_t;

typedef struct {
    bool available;
    i2c_master_bus_handle_t i2c_bus;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    SemaphoreHandle_t flush_done_sem;
    uint8_t framebuffer[LORA_TEST_OLED_FB_SIZE];
} lora_oled_ctx_t;

static void mac_to_str(const uint8_t mac[6], char *buffer, size_t buffer_len);

static const lora_oled_glyph_t s_oled_font[] = {
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

static const char *TAG = "LORA_TEST";
static lora_node_stats_t s_nodes[LORA_TEST_MAX_NODES] = {0};
static lora_node_stats_t *s_last_active_node = NULL;
static lora_node_view_t s_node_view = {
    .last_tx_rssi_dbm = INT16_MIN,
    .last_rx_rssi_dbm = INT16_MIN,
    .last_snr_db = INT8_MIN,
    .last_ack_status = LORA_TEST_NODE_ACK_STATUS_IDLE,
};
static lora_oled_ctx_t s_oled = {0};

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

static void reset_runtime_state(void)
{
    memset(s_nodes, 0, sizeof(s_nodes));
    s_last_active_node = NULL;

    memset(&s_node_view, 0, sizeof(s_node_view));
    s_node_view.last_tx_rssi_dbm = INT16_MIN;
    s_node_view.last_rx_rssi_dbm = INT16_MIN;
    s_node_view.last_snr_db = INT8_MIN;
    s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_IDLE;
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
    if (node == NULL || !node->has_first_seq || !node->has_last_seq || node->last_seq < node->first_seq) {
        return 0U;
    }

    return node->last_seq - node->first_seq + 1U;
}

static const char *node_ack_status_text(lora_test_node_ack_status_t status)
{
    switch (status) {
    case LORA_TEST_NODE_ACK_STATUS_TXWAIT:
        return "TXWAIT";
    case LORA_TEST_NODE_ACK_STATUS_WAITACK:
        return "WAITACK";
    case LORA_TEST_NODE_ACK_STATUS_OK:
        return "OK";
    case LORA_TEST_NODE_ACK_STATUS_TXFAIL:
        return "TXFAIL";
    case LORA_TEST_NODE_ACK_STATUS_TDOUT:
        return "TDOUT";
    case LORA_TEST_NODE_ACK_STATUS_TIMEOUT:
        return "TIMEOUT";
    case LORA_TEST_NODE_ACK_STATUS_ERROR:
        return "ERROR";
    case LORA_TEST_NODE_ACK_STATUS_IDLE:
    default:
        return "IDLE";
    }
}

static void format_dbm_value(int16_t value, bool valid, char *buffer, size_t buffer_len)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--");
        return;
    }

    snprintf(buffer, buffer_len, "%dDBM", (int)value);
}

static void format_db_value(int value, bool valid, char *buffer, size_t buffer_len)
{
    if (!valid) {
        snprintf(buffer, buffer_len, "--");
        return;
    }

    snprintf(buffer, buffer_len, "%dDB", value);
}

static void format_percent_value(uint32_t numerator, uint32_t denominator, char *buffer, size_t buffer_len)
{
    uint32_t percent_tenths = compute_percent_tenths(numerator, denominator);
    snprintf(buffer, buffer_len, "%lu.%lu%%",
             (unsigned long)(percent_tenths / 10U),
             (unsigned long)(percent_tenths % 10U));
}

static bool oled_flush_done_cb(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_io_event_data_t *edata,
                               void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    lora_oled_ctx_t *oled = (lora_oled_ctx_t *)user_ctx;
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

    /* Ativa o Vext da placa (Heltec LoRa32 v2) para ligar o OLED */
    gpio_reset_pin(21);
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_level(21, 0); /* LOW = Liga Vext */
    vTaskDelay(pdMS_TO_TICKS(50)); /* Tempo p/ estabilizar a tensão */

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
        .lcd_cmd_bits = LORA_TEST_OLED_CMD_BITS,
        .lcd_param_bits = LORA_TEST_OLED_PARAM_BITS,
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
    if (!s_oled.available || col < 0 || col >= LORA_TEST_OLED_TEXT_COLS || row < 0 || row >= LORA_TEST_OLED_TEXT_ROWS) {
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
    if (!s_oled.available || row < 0 || row >= LORA_TEST_OLED_TEXT_ROWS || text == NULL) {
        return;
    }

    for (int col = 0; col < LORA_TEST_OLED_TEXT_COLS && text[col] != '\0'; ++col) {
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

    if (xSemaphoreTake(s_oled.flush_done_sem, pdMS_TO_TICKS(LORA_TEST_OLED_FLUSH_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void oled_render_root_view(void)
{
    if (!s_oled.available) {
        return;
    }

    const lora_node_stats_t *node = s_last_active_node;
    char line[LORA_TEST_OLED_TEXT_ROWS][LORA_TEST_OLED_TEXT_COLS + 1] = {{0}};
    char rssi_text[16] = {0};
    char snr_text[16] = {0};
    char pdr_text[16] = {0};
    char seq_text[16] = {0};
    char mac_text[18] = "--:--:--:--:--:--";
    uint32_t packet_count = 0;
    uint32_t ack_ok_count = 0;
    uint32_t ack_fail_count = 0;
    uint32_t expected_packets = 0;

    if (node != NULL) {
        mac_to_str(node->mac, mac_text, sizeof(mac_text));
        packet_count = node->unique_rx_count;
        ack_ok_count = node->ack_ok_count;
        ack_fail_count = node->ack_fail_count;
        expected_packets = root_expected_packets(node);
        format_dbm_value(node->last_rssi_dbm, true, rssi_text, sizeof(rssi_text));
        format_db_value((int)node->last_snr_db, true, snr_text, sizeof(snr_text));
        format_percent_value(node->unique_rx_count, expected_packets, pdr_text, sizeof(pdr_text));
        snprintf(seq_text, sizeof(seq_text), "%lu", (unsigned long)node->last_seq);
    } else {
        format_dbm_value(0, false, rssi_text, sizeof(rssi_text));
        format_db_value(0, false, snr_text, sizeof(snr_text));
        format_percent_value(0, 0, pdr_text, sizeof(pdr_text));
        snprintf(seq_text, sizeof(seq_text), "--");
    }

    snprintf(line[0], sizeof(line[0]), "ROOT %s", node != NULL ? "LAST NODE" : "WAIT NODE");
    snprintf(line[1], sizeof(line[1]), "MAC:%s", mac_text);
    snprintf(line[2], sizeof(line[2]), "RSSI:%s", rssi_text);
    snprintf(line[3], sizeof(line[3]), "SNR:%s", snr_text);
    snprintf(line[4], sizeof(line[4]), "PKT:%lu", (unsigned long)packet_count);
    snprintf(line[5], sizeof(line[5]), "ACK:%lu/%lu", (unsigned long)ack_ok_count, (unsigned long)ack_fail_count);
    snprintf(line[6], sizeof(line[6]), "PDR:%s", pdr_text);
    snprintf(line[7], sizeof(line[7]), "SEQ:%s", seq_text);

    oled_clear();
    for (int i = 0; i < LORA_TEST_OLED_TEXT_ROWS; ++i) {
        oled_draw_text(i, line[i]);
    }

    esp_err_t err = oled_flush();
    if (err != ESP_OK) {
        oled_disable_after_error(err, "root_flush");
    }
}

static void oled_render_node_view(void)
{
    if (!s_oled.available) {
        return;
    }

    char line[LORA_TEST_OLED_TEXT_ROWS][LORA_TEST_OLED_TEXT_COLS + 1] = {{0}};
    char tx_rssi_text[12] = {0};
    char rx_rssi_text[12] = {0};
    char snr_text[12] = {0};
    char pdr_text[12] = {0};

    format_dbm_value(s_node_view.last_tx_rssi_dbm,
                     s_node_view.last_tx_rssi_dbm != INT16_MIN,
                     tx_rssi_text,
                     sizeof(tx_rssi_text));
    format_dbm_value(s_node_view.last_rx_rssi_dbm,
                     s_node_view.last_rx_rssi_dbm != INT16_MIN,
                     rx_rssi_text,
                     sizeof(rx_rssi_text));
    format_db_value((int)s_node_view.last_snr_db,
                    s_node_view.last_snr_db != INT8_MIN,
                    snr_text,
                    sizeof(snr_text));
    format_percent_value(s_node_view.ack_ok_count, s_node_view.tx_count, pdr_text, sizeof(pdr_text));

    snprintf(line[0], sizeof(line[0]), "NODE STATUS");
    snprintf(line[1], sizeof(line[1]), "TXRSSI:%s", tx_rssi_text);
    snprintf(line[2], sizeof(line[2]), "RXRSSI:%s", rx_rssi_text);
    snprintf(line[3], sizeof(line[3]), "SNR:%s", snr_text);
    snprintf(line[4], sizeof(line[4]), "ACK:%s", node_ack_status_text(s_node_view.last_ack_status));
    snprintf(line[5], sizeof(line[5]), "PDR:%s", pdr_text);
    snprintf(line[6], sizeof(line[6]), "TX:%lu OK:%lu",
             (unsigned long)s_node_view.tx_count,
             (unsigned long)s_node_view.ack_ok_count);
    snprintf(line[7], sizeof(line[7]), "FAIL:%lu", (unsigned long)s_node_view.ack_fail_count);

    oled_clear();
    for (int i = 0; i < LORA_TEST_OLED_TEXT_ROWS; ++i) {
        oled_draw_text(i, line[i]);
    }

    esp_err_t err = oled_flush();
    if (err != ESP_OK) {
        oled_disable_after_error(err, "node_flush");
    }
}

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
        e220_lora_event_t event = {0};
        int64_t remaining_ms = (deadline_us - esp_timer_get_time()) / 1000LL;
        TickType_t wait = pdMS_TO_TICKS((remaining_ms > 50) ? 50 : (remaining_ms > 0 ? remaining_ms : 1));
        esp_err_t err = e220_lora_receive_event(&event, wait);
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
             CONFIG_FLOW_E220_UART_PORT, CONFIG_FLOW_E220_PIN_TX, CONFIG_FLOW_E220_PIN_RX,
             CONFIG_FLOW_E220_PIN_M0, CONFIG_FLOW_E220_PIN_M1, CONFIG_FLOW_E220_PIN_AUX,
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

        char mac_buffer[18] = {0};
        char pdr_buffer[16] = {0};
        uint32_t expected_packets = root_expected_packets(&s_nodes[i]);

        mac_to_str(s_nodes[i].mac, mac_buffer, sizeof(mac_buffer));
        format_percent_value(s_nodes[i].unique_rx_count, expected_packets, pdr_buffer, sizeof(pdr_buffer));
        ESP_LOGI(TAG,
                 "  mac=%s pkt=%lu dup=%lu ack_ok=%lu ack_fail=%lu last_seq=%lu pdr=%s rssi=%d snr=%d",
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
        .uart_port = CONFIG_FLOW_E220_UART_PORT,
        .pin_tx = CONFIG_FLOW_E220_PIN_TX,
        .pin_rx = CONFIG_FLOW_E220_PIN_RX,
        .pin_m0 = CONFIG_FLOW_E220_PIN_M0,
        .pin_m1 = CONFIG_FLOW_E220_PIN_M1,
        .pin_aux = CONFIG_FLOW_E220_PIN_AUX,
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
            oled_render_root_view();
            next_summary_us = now_us + ((int64_t)CONFIG_LORA_TEST_ROOT_SUMMARY_INTERVAL_MS * 1000LL);
        }

        int64_t remaining_summary_ms = (next_summary_us - now_us) / 1000LL;
        TickType_t wait = pdMS_TO_TICKS((remaining_summary_ms > 200) ? 200 :
                                        (remaining_summary_ms > 0 ? remaining_summary_ms : 1));

        e220_lora_event_t event = {0};
        esp_err_t err = e220_lora_receive_event(&event, wait);
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

        lora_node_stats_t *node = find_or_allocate_node(probe.mac);
        bool duplicate = false;
        if (node != NULL) {
            node->rx_count++;
            duplicate = node->has_last_seq && (probe.seq <= node->last_seq);
            if (duplicate) {
                node->duplicate_count++;
            } else {
                if (!node->has_first_seq) {
                    node->has_first_seq = true;
                    node->first_seq = probe.seq;
                }
                node->has_last_seq = true;
                node->last_seq = probe.seq;
                node->unique_rx_count++;
            }
            node->last_rssi_dbm = event.rssi_dbm;
            node->last_snr_db = event.snr_db;
            s_last_active_node = node;
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

        oled_render_root_view();

        lora_ack_frame_t ack = {
            .type = LORA_TEST_FRAME_TYPE_ACK,
            .seq = probe.seq,
            .probe_rssi_dbm = event.rssi_dbm,
            .probe_snr_db = event.snr_db,
            .reserved = 0,
            .uptime_ms = uptime_ms_now(),
        };
        memcpy(ack.mac, probe.mac, sizeof(ack.mac));

        e220_lora_flush_events();
        err = e220_lora_transmit((const uint8_t *)&ack, sizeof(ack));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ROOT: falha ao iniciar ACK para %s: %s", mac_buffer, esp_err_to_name(err));
            if (node != NULL) {
                node->ack_fail_count++;
            }
            oled_render_root_view();
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
            /* Aguarda 500 ms antes de voltar ao RX para evitar que o proprio
             * sinal de TX do ACK interfira no proximo PROBE recebido. */
            vTaskDelay(pdMS_TO_TICKS(500));
            e220_lora_flush_events();
        } else {
            ESP_LOGW(TAG, "ROOT: timeout aguardando TX_DONE do ACK para %s", mac_buffer);
            if (node != NULL) {
                node->ack_fail_count++;
            }
        }

        oled_render_root_view();
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

        s_node_view.tx_count++;
        s_node_view.last_seq = probe.seq;
        s_node_view.last_tx_rssi_dbm = INT16_MIN;
        s_node_view.last_rx_rssi_dbm = INT16_MIN;
        s_node_view.last_snr_db = INT8_MIN;
        s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_TXWAIT;
        oled_render_node_view();

        e220_lora_flush_events();
        esp_err_t err = e220_lora_transmit((const uint8_t *)&probe, sizeof(probe));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NODE: falha ao iniciar PROBE seq=%lu: %s",
                     (unsigned long)probe.seq,
                     esp_err_to_name(err));
            s_node_view.ack_fail_count++;
            s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_TXFAIL;
            oled_render_node_view();
        } else {
            err = wait_for_tx_done(LORA_TEST_TX_DONE_TIMEOUT_MS);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "NODE: timeout aguardando TX_DONE seq=%lu", (unsigned long)probe.seq);
                s_node_view.ack_fail_count++;
                s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_TDOUT;
                oled_render_node_view();
            } else {
                bool ack_received = false;
                bool ack_wait_error = false;
                int64_t ack_deadline_us = esp_timer_get_time() +
                                          ((int64_t)LORA_TEST_ACK_TIMEOUT_MS * 1000LL);

                s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_WAITACK;
                oled_render_node_view();

                while (esp_timer_get_time() < ack_deadline_us) {
                    e220_lora_event_t event = {0};
                    int64_t remaining_ms = (ack_deadline_us - esp_timer_get_time()) / 1000LL;
                    TickType_t wait = pdMS_TO_TICKS((remaining_ms > 100) ? 100 :
                                                    (remaining_ms > 0 ? remaining_ms : 1));
                    err = e220_lora_receive_event(&event, wait);
                    if (err == ESP_ERR_TIMEOUT) {
                        continue;
                    }
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "NODE: erro ao aguardar ACK seq=%lu: %s",
                                 (unsigned long)probe.seq,
                                 esp_err_to_name(err));
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
                    s_node_view.ack_ok_count++;
                    s_node_view.last_tx_rssi_dbm = ack.probe_rssi_dbm;
                    s_node_view.last_rx_rssi_dbm = event.rssi_dbm;
                    s_node_view.last_snr_db = event.snr_db;
                    s_node_view.last_ack_status = LORA_TEST_NODE_ACK_STATUS_OK;
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

                oled_render_node_view();
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

    esp_err_t oled_err = oled_init();
    if (oled_err != ESP_OK) {
        ESP_LOGW(TAG, "OLED nao inicializado: %s", esp_err_to_name(oled_err));
    }

#if CONFIG_LORA_TEST_ROLE_ROOT
    oled_render_root_view();
#else
    oled_render_node_view();
#endif

    ESP_ERROR_CHECK(init_lora_radio());

#if CONFIG_LORA_TEST_ROLE_ROOT
    run_root(self_mac);
#else
    run_node(self_mac);
#endif
}