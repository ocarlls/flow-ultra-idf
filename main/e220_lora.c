#include "e220_lora.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "E220";

#define E220_EVENT_QUEUE_LEN 8
#define E220_UART_RX_BUF (E220_LORA_MAX_PAYLOAD_LEN * 4 + 16)
#define E220_TASK_STACK 4096
#define E220_TASK_PRIO 10

/* AUX timings (ms) */
#define E220_AUX_TIMEOUT_MS 1000
#define E220_MODE_SETTLE_MS 10
/* Gap de idle da UART usado p/ delimitar 1 pacote em modo transparente */
#define E220_RX_GAP_MS 20

/* Modos via M1:M0 (E220 transparente). */
typedef enum {
    E220_MODE_NORMAL = 0,   /* M0=0 M1=0 : TX/RX transparente */
    E220_MODE_WOR_TX = 1,   /* M0=1 M1=0 : WOR transmissor */
    E220_MODE_WOR_RX = 2,   /* M0=0 M1=1 : WOR receptor (tambem usado p/ config) */
    E220_MODE_CONFIG = 3,   /* M0=0 M1=1 : modo de configuracao (deep sleep/config) */
} e220_mode_t;

typedef struct {
    bool initialized;
    e220_lora_config_t cfg;
    QueueHandle_t event_queue;
    TaskHandle_t rx_task;
    SemaphoreHandle_t tx_mutex;
    volatile bool stop;
} e220_ctx_t;

static e220_ctx_t s_ctx = {0};

// ---------------------------------------------------------------------------
// Helpers de encoding dos registradores (verificar contra o datasheet E220 na
// hora do bring-up; valores marcados com a referencia do manual).
// ---------------------------------------------------------------------------
static uint8_t e220_baud_code(int baud)
{
    switch (baud) {
        case 1200:   return 0;
        case 2400:   return 1;
        case 4800:   return 2;
        case 9600:   return 3;
        case 19200:  return 4;
        case 38400:  return 5;
        case 57600:  return 6;
        case 115200: return 7;
        default:     return 3; /* 9600 (fabrica) */
    }
}

static uint8_t e220_power_code(int8_t dbm)
{
    /* E220-900T22 REG1[1:0]: 00=22dBm 01=17 10=13 11=10 */
    if (dbm >= 22) return 0;
    if (dbm >= 17) return 1;
    if (dbm >= 13) return 2;
    return 3;
}

static uint8_t e220_wor_code(uint16_t ms)
{
    /* REG3[2:0]: 000=500ms ... 111=4000ms em passos de 500ms */
    if (ms == 0) return 0;
    int code = (ms / 500) - 1;
    if (code < 0) code = 0;
    if (code > 7) code = 7;
    return (uint8_t)code;
}

// ---------------------------------------------------------------------------
// GPIO de modo / AUX
// ---------------------------------------------------------------------------
static esp_err_t e220_wait_aux_high(int timeout_ms)
{
    if (s_ctx.cfg.pin_aux < 0) {
        vTaskDelay(pdMS_TO_TICKS(E220_MODE_SETTLE_MS));
        return ESP_OK; /* sem AUX: usa atraso fixo */
    }
    const int64_t step_ms = 2;
    int waited = 0;
    while (gpio_get_level(s_ctx.cfg.pin_aux) == 0) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
        if (waited >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
    }
    /* AUX estavel alto por uma folga curta */
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

static esp_err_t e220_set_mode(e220_mode_t mode)
{
    int m0 = (mode == E220_MODE_WOR_TX) ? 1 : 0;
    int m1 = (mode == E220_MODE_WOR_RX || mode == E220_MODE_CONFIG) ? 1 : 0;

    if (s_ctx.cfg.pin_m0 >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_level(s_ctx.cfg.pin_m0, m0), TAG, "set M0");
    }
    if (s_ctx.cfg.pin_m1 >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_level(s_ctx.cfg.pin_m1, m1), TAG, "set M1");
    }
    vTaskDelay(pdMS_TO_TICKS(E220_MODE_SETTLE_MS));
    return e220_wait_aux_high(E220_AUX_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Modo de configuracao: grava REG0..REG3 e le de volta para verificar
// ---------------------------------------------------------------------------
static esp_err_t e220_write_config(void)
{
    const e220_lora_config_t *c = &s_ctx.cfg;

    uint8_t addh = (uint8_t)(c->address >> 8);
    uint8_t addl = (uint8_t)(c->address & 0xFF);
    uint8_t reg0 = (uint8_t)((e220_baud_code(c->baud) << 5) | /* parity 8N1 = 00 */
                             (c->air_data_rate & 0x07));
    uint8_t reg1 = (uint8_t)(/* subpacket 200B = 00 */ /* RSSI ambient off */
                             (e220_power_code(c->tx_power_dbm) & 0x03));
    uint8_t reg2 = c->channel;
    uint8_t reg3 = (uint8_t)((c->rssi_byte ? 0x80 : 0x00) |
                             (c->fixed_mode ? 0x40 : 0x00) |
                             (e220_wor_code(c->wor_period_ms) & 0x07));

    ESP_RETURN_ON_ERROR(e220_set_mode(E220_MODE_CONFIG), TAG, "entrar config");

    uint8_t cmd[9] = {0xC0, 0x00, 0x06, addh, addl, reg0, reg1, reg2, reg3};
    uart_flush(c->uart_port);
    int w = uart_write_bytes(c->uart_port, (const char *)cmd, sizeof(cmd));
    if (w != (int)sizeof(cmd)) {
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(c->uart_port, pdMS_TO_TICKS(200));

    uint8_t resp[9] = {0};
    int r = uart_read_bytes(c->uart_port, resp, sizeof(resp), pdMS_TO_TICKS(500));
    if (r >= 4 && resp[0] == 0xC1) {
        ESP_LOGI(TAG, "Config OK: ADDH=%02X ADDL=%02X REG0=%02X REG1=%02X REG2=%02X REG3=%02X",
                 resp[3], resp[4], resp[5 % r], reg1, reg2, reg3);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Resposta de config inesperada (r=%d, byte0=0x%02X)", r, r > 0 ? resp[0] : 0);
    return ESP_ERR_INVALID_RESPONSE;
}

// ---------------------------------------------------------------------------
// Task de RX: enquadra por gap de idle e empurra evento RX_DONE
// ---------------------------------------------------------------------------
static void e220_push_event(const e220_lora_event_t *ev)
{
    if (s_ctx.event_queue == NULL) {
        return;
    }
    if (xQueueSend(s_ctx.event_queue, ev, 0) != pdTRUE) {
        e220_lora_event_t dropped;
        (void)xQueueReceive(s_ctx.event_queue, &dropped, 0);
        (void)xQueueSend(s_ctx.event_queue, ev, 0);
    }
}

static void e220_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[E220_LORA_MAX_PAYLOAD_LEN + 1];

    while (!s_ctx.stop) {
        int n = uart_read_bytes(s_ctx.cfg.uart_port, buf, sizeof(buf),
                                pdMS_TO_TICKS(E220_RX_GAP_MS));
        if (n <= 0) {
            continue;
        }

        e220_lora_event_t ev = {0};
        ev.type = E220_LORA_EVENT_RX_DONE;
        ev.snr_db = INT8_MIN; /* indisponivel */
        ev.rssi_dbm = INT16_MIN;

        size_t len = (size_t)n;
        if (s_ctx.cfg.rssi_byte && len >= 1) {
            uint8_t rssi_raw = buf[len - 1];
            ev.rssi_dbm = (int16_t)(-(256 - (int)rssi_raw));
            len -= 1;
        }
        if (len > E220_LORA_MAX_PAYLOAD_LEN) {
            len = E220_LORA_MAX_PAYLOAD_LEN;
        }
        ev.payload_len = len;
        memcpy(ev.payload, buf, len);
        e220_push_event(&ev);
    }

    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// API publica
// ---------------------------------------------------------------------------
esp_err_t e220_lora_init(const e220_lora_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->uart_port < 0 || config->pin_tx < 0 || config->pin_rx < 0) {
        ESP_LOGE(TAG, "Pinos UART nao configurados (uart=%d tx=%d rx=%d) — defina no Kconfig",
                 config->uart_port, config->pin_tx, config->pin_rx);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.cfg = *config;

    s_ctx.tx_mutex = xSemaphoreCreateMutex();
    s_ctx.event_queue = xQueueCreate(E220_EVENT_QUEUE_LEN, sizeof(e220_lora_event_t));
    if (s_ctx.tx_mutex == NULL || s_ctx.event_queue == NULL) {
        (void)e220_lora_deinit();
        return ESP_ERR_NO_MEM;
    }

    /* GPIO de modo (saidas) e AUX (entrada). */
    uint64_t out_mask = 0;
    if (config->pin_m0 >= 0) out_mask |= (1ULL << config->pin_m0);
    if (config->pin_m1 >= 0) out_mask |= (1ULL << config->pin_m1);
    if (out_mask) {
        gpio_config_t oc = {.pin_bit_mask = out_mask, .mode = GPIO_MODE_OUTPUT};
        ESP_RETURN_ON_ERROR(gpio_config(&oc), TAG, "gpio M0/M1");
    }
    if (config->pin_aux >= 0) {
        gpio_config_t ic = {.pin_bit_mask = (1ULL << config->pin_aux), .mode = GPIO_MODE_INPUT};
        ESP_RETURN_ON_ERROR(gpio_config(&ic), TAG, "gpio AUX");
    }

    /* UART 8N1. */
    uart_config_t uc = {
        .baud_rate = config->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(config->uart_port, E220_UART_RX_BUF, 0, 0, NULL, 0),
                        TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(config->uart_port, &uc), TAG, "uart param");
    ESP_RETURN_ON_ERROR(uart_set_pin(config->uart_port, config->pin_tx, config->pin_rx,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart pins");

    /* Grava configuracao e volta ao modo normal. */
    esp_err_t err = e220_write_config();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha na config inicial (%s) — seguindo em modo normal", esp_err_to_name(err));
    }
    ESP_RETURN_ON_ERROR(e220_set_mode(E220_MODE_NORMAL), TAG, "modo normal");

    s_ctx.stop = false;
    if (xTaskCreate(e220_rx_task, "e220_rx", E220_TASK_STACK, NULL, E220_TASK_PRIO, &s_ctx.rx_task) != pdPASS) {
        (void)e220_lora_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "E220 pronto: uart%d tx=%d rx=%d m0=%d m1=%d aux=%d ch=%u addr=0x%04X freq~%luHz",
             config->uart_port, config->pin_tx, config->pin_rx,
             config->pin_m0, config->pin_m1, config->pin_aux,
             (unsigned)config->channel, (unsigned)config->address,
             (unsigned long)config->frequency_hz);
    return ESP_OK;
}

esp_err_t e220_lora_deinit(void)
{
    s_ctx.stop = true;
    if (s_ctx.rx_task != NULL) {
        /* a task se autodeleta ao ver stop; aguarda um ciclo de leitura */
        vTaskDelay(pdMS_TO_TICKS(E220_RX_GAP_MS + 10));
        s_ctx.rx_task = NULL;
    }
    if (s_ctx.cfg.uart_port >= 0 && uart_is_driver_installed(s_ctx.cfg.uart_port)) {
        (void)uart_driver_delete(s_ctx.cfg.uart_port);
    }
    if (s_ctx.event_queue != NULL) {
        vQueueDelete(s_ctx.event_queue);
        s_ctx.event_queue = NULL;
    }
    if (s_ctx.tx_mutex != NULL) {
        vSemaphoreDelete(s_ctx.tx_mutex);
        s_ctx.tx_mutex = NULL;
    }
    s_ctx.initialized = false;
    return ESP_OK;
}

esp_err_t e220_lora_start_rx_continuous(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Em modo transparente o E220 ja recebe continuamente em modo normal. */
    return e220_set_mode(E220_MODE_NORMAL);
}

esp_err_t e220_lora_transmit(const uint8_t *payload, size_t payload_len)
{
    if (!s_ctx.initialized || payload == NULL || payload_len == 0 ||
        payload_len > E220_LORA_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_ctx.tx_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = e220_wait_aux_high(E220_AUX_TIMEOUT_MS);
    if (err == ESP_OK) {
        int w = uart_write_bytes(s_ctx.cfg.uart_port, (const char *)payload, payload_len);
        err = (w == (int)payload_len) ? ESP_OK : ESP_FAIL;
    }
    if (err == ESP_OK) {
        err = uart_wait_tx_done(s_ctx.cfg.uart_port, pdMS_TO_TICKS(1000));
    }
    /* TX_DONE sintetizado: AUX volta a HIGH apos o envio pelo ar. */
    if (err == ESP_OK) {
        (void)e220_wait_aux_high(E220_AUX_TIMEOUT_MS);
        e220_lora_event_t ev = {0};
        ev.type = E220_LORA_EVENT_TX_DONE;
        ev.snr_db = INT8_MIN;
        ev.rssi_dbm = INT16_MIN;
        e220_push_event(&ev);
    }

    xSemaphoreGive(s_ctx.tx_mutex);
    return err;
}

esp_err_t e220_lora_receive_event(e220_lora_event_t *event, TickType_t timeout)
{
    if (event == NULL || s_ctx.event_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xQueueReceive(s_ctx.event_queue, event, timeout) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

void e220_lora_flush_events(void)
{
    if (s_ctx.event_queue == NULL) {
        return;
    }
    e220_lora_event_t ev;
    while (xQueueReceive(s_ctx.event_queue, &ev, 0) == pdTRUE) {
    }
}

void e220_lora_debug_dump(void)
{
    if (!s_ctx.initialized) {
        return;
    }
    ESP_LOGI(TAG, "E220 dump: ch=%u addr=0x%04X air=%u pwr=%ddBm wor=%ums rssi_byte=%d fixed=%d",
             (unsigned)s_ctx.cfg.channel, (unsigned)s_ctx.cfg.address,
             (unsigned)s_ctx.cfg.air_data_rate, (int)s_ctx.cfg.tx_power_dbm,
             (unsigned)s_ctx.cfg.wor_period_ms, (int)s_ctx.cfg.rssi_byte,
             (int)s_ctx.cfg.fixed_mode);
}
