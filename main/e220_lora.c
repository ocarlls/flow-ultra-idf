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
#define E220_AUX_TIMEOUT_MS 1500
#define E220_MODE_SETTLE_MS 50
/* Gap de idle da UART usado p/ delimitar 1 pacote em modo transparente */
#define E220_RX_GAP_MS 20

/* Modos via M1:M0 (E220 transparente). */
typedef enum {
    E220_MODE_NORMAL = 0,   /* M0=0 M1=0 : TX/RX transparente */
    E220_MODE_WOR_TX = 1,   /* M0=1 M1=0 : WOR transmissor */
    E220_MODE_WOR_RX = 2,   /* M0=0 M1=1 : modo 2 = WOR receptor */
    E220_MODE_CONFIG = 3,   /* M0=1 M1=1 : modo 3 = deep sleep/configuracao (registradores) */
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
    int waited = 0;
    while (gpio_get_level(s_ctx.cfg.pin_aux) == 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
        waited += 2;
        if (waited >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
    }
    /* AUX estavel alto por uma folga curta */
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

/* Escreve M0/M1 e SEGURA os niveis com gpio_hold: com PM_SLP_DISABLE_GPIO, o
 * auto light sleep desliga os GPIOs e os pull-ups internos do E220 puxariam
 * M0/M1 p/ alto (modo sleep) no meio de uma escuta. O hold congela o nivel em
 * light E deep sleep. */
static esp_err_t e220_write_mode_pins(int m0, int m1)
{
    if (s_ctx.cfg.pin_m0 >= 0) {
        (void)gpio_hold_dis(s_ctx.cfg.pin_m0);
        ESP_RETURN_ON_ERROR(gpio_set_level(s_ctx.cfg.pin_m0, m0), TAG, "set M0");
        (void)gpio_hold_en(s_ctx.cfg.pin_m0);
    }
    if (s_ctx.cfg.pin_m1 >= 0) {
        (void)gpio_hold_dis(s_ctx.cfg.pin_m1);
        ESP_RETURN_ON_ERROR(gpio_set_level(s_ctx.cfg.pin_m1, m1), TAG, "set M1");
        (void)gpio_hold_en(s_ctx.cfg.pin_m1);
    }
    return ESP_OK;
}

static esp_err_t e220_set_mode(e220_mode_t mode)
{
    /* M0/M1 por modo (manual pg 11): 0=normal(0,0) 1=WOR_TX(1,0) 2=WOR_RX(0,1) 3=config(1,1) */
    int m0 = (mode == E220_MODE_WOR_TX || mode == E220_MODE_CONFIG) ? 1 : 0;
    int m1 = (mode == E220_MODE_WOR_RX || mode == E220_MODE_CONFIG) ? 1 : 0;

    ESP_RETURN_ON_ERROR(e220_write_mode_pins(m0, m1), TAG, "modo M0/M1");
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

    int aux_before = (c->pin_aux >= 0) ? gpio_get_level(c->pin_aux) : -1;
    ESP_LOGW(TAG, "Config diag: AUX=%d antes do modo config (M0=%d M1=%d)",
             aux_before, c->pin_m0, c->pin_m1);

    ESP_RETURN_ON_ERROR(e220_set_mode(E220_MODE_CONFIG), TAG, "entrar config");

    int aux_after = (c->pin_aux >= 0) ? gpio_get_level(c->pin_aux) : -1;
    ESP_LOGW(TAG, "Config diag: AUX=%d apos modo config (M0=1 M1=1 definidos)",
             aux_after);

    /* O modo de configuracao do E220 SEMPRE usa 9600 8N1, independente do baud de
     * operacao (manual pg 13). A UART ja esta em 8N1 (init); aqui ajustamos so o
     * baud para 9600 durante a troca de comandos e restauramos no fim. Apos sair
     * da config o modulo opera no baud gravado no REG0 (= c->baud). */
    const bool baud_switched = (c->baud != 9600);
    if (baud_switched) {
        (void)uart_set_baudrate(c->uart_port, 9600);
    }

    uint8_t cmd[9] = {0xC0, 0x00, 0x06, addh, addl, reg0, reg1, reg2, reg3};
    ESP_LOGW(TAG, "Config diag: enviando cmd [%02X %02X %02X %02X %02X %02X %02X %02X %02X]",
             cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7], cmd[8]);
    uart_flush(c->uart_port);
    int w = uart_write_bytes(c->uart_port, (const char *)cmd, sizeof(cmd));
    ESP_LOGW(TAG, "Config diag: uart_write_bytes retornou %d (esperado 9)", w);

    esp_err_t result;
    if (w != (int)sizeof(cmd)) {
        result = ESP_FAIL;
    } else {
        (void)uart_wait_tx_done(c->uart_port, pdMS_TO_TICKS(200));
        uint8_t resp[9] = {0};
        int r = uart_read_bytes(c->uart_port, resp, sizeof(resp), pdMS_TO_TICKS(500));
        ESP_LOGW(TAG, "Config diag: uart_read_bytes retornou %d bytes", r);
        for (int i = 0; i < r && i < 9; i++) {
            ESP_LOGW(TAG, "  resp[%d]=0x%02X", i, resp[i]);
        }
        if (r >= 4 && resp[0] == 0xC1) {
            if (r >= 9) {
                ESP_LOGI(TAG, "Config OK (lido): ADDH=%02X ADDL=%02X REG0=%02X REG1=%02X REG2=%02X REG3=%02X",
                         resp[3], resp[4], resp[5], resp[6], resp[7], resp[8]);
            } else {
                ESP_LOGI(TAG, "Config aceita (resposta curta r=%d)", r);
            }
            result = ESP_OK;
        } else {
            ESP_LOGE(TAG, "Resposta de config inesperada (r=%d, byte0=0x%02X)",
                     r, r > 0 ? resp[0] : 0);
            result = ESP_ERR_INVALID_RESPONSE;
        }
    }

    /* Restaura o baud de operacao para o RX/TX normal subsequente. */
    if (baud_switched) {
        (void)uart_set_baudrate(c->uart_port, c->baud);
    }
    /* Aguarda AUX alto apos a transacao: o modulo pode estar ocupado
     * processando o comando de config. Sem esta espera, a transicao para
     * modo normal logo em seguida pode falhar (AUX ainda baixo). */
    if (result == ESP_OK) {
        esp_err_t aux_err = e220_wait_aux_high(E220_AUX_TIMEOUT_MS);
        if (aux_err != ESP_OK) {
            ESP_LOGW(TAG, "AUX nao subiu apos config (%s)", esp_err_to_name(aux_err));
        }
    }
    return result;
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

    /* GPIO de modo (saidas) e AUX (entrada). Solta holds de um deep sleep
     * anterior (prepare_deep_sleep) antes de reconfigurar. */
    uint64_t out_mask = 0;
    if (config->pin_m0 >= 0) out_mask |= (1ULL << config->pin_m0);
    if (config->pin_m1 >= 0) out_mask |= (1ULL << config->pin_m1);
    if (config->pin_m0 >= 0) (void)gpio_hold_dis(config->pin_m0);
    if (config->pin_m1 >= 0) (void)gpio_hold_dis(config->pin_m1);
    if (out_mask) {
        gpio_config_t oc = {.pin_bit_mask = out_mask, .mode = GPIO_MODE_OUTPUT};
        ESP_RETURN_ON_ERROR(gpio_config(&oc), TAG, "gpio M0/M1");
    }
    if (config->pin_aux >= 0) {
        gpio_config_t ic = {.pin_bit_mask = (1ULL << config->pin_aux), .mode = GPIO_MODE_INPUT,
                            .pull_up_en = GPIO_PULLUP_ENABLE};
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

    /* Grava configuracao (pulavel em re-wake: registradores sao nao-volateis)
     * e volta ao modo normal. */
    if (config->skip_radio_config) {
        ESP_LOGI(TAG, "Config de registradores pulada (re-wake; config e nao-volatil)");
    } else {
        esp_err_t err = e220_write_config();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Falha na config inicial (%s) — seguindo em modo normal", esp_err_to_name(err));
        }
        /* Aguarda o modulo terminar de gravar os registradores nao-volateis
         * antes de trocar o modo. Sem esta pausa a transicao pode falhar. */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* Entrada em RX e o degrau de corrente do modulo: um sag transiente da
     * fonte pode segurar o AUX baixo. Tenta 2x; se falhar, deixa o radio em
     * SLEEP (M0=M1=1) para nao drenar ~12 mA enquanto o chamador decide. */
    esp_err_t mode_err = e220_set_mode(E220_MODE_NORMAL);
    if (mode_err != ESP_OK) {
        ESP_LOGW(TAG, "AUX nao subiu no modo normal (%s); re-tentando", esp_err_to_name(mode_err));
        vTaskDelay(pdMS_TO_TICKS(250));
        mode_err = e220_set_mode(E220_MODE_NORMAL);
    }
    if (mode_err != ESP_OK) {
        (void)e220_write_mode_pins(1, 1); /* radio em sleep, pinos em hold */
        ESP_LOGE(TAG, "modo normal falhou (%s); radio deixado em sleep", esp_err_to_name(mode_err));
        return mode_err;
    }

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

esp_err_t e220_lora_sleep(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* M0=M1=1 = modo 3 (sleep/config): radio desliga RX/TX, registradores
     * retidos, ~2 uA. Nao espera AUX (em sleep ele pode ficar baixo). */
    ESP_RETURN_ON_ERROR(e220_write_mode_pins(1, 1), TAG, "sleep M0/M1");
    vTaskDelay(pdMS_TO_TICKS(E220_MODE_SETTLE_MS));
    return ESP_OK;
}

esp_err_t e220_lora_prepare_deep_sleep(void)
{
    esp_err_t err = e220_lora_sleep();
    if (err != ESP_OK) {
        return err;
    }
    /* Congela M0/M1 em nivel alto atraves do deep sleep do ESP; sem isso os
     * pinos flutuam e o E220 pode voltar a RX continuo (~10-12 mA). */
    if (s_ctx.cfg.pin_m0 >= 0) {
        ESP_RETURN_ON_ERROR(gpio_hold_en(s_ctx.cfg.pin_m0), TAG, "hold M0");
    }
    if (s_ctx.cfg.pin_m1 >= 0) {
        ESP_RETURN_ON_ERROR(gpio_hold_en(s_ctx.cfg.pin_m1), TAG, "hold M1");
    }
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    /* Em chips sem hold individual em deep sleep (ex.: ESP32 classico) o hold
     * global precisa ser armado; no C6 o gpio_hold_en por pino ja persiste. */
    gpio_deep_sleep_hold_en();
#endif
    return ESP_OK;
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
