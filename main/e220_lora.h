#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "lora_test_config.h"

/*
 * Driver do modulo EBYTE E220-900T22 (LLCC68) via UART em modo transparente.
 *
 * Expoe init / start_rx / transmit / receive_event por fila de eventos,
 * com a API e220_lora_*.
 *
 * Controle por pinos M0/M1 (modo) + AUX (busy/ready). Configuracao dos
 * registradores e feita uma unica vez no init (modo de configuracao).
 */

/* Sub-pacote maximo do E220 em modo transparente. */
#define E220_LORA_MAX_PAYLOAD_LEN 200U

/*
 * Fiacao fixa do E220 na placa (esquematico U3 ESP32-C6-W-1-N8, conector J3):
 *   GPIO12 -> RX LoRa (RXD do E220; TX do ESP)
 *   GPIO13 -> TX LoRa (TXD do E220; RX do ESP)
 *   GPIO18 -> M0 (saida)
 *   GPIO19 -> M1 (saida)
 *   GPIO11 -> AUX (entrada; busy=0/ready=1)
 */
#define E220_PIN_TX  12
#define E220_PIN_RX  13
#define E220_PIN_M0  18
#define E220_PIN_M1  19
#define E220_PIN_AUX 11

typedef enum {
    E220_LORA_EVENT_NONE = 0,
    E220_LORA_EVENT_RX_DONE,
    E220_LORA_EVENT_TX_DONE,
    E220_LORA_EVENT_RX_ERROR,
} e220_lora_event_type_t;

typedef struct {
    /* --- UART / pinos de controle (todos configuraveis; -1 = invalido) --- */
    int uart_port;   /* ex.: 1 (UART0 e o console no ESP32-C6) */
    int pin_tx;      /* TX do ESP -> RXD do E220 */
    int pin_rx;      /* RX do ESP <- TXD do E220 */
    int pin_m0;      /* saida: bit0 de modo */
    int pin_m1;      /* saida: bit1 de modo */
    int pin_aux;     /* entrada: busy(0)/ready(1) */
    int baud;        /* baud da UART; deve casar com o baud gravado no modulo */

    /* --- Parametros de radio --- */
    uint16_t address;        /* ADDH:ADDL */
    uint8_t  channel;        /* registrador CH */
    uint8_t  air_data_rate;  /* codigo 3 bits (REG0[2:0]); ver datasheet */
    int8_t   tx_power_dbm;    /* arredonda para 22/17/13/10 dBm */
    uint16_t wor_period_ms;  /* 0 = WOR desabilitado */
    bool     fixed_mode;     /* false = transparente (recomendado) */
    bool     rssi_byte;      /* true = E220 anexa byte de RSSI ao pacote RX */

    /* true = pula a gravacao de registradores no init (eles sao nao-volateis
     * no E220; num re-wake de deep sleep a config do cold boot ainda vale).
     * Economiza ~50-100 ms (e ate ~1,5 s de timeouts) por acordada. */
    bool     skip_radio_config;

    uint32_t frequency_hz;   /* apenas para log */
} e220_lora_config_t;

typedef struct {
    e220_lora_event_type_t type;
    size_t  payload_len;
    int16_t rssi_dbm;   /* INT16_MIN se indisponivel */
    int8_t  snr_db;     /* INT8_MIN: indisponivel no E220 transparente */
    uint8_t payload[E220_LORA_MAX_PAYLOAD_LEN];
} e220_lora_event_t;

esp_err_t e220_lora_init(const e220_lora_config_t *config);
esp_err_t e220_lora_deinit(void);
esp_err_t e220_lora_start_rx_continuous(void);

/* Poe o radio em sleep (M0=M1=1, ~2 uA). Registradores sao retidos.
 * Acordar: e220_lora_start_rx_continuous() (volta ao modo normal). */
esp_err_t e220_lora_sleep(void);

/* Prepara o E220 para o deep sleep do ESP: radio em sleep + hold dos pinos
 * M0/M1 (senao os GPIOs flutuam no deep sleep e o E220 volta sozinho a RX
 * continuo, ~10-12 mA). Chamar imediatamente antes de esp_deep_sleep_start().
 * O proximo e220_lora_init() solta os holds automaticamente. */
esp_err_t e220_lora_prepare_deep_sleep(void);
esp_err_t e220_lora_transmit(const uint8_t *payload, size_t payload_len);
esp_err_t e220_lora_receive_event(e220_lora_event_t *event, TickType_t timeout);
void e220_lora_flush_events(void);
void e220_lora_debug_dump(void);
