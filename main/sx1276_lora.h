#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "lora_test_config.h"

#define SX1276_LORA_MAX_PAYLOAD_LEN 255U

typedef enum {
    SX1276_LORA_EVENT_NONE = 0,
    SX1276_LORA_EVENT_RX_DONE,
    SX1276_LORA_EVENT_TX_DONE,
    SX1276_LORA_EVENT_RX_ERROR,
} sx1276_lora_event_type_t;

typedef struct {
    spi_host_device_t spi_host;
    int pin_sck;
    int pin_miso;
    int pin_mosi;
    int pin_cs;
    int pin_rst;
    int pin_dio0;
    uint32_t frequency_hz;
    uint32_t bandwidth_hz;
    uint8_t spreading_factor;
    int8_t tx_power_dbm;
    uint8_t coding_rate;  /* 1=4/5  2=4/6  3=4/7  4=4/8 (default: 1) */
    uint8_t preamble_len; /* simbolos; 0 => usa default 8 */
} sx1276_lora_config_t;

typedef struct {
    sx1276_lora_event_type_t type;
    uint16_t irq_flags;
    size_t payload_len;
    int16_t rssi_dbm;
    int8_t snr_db;
    uint8_t payload[SX1276_LORA_MAX_PAYLOAD_LEN];
} sx1276_lora_event_t;

esp_err_t sx1276_lora_init(const sx1276_lora_config_t *config);
esp_err_t sx1276_lora_deinit(void);
esp_err_t sx1276_lora_start_rx_continuous(void);
esp_err_t sx1276_lora_transmit(const uint8_t *payload, size_t payload_len);
esp_err_t sx1276_lora_receive_event(sx1276_lora_event_t *event, TickType_t timeout);
void sx1276_lora_flush_events(void);
void sx1276_lora_debug_dump(void);
