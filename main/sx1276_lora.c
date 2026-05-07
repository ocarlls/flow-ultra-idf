#include "sx1276_lora.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

enum {
    SX1276_REG_FIFO = 0x00,
    SX1276_REG_OP_MODE = 0x01,
    SX1276_REG_FRF_MSB = 0x06,
    SX1276_REG_FRF_MID = 0x07,
    SX1276_REG_FRF_LSB = 0x08,
    SX1276_REG_PA_CONFIG = 0x09,
    SX1276_REG_OCP = 0x0B,
    SX1276_REG_LNA = 0x0C,
    SX1276_REG_FIFO_ADDR_PTR = 0x0D,
    SX1276_REG_FIFO_TX_BASE_ADDR = 0x0E,
    SX1276_REG_FIFO_RX_BASE_ADDR = 0x0F,
    SX1276_REG_FIFO_RX_CURRENT_ADDR = 0x10,
    SX1276_REG_IRQ_FLAGS_MASK = 0x11,
    SX1276_REG_IRQ_FLAGS = 0x12,
    SX1276_REG_RX_NB_BYTES = 0x13,
    SX1276_REG_PKT_SNR_VALUE = 0x19,
    SX1276_REG_PKT_RSSI_VALUE = 0x1A,
    SX1276_REG_MODEM_CONFIG1 = 0x1D,
    SX1276_REG_MODEM_CONFIG2 = 0x1E,
    SX1276_REG_SYMB_TIMEOUT_LSB = 0x1F,
    SX1276_REG_PREAMBLE_MSB = 0x20,
    SX1276_REG_PREAMBLE_LSB = 0x21,
    SX1276_REG_PAYLOAD_LENGTH = 0x22,
    SX1276_REG_PAYLOAD_MAX_LENGTH = 0x23,
    SX1276_REG_HOP_PERIOD = 0x24,
    SX1276_REG_MODEM_CONFIG3 = 0x26,
    SX1276_REG_DETECT_OPTIMIZE = 0x31,
    SX1276_REG_DETECTION_THRESHOLD = 0x37,
    SX1276_REG_SYNC_WORD = 0x39,
    SX1276_REG_DIO_MAPPING1 = 0x40,
    SX1276_REG_VERSION = 0x42,
    SX1276_REG_PA_DAC = 0x4D,
};

enum {
    SX1276_OPMODE_LONG_RANGE = 0x80,
    SX1276_OPMODE_LOW_FREQUENCY = 0x08,
    SX1276_MODE_SLEEP = 0x00,
    SX1276_MODE_STDBY = 0x01,
    SX1276_MODE_TX = 0x03,
    SX1276_MODE_RX_CONTINUOUS = 0x05,
};

enum {
    SX1276_IRQ_RX_TIMEOUT = 0x80,
    SX1276_IRQ_RX_DONE = 0x40,
    SX1276_IRQ_PAYLOAD_CRC_ERROR = 0x20,
    SX1276_IRQ_TX_DONE = 0x08,
};

enum {
    SX1276_DIO_MAPPING1_RX_DONE = 0x00,
    SX1276_DIO_MAPPING1_TX_DONE = 0x40,
};

#define SX1276_EXPECTED_VERSION 0x12
#define SX1276_SPI_CLOCK_HZ 5000000
#define SX1276_IRQ_QUEUE_LEN 8
#define SX1276_EVENT_QUEUE_LEN 8
#define SX1276_TASK_STACK_SIZE 4096
#define SX1276_TASK_PRIORITY 10

typedef struct {
    bool initialized;
    spi_host_device_t spi_host;
    spi_device_handle_t spi_handle;
    SemaphoreHandle_t spi_mutex;
    QueueHandle_t irq_queue;
    QueueHandle_t event_queue;
    TaskHandle_t irq_task;
    int pin_rst;
    int pin_dio0;
    bool isr_added;
    bool auto_rx_after_tx;
    uint32_t frequency_hz;
    uint32_t bandwidth_hz;
    uint8_t spreading_factor;
    int8_t tx_power_dbm;
    uint8_t coding_rate;  /* 1=4/5  2=4/6  3=4/7  4=4/8 */
    uint8_t preamble_len; /* simbolos */
} sx1276_lora_context_t;

static const char *TAG = "SX1276";
static sx1276_lora_context_t s_ctx = {0};

static bool sx1276_take_lock(TickType_t timeout)
{
    return (s_ctx.spi_mutex != NULL) && (xSemaphoreTake(s_ctx.spi_mutex, timeout) == pdTRUE);
}

static void sx1276_give_lock(void)
{
    if (s_ctx.spi_mutex != NULL) {
        xSemaphoreGive(s_ctx.spi_mutex);
    }
}

static esp_err_t sx1276_transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    spi_transaction_t transaction = {
        .length = len * 8U,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return spi_device_polling_transmit(s_ctx.spi_handle, &transaction);
}

static esp_err_t sx1276_write_reg_locked(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {(uint8_t)(reg | 0x80U), value};
    return sx1276_transfer(tx_buf, NULL, sizeof(tx_buf));
}

static esp_err_t sx1276_read_reg_locked(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2] = {(uint8_t)(reg & 0x7FU), 0x00U};
    uint8_t rx_buf[2] = {0};
    esp_err_t err = sx1276_transfer(tx_buf, rx_buf, sizeof(tx_buf));
    if (err == ESP_OK) {
        *value = rx_buf[1];
    }
    return err;
}

static esp_err_t sx1276_write_burst_locked(uint8_t reg, const uint8_t *data, size_t len)
{
    if (len > SX1276_LORA_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t tx_buf[SX1276_LORA_MAX_PAYLOAD_LEN + 1] = {0};
    tx_buf[0] = (uint8_t)(reg | 0x80U);
    memcpy(&tx_buf[1], data, len);
    return sx1276_transfer(tx_buf, NULL, len + 1U);
}

static esp_err_t sx1276_read_burst_locked(uint8_t reg, uint8_t *data, size_t len)
{
    if (len > SX1276_LORA_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t tx_buf[SX1276_LORA_MAX_PAYLOAD_LEN + 1] = {0};
    uint8_t rx_buf[SX1276_LORA_MAX_PAYLOAD_LEN + 1] = {0};
    tx_buf[0] = (uint8_t)(reg & 0x7FU);

    esp_err_t err = sx1276_transfer(tx_buf, rx_buf, len + 1U);
    if (err == ESP_OK) {
        memcpy(data, &rx_buf[1], len);
    }
    return err;
}

static uint8_t sx1276_mode_base(void)
{
    uint8_t mode = SX1276_OPMODE_LONG_RANGE;
    if (s_ctx.frequency_hz < 525000000UL) {
        mode |= SX1276_OPMODE_LOW_FREQUENCY;
    }
    return mode;
}

static esp_err_t sx1276_write_mode_locked(uint8_t mode)
{
    return sx1276_write_reg_locked(SX1276_REG_OP_MODE, (uint8_t)(sx1276_mode_base() | mode));
}

static void sx1276_push_event(const sx1276_lora_event_t *event)
{
    if (s_ctx.event_queue == NULL || event == NULL) {
        return;
    }

    if (xQueueSend(s_ctx.event_queue, event, 0) != pdTRUE) {
        sx1276_lora_event_t dropped = {0};
        (void)xQueueReceive(s_ctx.event_queue, &dropped, 0);
        (void)xQueueSend(s_ctx.event_queue, event, 0);
    }
}

static uint32_t sx1276_closest_bandwidth_hz(uint32_t requested_hz, uint8_t *reg_value)
{
    static const struct {
        uint32_t bandwidth_hz;
        uint8_t reg_value;
    } bandwidth_map[] = {
        {7800U, 0x00U},
        {10400U, 0x01U},
        {15600U, 0x02U},
        {20800U, 0x03U},
        {31250U, 0x04U},
        {41700U, 0x05U},
        {62500U, 0x06U},
        {125000U, 0x07U},
        {250000U, 0x08U},
        {500000U, 0x09U},
    };

    size_t best_idx = 0;
    uint32_t best_diff = UINT32_MAX;

    for (size_t i = 0; i < sizeof(bandwidth_map) / sizeof(bandwidth_map[0]); ++i) {
        uint32_t candidate = bandwidth_map[i].bandwidth_hz;
        uint32_t diff = (candidate > requested_hz) ? (candidate - requested_hz) : (requested_hz - candidate);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = i;
        }
    }

    *reg_value = bandwidth_map[best_idx].reg_value;
    return bandwidth_map[best_idx].bandwidth_hz;
}

static esp_err_t sx1276_apply_modem_config_locked(void)
{
    uint8_t bandwidth_reg = 0;
    uint32_t actual_bandwidth_hz = sx1276_closest_bandwidth_hz(s_ctx.bandwidth_hz, &bandwidth_reg);
    uint16_t symbol_timeout = 0x64U;
    bool low_data_rate_optimize = (actual_bandwidth_hz <= 125000UL) && (s_ctx.spreading_factor >= 11U);
    /* CodingRate bits [3:1]: 1=4/5, 2=4/6, 3=4/7, 4=4/8 */
    uint8_t modem_config1 = (uint8_t)((bandwidth_reg << 4U) | (s_ctx.coding_rate << 1U));
    uint8_t modem_config2 = (uint8_t)((s_ctx.spreading_factor << 4U) | (1U << 2U) |
                                      ((symbol_timeout >> 8U) & 0x03U));
    /* AGC desabilitado: LNA controlado manualmente em G1+boost (0x23) */
    uint8_t modem_config3 = (uint8_t)(low_data_rate_optimize ? 0x08U : 0x00U);

    s_ctx.bandwidth_hz = actual_bandwidth_hz;

    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_MODEM_CONFIG1, modem_config1), TAG,
                        "Falha ao configurar MODEM_CONFIG1");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_MODEM_CONFIG2, modem_config2), TAG,
                        "Falha ao configurar MODEM_CONFIG2");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_SYMB_TIMEOUT_LSB, (uint8_t)(symbol_timeout & 0xFFU)), TAG,
                        "Falha ao configurar symbol timeout");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_MODEM_CONFIG3, modem_config3), TAG,
                        "Falha ao configurar MODEM_CONFIG3");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_DETECT_OPTIMIZE, 0x03U), TAG,
                        "Falha ao configurar detect optimize");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_DETECTION_THRESHOLD, 0x0AU), TAG,
                        "Falha ao configurar detection threshold");
    return ESP_OK;
}

static esp_err_t sx1276_prepare_rx_locked(void)
{
    uint8_t fifo_rx_base_addr = 0;

    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_DIO_MAPPING1, SX1276_DIO_MAPPING1_RX_DONE), TAG,
                        "Falha ao mapear DIO0 para RX_DONE");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_IRQ_FLAGS, 0xFFU), TAG,
                        "Falha ao limpar IRQs");
    ESP_RETURN_ON_ERROR(sx1276_read_reg_locked(SX1276_REG_FIFO_RX_BASE_ADDR, &fifo_rx_base_addr), TAG,
                        "Falha ao ler FIFO RX base addr");
    ESP_RETURN_ON_ERROR(sx1276_write_reg_locked(SX1276_REG_FIFO_ADDR_PTR, fifo_rx_base_addr), TAG,
                        "Falha ao resetar FIFO RX addr ptr");
    return ESP_OK;
}

static esp_err_t sx1276_start_rx_locked(void)
{
    ESP_RETURN_ON_ERROR(sx1276_prepare_rx_locked(), TAG, "Falha ao preparar RX");
    return sx1276_write_mode_locked(SX1276_MODE_RX_CONTINUOUS);
}

static int16_t sx1276_packet_rssi_dbm(uint8_t raw_rssi, int8_t raw_snr)
{
    int16_t offset = (s_ctx.frequency_hz < 779000000UL) ? 164 : 157;
    int16_t rssi_dbm = (int16_t)raw_rssi - offset;
    if (raw_snr < 0) {
        rssi_dbm += raw_snr / 4;
    }
    return rssi_dbm;
}

static void sx1276_process_irq(void)
{
    sx1276_lora_event_t event = {0};
    uint8_t irq_flags = 0;

    if (!sx1276_take_lock(pdMS_TO_TICKS(200))) {
        ESP_LOGW(TAG, "Timeout ao adquirir mutex SPI no handler de IRQ");
        return;
    }

    if (sx1276_read_reg_locked(SX1276_REG_IRQ_FLAGS, &irq_flags) != ESP_OK || irq_flags == 0U) {
        sx1276_give_lock();
        return;
    }

    event.irq_flags = irq_flags;
    (void)sx1276_write_reg_locked(SX1276_REG_IRQ_FLAGS, irq_flags);

    if ((irq_flags & SX1276_IRQ_TX_DONE) != 0U) {
        event.type = SX1276_LORA_EVENT_TX_DONE;
        if (s_ctx.auto_rx_after_tx) {
            s_ctx.auto_rx_after_tx = false;
            (void)sx1276_start_rx_locked();
        }
        sx1276_give_lock();
        sx1276_push_event(&event);
        return;
    }

    if ((irq_flags & SX1276_IRQ_RX_DONE) != 0U) {
        if ((irq_flags & SX1276_IRQ_PAYLOAD_CRC_ERROR) != 0U) {
            event.type = SX1276_LORA_EVENT_RX_ERROR;
            (void)sx1276_start_rx_locked();
            sx1276_give_lock();
            sx1276_push_event(&event);
            return;
        }

        uint8_t fifo_addr = 0;
        uint8_t payload_len = 0;
        uint8_t raw_snr = 0;
        uint8_t raw_rssi = 0;

        (void)sx1276_read_reg_locked(SX1276_REG_FIFO_RX_CURRENT_ADDR, &fifo_addr);
        (void)sx1276_read_reg_locked(SX1276_REG_RX_NB_BYTES, &payload_len);
        (void)sx1276_read_reg_locked(SX1276_REG_PKT_SNR_VALUE, &raw_snr);
        (void)sx1276_read_reg_locked(SX1276_REG_PKT_RSSI_VALUE, &raw_rssi);

        event.type = SX1276_LORA_EVENT_RX_DONE;
        event.payload_len = payload_len;
        event.snr_db = (int8_t)raw_snr / 4;
        event.rssi_dbm = sx1276_packet_rssi_dbm(raw_rssi, (int8_t)raw_snr);

        (void)sx1276_write_reg_locked(SX1276_REG_FIFO_ADDR_PTR, fifo_addr);
        if (payload_len > 0U) {
            (void)sx1276_read_burst_locked(SX1276_REG_FIFO, event.payload, payload_len);
        }

        (void)sx1276_start_rx_locked();
        sx1276_give_lock();
        sx1276_push_event(&event);
        return;
    }

    if ((irq_flags & (SX1276_IRQ_RX_TIMEOUT | SX1276_IRQ_PAYLOAD_CRC_ERROR)) != 0U) {
        event.type = SX1276_LORA_EVENT_RX_ERROR;
        (void)sx1276_start_rx_locked();
        sx1276_give_lock();
        sx1276_push_event(&event);
        return;
    }

    sx1276_give_lock();
}

static void sx1276_irq_task(void *arg)
{
    uint32_t gpio_num = 0;
    (void)arg;

    while (xQueueReceive(s_ctx.irq_queue, &gpio_num, portMAX_DELAY) == pdTRUE) {
        (void)gpio_num;
        sx1276_process_irq();
    }

    vTaskDelete(NULL);
}

static void IRAM_ATTR sx1276_dio0_isr(void *arg)
{
    uint32_t gpio_num = (uint32_t)(uintptr_t)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_ctx.irq_queue != NULL) {
        xQueueSendFromISR(s_ctx.irq_queue, &gpio_num, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t sx1276_hw_reset(int pin_rst)
{
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << pin_rst,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), TAG, "Falha ao configurar pino RST");
    ESP_RETURN_ON_ERROR(gpio_set_level(pin_rst, 0), TAG, "Falha ao setar RST low");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level(pin_rst, 1), TAG, "Falha ao setar RST high");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t sx1276_lora_init(const sx1276_lora_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->spreading_factor < 7U || config->spreading_factor > 12U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.spi_host = config->spi_host;
    s_ctx.pin_rst = config->pin_rst;
    s_ctx.pin_dio0 = config->pin_dio0;
    s_ctx.frequency_hz = config->frequency_hz;
    s_ctx.bandwidth_hz = config->bandwidth_hz;
    s_ctx.spreading_factor = config->spreading_factor;
    s_ctx.tx_power_dbm = config->tx_power_dbm;
    s_ctx.coding_rate  = (config->coding_rate >= 1U && config->coding_rate <= 4U) ? config->coding_rate : 1U;
    s_ctx.preamble_len = (config->preamble_len > 0U) ? config->preamble_len : 8U;

    s_ctx.spi_mutex = xSemaphoreCreateMutex();
    if (s_ctx.spi_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_ctx.irq_queue = xQueueCreate(SX1276_IRQ_QUEUE_LEN, sizeof(uint32_t));
    s_ctx.event_queue = xQueueCreate(SX1276_EVENT_QUEUE_LEN, sizeof(sx1276_lora_event_t));
    if (s_ctx.irq_queue == NULL || s_ctx.event_queue == NULL) {
        (void)sx1276_lora_deinit();
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sck,
        .max_transfer_sz = SX1276_LORA_MAX_PAYLOAD_LEN + 1,
    };
    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }

    spi_device_interface_config_t device_cfg = {
        .clock_speed_hz = SX1276_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = config->pin_cs,
        .queue_size = 4,
    };
    err = spi_bus_add_device(config->spi_host, &device_cfg, &s_ctx.spi_handle);
    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }

    err = sx1276_hw_reset(config->pin_rst);
    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }

    gpio_config_t dio0_cfg = {
        .pin_bit_mask = 1ULL << config->pin_dio0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&dio0_cfg);
    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }

    if (xTaskCreate(sx1276_irq_task, "sx1276_irq", SX1276_TASK_STACK_SIZE, NULL,
                    SX1276_TASK_PRIORITY, &s_ctx.irq_task) != pdPASS) {
        (void)sx1276_lora_deinit();
        return ESP_ERR_NO_MEM;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        (void)sx1276_lora_deinit();
        return err;
    }

    err = gpio_isr_handler_add(config->pin_dio0, sx1276_dio0_isr, (void *)(uintptr_t)config->pin_dio0);
    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }
    s_ctx.isr_added = true;

    if (!sx1276_take_lock(pdMS_TO_TICKS(200))) {
        (void)sx1276_lora_deinit();
        return ESP_ERR_TIMEOUT;
    }

    uint8_t version = 0;
    err = sx1276_read_reg_locked(SX1276_REG_VERSION, &version);
    if (err == ESP_OK && version != SX1276_EXPECTED_VERSION) {
        err = ESP_ERR_INVALID_RESPONSE;
        ESP_LOGE(TAG, "SX1276 nao respondeu com versao esperada: 0x%02X", version);
    }
    if (err == ESP_OK) {
        err = sx1276_write_mode_locked(SX1276_MODE_SLEEP);
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(5));
        err = sx1276_write_mode_locked(SX1276_MODE_STDBY);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_FIFO_TX_BASE_ADDR, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_FIFO_RX_BASE_ADDR, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_LNA, 0x23U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_PREAMBLE_MSB, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_PREAMBLE_LSB, s_ctx.preamble_len);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_PAYLOAD_MAX_LENGTH, SX1276_LORA_MAX_PAYLOAD_LEN);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_HOP_PERIOD, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_DIO_MAPPING1, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_IRQ_FLAGS_MASK, 0x00U);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_SYNC_WORD, 0x12U);
    }
    if (err == ESP_OK) {
        uint64_t frf = (((uint64_t)s_ctx.frequency_hz) << 19U) / 32000000ULL;
        err = sx1276_write_reg_locked(SX1276_REG_FRF_MSB, (uint8_t)(frf >> 16U));
        if (err == ESP_OK) {
            err = sx1276_write_reg_locked(SX1276_REG_FRF_MID, (uint8_t)(frf >> 8U));
        }
        if (err == ESP_OK) {
            err = sx1276_write_reg_locked(SX1276_REG_FRF_LSB, (uint8_t)frf);
        }
    }
    if (err == ESP_OK) {
        err = sx1276_apply_modem_config_locked();
    }
    if (err == ESP_OK) {
        int8_t power_dbm = s_ctx.tx_power_dbm;
        if (power_dbm < 2) {
            power_dbm = 2;
        } else if (power_dbm > 20) {
            power_dbm = 20;
        }

        uint8_t pa_dac = (power_dbm > 17) ? 0x87U : 0x84U;
        int8_t output_power = (power_dbm > 17) ? (power_dbm - 5) : (power_dbm - 2);
        if (output_power < 0) {
            output_power = 0;
        }
        if (output_power > 15) {
            output_power = 15;
        }

        err = sx1276_write_reg_locked(SX1276_REG_PA_DAC, pa_dac);
        if (err == ESP_OK) {
            err = sx1276_write_reg_locked(SX1276_REG_PA_CONFIG, (uint8_t)(0x80U | 0x70U | (uint8_t)output_power));
        }
        if (err == ESP_OK) {
            err = sx1276_write_reg_locked(SX1276_REG_OCP, 0x2BU);
        }
    }
    if (err == ESP_OK) {
        err = sx1276_prepare_rx_locked();
    }
    sx1276_give_lock();

    if (err != ESP_OK) {
        (void)sx1276_lora_deinit();
        return err;
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG,
             "SX1276 pronto freq=%luHz bw=%luHz sf=%u tx=%ddBm",
             (unsigned long)s_ctx.frequency_hz,
             (unsigned long)s_ctx.bandwidth_hz,
             (unsigned)s_ctx.spreading_factor,
             (int)s_ctx.tx_power_dbm);
    return ESP_OK;
}

esp_err_t sx1276_lora_deinit(void)
{
    if (s_ctx.isr_added) {
        (void)gpio_isr_handler_remove(s_ctx.pin_dio0);
        s_ctx.isr_added = false;
    }

    if (s_ctx.spi_handle != NULL && sx1276_take_lock(pdMS_TO_TICKS(50))) {
        (void)sx1276_write_mode_locked(SX1276_MODE_SLEEP);
        sx1276_give_lock();
    }

    if (s_ctx.irq_task != NULL) {
        vTaskDelete(s_ctx.irq_task);
        s_ctx.irq_task = NULL;
    }
    if (s_ctx.irq_queue != NULL) {
        vQueueDelete(s_ctx.irq_queue);
        s_ctx.irq_queue = NULL;
    }
    if (s_ctx.event_queue != NULL) {
        vQueueDelete(s_ctx.event_queue);
        s_ctx.event_queue = NULL;
    }
    if (s_ctx.spi_mutex != NULL) {
        vSemaphoreDelete(s_ctx.spi_mutex);
        s_ctx.spi_mutex = NULL;
    }
    if (s_ctx.spi_handle != NULL) {
        (void)spi_bus_remove_device(s_ctx.spi_handle);
        s_ctx.spi_handle = NULL;
    }
    if (s_ctx.spi_host != SPI_HOST_MAX) {
        (void)spi_bus_free(s_ctx.spi_host);
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    return ESP_OK;
}

esp_err_t sx1276_lora_start_rx_continuous(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sx1276_take_lock(pdMS_TO_TICKS(200))) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = sx1276_start_rx_locked();
    sx1276_give_lock();
    return err;
}

esp_err_t sx1276_lora_transmit(const uint8_t *payload, size_t payload_len)
{
    if (!s_ctx.initialized || payload == NULL || payload_len == 0U || payload_len > SX1276_LORA_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sx1276_take_lock(pdMS_TO_TICKS(200))) {
        return ESP_ERR_TIMEOUT;
    }

    s_ctx.auto_rx_after_tx = false;

    esp_err_t err = sx1276_write_mode_locked(SX1276_MODE_STDBY);
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_DIO_MAPPING1, SX1276_DIO_MAPPING1_TX_DONE);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_IRQ_FLAGS, 0xFFU);
    }
    if (err == ESP_OK) {
        uint8_t fifo_tx_base_addr = 0;
        err = sx1276_read_reg_locked(SX1276_REG_FIFO_TX_BASE_ADDR, &fifo_tx_base_addr);
        if (err == ESP_OK) {
            err = sx1276_write_reg_locked(SX1276_REG_FIFO_ADDR_PTR, fifo_tx_base_addr);
        }
    }
    if (err == ESP_OK) {
        err = sx1276_write_burst_locked(SX1276_REG_FIFO, payload, payload_len);
    }
    if (err == ESP_OK) {
        err = sx1276_write_reg_locked(SX1276_REG_PAYLOAD_LENGTH, (uint8_t)payload_len);
    }
    if (err == ESP_OK) {
        s_ctx.auto_rx_after_tx = true;
        err = sx1276_write_mode_locked(SX1276_MODE_TX);
        if (err != ESP_OK) {
            s_ctx.auto_rx_after_tx = false;
        }
    }
    sx1276_give_lock();
    return err;
}

esp_err_t sx1276_lora_receive_event(sx1276_lora_event_t *event, TickType_t timeout)
{
    if (event == NULL || s_ctx.event_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueReceive(s_ctx.event_queue, event, timeout) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

void sx1276_lora_flush_events(void)
{
    if (s_ctx.event_queue == NULL) {
        return;
    }

    sx1276_lora_event_t event = {0};
    while (xQueueReceive(s_ctx.event_queue, &event, 0) == pdTRUE) {
    }
}

void sx1276_lora_debug_dump(void)
{
    if (!s_ctx.initialized) {
        return;
    }
    if (!sx1276_take_lock(pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "Nao foi possivel obter mutex para dump de debug");
        return;
    }

    uint8_t version = 0;
    uint8_t op_mode = 0;
    uint8_t modem1 = 0;
    uint8_t modem2 = 0;
    uint8_t modem3 = 0;

    (void)sx1276_read_reg_locked(SX1276_REG_VERSION, &version);
    (void)sx1276_read_reg_locked(SX1276_REG_OP_MODE, &op_mode);
    (void)sx1276_read_reg_locked(SX1276_REG_MODEM_CONFIG1, &modem1);
    (void)sx1276_read_reg_locked(SX1276_REG_MODEM_CONFIG2, &modem2);
    (void)sx1276_read_reg_locked(SX1276_REG_MODEM_CONFIG3, &modem3);
    sx1276_give_lock();

    ESP_LOGI(TAG,
             "Dump version=0x%02X opmode=0x%02X modem1=0x%02X modem2=0x%02X modem3=0x%02X",
             version, op_mode, modem1, modem2, modem3);
}
