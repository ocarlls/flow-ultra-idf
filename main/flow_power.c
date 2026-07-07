#include "sdkconfig.h"
#include "flow_power.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FLOW_PWR";

static bool s_wifi_up = false;

void flow_power_mark_wifi_up(void)
{
    s_wifi_up = true;
}

void flow_power_stay_awake_hatch(void)
{
#if CONFIG_FLOW_STAY_AWAKE_GPIO >= 0
    const gpio_num_t pin = (gpio_num_t)CONFIG_FLOW_STAY_AWAKE_GPIO;
    gpio_config_t ic = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    (void)gpio_config(&ic);
    vTaskDelay(pdMS_TO_TICKS(10)); /* pull-up assentar */
    if (gpio_get_level(pin) == 0) {
        ESP_LOGW(TAG, "ESCAPE HATCH: GPIO%d baixo no boot — ficando acordado p/ regravacao",
                 (int)pin);
        while (1) {
            ESP_LOGW(TAG, "Acordado p/ regravacao (idf.py flash). Reset p/ voltar ao normal.");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
#endif
}

void flow_power_wifi_teardown(void)
{
    if (!s_wifi_up) {
        return;
    }
    s_wifi_up = false;
    (void)esp_now_unregister_recv_cb();
    (void)esp_now_deinit();
    (void)esp_wifi_stop();
    (void)esp_wifi_deinit();
}

void flow_power_deep_sleep_ms(uint32_t sleep_ms)
{
    flow_power_wifi_teardown();
    if (sleep_ms < 1000U) {
        sleep_ms = 1000U;
    }
    ESP_LOGI(TAG, "Deep sleep por %lu ms", (unsigned long)sleep_ms);
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
    esp_deep_sleep_start();
    while (1) { /* nunca chega aqui */ }
}
