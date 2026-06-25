#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "espnow_test_module.h"
#include "lora_test_module.h"
#include "meter_node.h"
#include "root_node.h"
#include "subroot_node.h"

static const char *TAG = "FLOW_MAIN";

// Ponto de entrada. Inicializa a NVS e despacha para o modo selecionado via
// Kconfig: test modes (LoRa / ESP-NOW) ou a malha Flow com seu role.
//
// Toda a implementacao legada de hidrometro standalone (sensor/SPI,
// gateway/MQTT, deep sleep diario) foi removida. O role "meter" agora roda
// como relay ESP-NOW em light sleep (ver meter_node.c -> meter_tx_test.c).
void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

#if CONFIG_LORA_TEST_MODE
    lora_test_run();
#elif CONFIG_ESPNOW_TEST_MODE
    espnow_test_run();
#elif CONFIG_FLOW_MESH_MODE
#if CONFIG_FLOW_ROLE_METER
    meter_node_run();
#elif CONFIG_FLOW_ROLE_SUBROOT
    subroot_node_run();
#elif CONFIG_FLOW_ROLE_ROOT
    root_node_run();
#else
    ESP_LOGE(TAG, "FLOW_MESH_MODE habilitado sem role selecionado");
#endif
#else
    ESP_LOGE(TAG, "Nenhum modo selecionado (FLOW_MESH_MODE / LORA_TEST_MODE / ESPNOW_TEST_MODE)");
#endif
}
