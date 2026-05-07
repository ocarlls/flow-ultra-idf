idf_component_register(SRCS "sx1276_lora.c" "lora_test_module.c" "as6031_remote.c" "espnow_test_module.c" "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lcd esp_driver_i2c esp_timer esp_wifi esp_event mqtt nvs_flash esp_netif)
