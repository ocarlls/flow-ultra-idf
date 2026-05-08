set(FLOW_MESH_SRCS "")
if(CONFIG_FLOW_MESH_MODE)
    list(APPEND FLOW_MESH_SRCS
        "mesh_dedup.c"
        "meter_node.c"
        "subroot_node.c"
        "root_node.c"
    )
endif()

idf_component_register(SRCS "sx1276_lora.c" "lora_test_module.c" "as6031_remote.c" "espnow_test_module.c" "main.c" ${FLOW_MESH_SRCS}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lcd esp_driver_i2c esp_timer esp_wifi esp_event mqtt nvs_flash esp_netif)
