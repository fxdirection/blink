idf_component_register(
    SRCS "wifi_debug.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        lwip
        debug_comm
        chassis
        motor
        pid
)