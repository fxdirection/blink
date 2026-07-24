idf_component_register(
    SRCS "tb6612.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_pcnt
        esp_rom
        esp_timer
        pid
)
