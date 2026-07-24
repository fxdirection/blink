idf_component_register(
    SRCS
        "blink_example_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_gpio
        config
        motor
        chassis
        pid
        debug_comm
        wifi_debug
        tb6612
)