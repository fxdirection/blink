idf_component_register(
    SRCS "debug_comm.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        driver
        esp_driver_uart
        chassis
        motor
        pid
)