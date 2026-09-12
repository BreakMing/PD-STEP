idf_component_register(
    SRCS "protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp esp_driver_uart freertos
)
