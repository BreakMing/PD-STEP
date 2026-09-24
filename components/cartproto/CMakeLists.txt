idf_component_register(
    SRCS "cartproto.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp esp_driver_uart freertos
)
