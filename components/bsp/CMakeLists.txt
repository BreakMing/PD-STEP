idf_component_register(
    SRCS "bsp.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_i2c esp_driver_gpio esp_driver_spi esp_common
)
