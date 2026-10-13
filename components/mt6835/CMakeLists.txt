idf_component_register(
    SRCS "mt6835.c" "mt6835_urad.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp esp_driver_spi freertos
)
