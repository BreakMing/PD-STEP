idf_component_register(
    SRCS "mt6835.c" "mt6835_urad.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp esp_driver_spi freertos
)

if(CONFIG_MT6835_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "mt6835_self_test.c")
endif()
