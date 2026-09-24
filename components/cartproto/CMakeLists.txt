idf_component_register(
    SRCS "cartproto.c" "cartproto_parse.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp esp_driver_uart freertos
)

if(CONFIG_CARTPROTO_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "cartproto_self_test.c")
endif()
