idf_component_register(SRCS "as5600.c" "as5600_tracker.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bsp settings freertos esp_driver_i2c)

if(CONFIG_AS5600_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "as5600_self_test.c")
endif()
