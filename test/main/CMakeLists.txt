set(test_main_sources "test_app_main.c")
if(CURRENT_MEASUREMENT)
    set(test_main_sources "current_measurement_main.c" "current_sequence.c" "current_pd.c")
endif()

# ESP-IDF 提前扫描依赖时不保留自定义构建变量，依赖必须统一声明。
idf_component_register(SRCS ${test_main_sources}
                    INCLUDE_DIRS "."
                    REQUIRES bsp ch224q settings magnetic_encoder cartpos poleang motor cartaxis
                             cartproto cartfsm telemetry tmc2209 step_engine esp_timer
                             esp_driver_gpio esp_driver_uart freertos
                    PRIV_REQUIRES unity)
