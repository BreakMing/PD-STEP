idf_component_register(SRCS "step_engine.c" "vel_ramp.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_gptimer esp_rom hal freertos
                    PRIV_REQUIRES bsp)

if(CONFIG_STEP_ENGINE_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "step_engine_self_test.c")
endif()
