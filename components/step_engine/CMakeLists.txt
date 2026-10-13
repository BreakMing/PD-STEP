idf_component_register(SRCS "step_engine.c" "vel_ramp.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_gptimer esp_rom hal freertos)
