idf_component_register(SRCS "as5600.c" "as5600_tracker.c" "as5600_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES settings freertos esp_driver_i2c)
