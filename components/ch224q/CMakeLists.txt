# ch224q — CH224Q PD 诱骗 I2C 调档组件（驱动核心，经组装点组合）
idf_component_register(SRCS "ch224q.c" "ch224q_status.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c)
