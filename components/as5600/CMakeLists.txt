# as5600 — 驱动核心组件：I2C 读层 + 多圈 tracker（经 magnetic_encoder 组合）
idf_component_register(SRCS "as5600_tracker.c" "as5600_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c)
