# as5600 — 驱动核心组件：I2C 读层（经 magnetic_encoder 组合；
# 多圈累计已上提为类别组件 magnetic_encoder_tracker）
idf_component_register(SRCS "as5600_hw.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c)
