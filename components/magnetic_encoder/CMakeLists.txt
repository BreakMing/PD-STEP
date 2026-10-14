# magnetic_encoder — 磁编码器类别组件：契约 + 实现源文件
# 依赖无条件字面量声明（各实现所需之并集）；仅源文件随 Kconfig 宏编入。
set(me_srcs "")

if(CONFIG_MAGNETIC_ENCODER_AS5600)
    list(APPEND me_srcs "magnetic_encoder_as5600.c")
endif()

if(CONFIG_MAGNETIC_ENCODER_MT6835)
    list(APPEND me_srcs "magnetic_encoder_mt6835.c")
endif()

idf_component_register(SRCS ${me_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES freertos esp_driver_i2c esp_driver_spi
                    PRIV_REQUIRES as5600 mt6835)
