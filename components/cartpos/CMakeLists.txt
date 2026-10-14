# cartpos — 小车位置域组件：契约头 + 实现源文件（Kconfig choice 选择）
# 依赖无条件声明（各实现所需之并集）；仅源文件随 choice 条件编入。
set(cartpos_srcs "")

if(CONFIG_CARTPOS_IMPL_AS5600)
    list(APPEND cartpos_srcs "cartpos_as5600.c")
endif()

idf_component_register(SRCS ${cartpos_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c
                    PRIV_REQUIRES as5600)
