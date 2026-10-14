# poleang — 摆角域组件：契约头 + 实现源文件（Kconfig choice 选择）
# 依赖无条件声明（各实现所需之并集）；仅源文件随 choice 条件编入。
set(poleang_srcs "")

if(CONFIG_POLEANG_IMPL_MT6835)
    list(APPEND poleang_srcs "poleang_mt6835.c")
endif()

idf_component_register(SRCS ${poleang_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi
                    PRIV_REQUIRES mt6835 cartparams)
