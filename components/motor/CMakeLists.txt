# motor — 电机运动域组件：契约头 + 实现源文件（Kconfig choice 选择）
# 依赖无条件声明（各实现所需之并集）；仅源文件随 choice 条件编入。
set(motor_srcs "motor_conv.c")

if(CONFIG_MOTOR_IMPL_TMC2209)
    list(APPEND motor_srcs "motor_tmc2209.c")
endif()

idf_component_register(SRCS ${motor_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES tmc2209
                    PRIV_REQUIRES step_engine settings)
