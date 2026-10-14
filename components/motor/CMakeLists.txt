# motor — 步进电机类别组件：契约 + 实现源文件（Kconfig choice 选择）
set(motor_srcs "")

if(CONFIG_MOTOR_IMPL_TMC2209)
    list(APPEND motor_srcs "motor_tmc2209.c")
endif()

idf_component_register(SRCS ${motor_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES tmc2209
                    PRIV_REQUIRES step_engine)
