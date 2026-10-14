idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp cartproto settings step_engine tmc2209 cartpos mt6835 telemetry cartfsm
)
