idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp cartproto settings step_engine tmc2209 as5600 mt6835 telemetry
)
