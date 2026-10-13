idf_component_register(
    SRCS "telemetry.c" "telemetry_format.c"
    INCLUDE_DIRS "include"
    REQUIRES as5600 mt6835 cartproto cartparams freertos
)
