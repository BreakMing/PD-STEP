idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp ch224q cartproto settings magnetic_encoder cartpos poleang motor cartaxis telemetry cartfsm
)
