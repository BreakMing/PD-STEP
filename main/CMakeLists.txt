idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp cartproto settings cartpos poleang motor telemetry cartfsm
)
