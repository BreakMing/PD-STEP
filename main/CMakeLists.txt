idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp protocol gcode_parser settings step_engine tmc2209
)
