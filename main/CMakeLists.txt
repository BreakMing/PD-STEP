idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp protocol gcode_parser settings step_engine tmc2209
)

if(CONFIG_APP_JOINT_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "app_joint_test.c")
endif()
