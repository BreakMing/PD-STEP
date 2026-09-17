idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bsp protocol gcode_parser settings step_engine tmc2209 planner as5600 monitor sysctl
)

if(CONFIG_APP_JOINT_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "app_joint_test.c")
endif()

if(CONFIG_APP_M3_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "app_m3_test.c")
endif()
