idf_component_register(
    SRCS "telemetry.c" "telemetry_format.c"
    INCLUDE_DIRS "include"
    REQUIRES as5600 mt6835 cartproto cartfsm freertos
)

if(CONFIG_TELEMETRY_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "telemetry_self_test.c")
endif()
