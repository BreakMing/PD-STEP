idf_component_register(SRCS "sysctl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES step_engine planner settings freertos)

if(CONFIG_SYSCTL_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "sysctl_self_test.c")
endif()
