idf_component_register(SRCS "monitor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES protocol planner step_engine as5600 tmc2209 settings sysctl freertos)

if(CONFIG_MONITOR_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "monitor_self_test.c")
endif()
