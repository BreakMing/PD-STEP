idf_component_register(SRCS "planner.c"
                    INCLUDE_DIRS "include"
                    REQUIRES step_engine settings freertos)

if(CONFIG_PLANNER_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "planner_self_test.c")
endif()
