idf_component_register(
    SRCS "cartfsm.c" "cartfsm_core.c" "params.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp settings tmc2209 step_engine as5600 mt6835 cartproto freertos
)

if(CONFIG_CARTFSM_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "cartfsm_self_test.c")
endif()

if(CONFIG_CARTFSM_FLOW_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "cartfsm_flow_test.c")
endif()
