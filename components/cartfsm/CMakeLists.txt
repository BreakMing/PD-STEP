idf_component_register(
    SRCS "cartfsm.c" "cartfsm_core.c" "params.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp settings tmc2209 step_engine as5600 mt6835 cartproto freertos
)
