idf_component_register(
    SRCS "tmc2209.c" "tmc2209_self_test.c" "tmc2209_bringup.c"
    INCLUDE_DIRS "include"
    REQUIRES driver bsp settings
)
