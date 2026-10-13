idf_component_register(
    SRCS "tmc2209_hw.c" "tmc2209.c"
    INCLUDE_DIRS "include"
    REQUIRES driver bsp settings
)
