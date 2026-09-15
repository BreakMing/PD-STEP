idf_component_register(
    SRCS "gcode_parser.c"
    INCLUDE_DIRS "include"
    REQUIRES
)

if(CONFIG_GCODE_PARSER_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "gcode_parser_self_test.c")
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
endif()
