idf_component_register(
    SRCS "settings.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)

if(CONFIG_SETTINGS_SELF_TEST)
    target_sources(${COMPONENT_LIB} PRIVATE "settings_self_test.c")
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
endif()
