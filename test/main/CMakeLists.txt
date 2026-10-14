idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bsp settings magnetic_encoder cartpos poleang motor cartaxis cartproto cartfsm telemetry
                    PRIV_REQUIRES unity)
