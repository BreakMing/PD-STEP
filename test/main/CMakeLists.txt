idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bsp settings cartpos poleang motor cartproto cartfsm telemetry
                    PRIV_REQUIRES unity)
