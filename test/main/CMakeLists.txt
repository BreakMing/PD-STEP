idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bsp settings cartpos poleang tmc2209 cartproto step_engine cartfsm telemetry
                    PRIV_REQUIRES unity)
