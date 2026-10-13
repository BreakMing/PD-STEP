idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bsp settings as5600 mt6835 tmc2209 cartproto step_engine cartfsm telemetry
                    PRIV_REQUIRES unity)
