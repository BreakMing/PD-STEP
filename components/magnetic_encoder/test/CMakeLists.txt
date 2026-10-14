# magnetic_encoder 测试组件（test/ 子目录约定：仅测试工程的 TEST_COMPONENTS 构建发现）
idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "."
                    REQUIRES magnetic_encoder cartpos poleang bsp settings mt6835
                    PRIV_REQUIRES unity freertos log)
