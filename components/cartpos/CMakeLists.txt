# cartpos — 小车位置解释组件：encoder counts → x_um（比例/原点归本层）
idf_component_register(SRCS "cartpos.c"
                    INCLUDE_DIRS "include"
                    REQUIRES magnetic_encoder settings)
