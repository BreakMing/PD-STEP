# cartaxis — 小车直线轴解释组件：mm↔steps 换算 + $0 单一写者 + X_INV 取反
idf_component_register(SRCS "cartaxis_conv.c" "cartaxis.c"
                    INCLUDE_DIRS "include"
                    REQUIRES motor settings cartparams)
