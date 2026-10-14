# poleang — 摆角解释组件：encoder 角量 → 已标定 θ（零位/符号归本层）
idf_component_register(SRCS "poleang.c"
                    INCLUDE_DIRS "include"
                    REQUIRES magnetic_encoder cartparams)
