# cartpole 独立测试工程（官方 examples/system/unit_test 模式）
#
# TEST_COMPONENTS 使所列组件的 test/ 子目录作为测试组件参与构建；
# 正式固件（仓库根工程）完全不包含测试代码。
# 可用 `idf.py -T <component>` 覆盖只测单个组件。
cmake_minimum_required(VERSION 3.16)

# 复用主工程组件目录与 sdkconfig 默认值（手动设置 SDKCONFIG_DEFAULTS
# 后 IDF 不再自动附加，需按序显式列出：先主工程默认，再本工程附加）
list(APPEND SDKCONFIG_DEFAULTS
     "${CMAKE_CURRENT_SOURCE_DIR}/../sdkconfig.defaults"
     "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults")

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../components")

option(CURRENT_MEASUREMENT "构建 UART0 空载电流测量固件" OFF)
if(CURRENT_MEASUREMENT)
    # 此模式只编入测量入口；不加载 Unity 用例及正式模式机。
    set(TEST_COMPONENTS "")
else()
    set(TEST_COMPONENTS "cartfsm ch224q cartproto magnetic_encoder mt6835 settings step_engine telemetry tmc2209"
        CACHE STRING "参与测试的组件（其 test/ 子目录被编入）")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
if(CURRENT_MEASUREMENT)
    project(cartpole-current-test)
else()
    project(cartpole-test)
endif()
