# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

# 调试构建（保留全部 ESP_LOG 输出）。sdkconfig.defaults 显式列出，
# ESP-IDF 在 SDKCONFIG_DEFAULTS 手动设置时不再自动包含它。
list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cartpole-fw)
