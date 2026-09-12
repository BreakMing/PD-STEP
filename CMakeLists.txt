# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

# ---- Build Type: Debug / Release ----
# Default: Debug  (full ESP_LOG output compiled in)
# Release:  compile-time strips all ESP_LOG macros (via sdkconfig.release)
#
# Usage:
#   Debug build:   idf.py build
#   Release build: idf.py -DCMAKE_BUILD_TYPE=Release build
#
# How Release log stripping works:
#   sdkconfig.release sets CONFIG_LOG_MAXIMUM_LEVEL=0 (ESP_LOG_NONE).
#   LOG_LOCAL_LEVEL defaults to CONFIG_LOG_MAXIMUM_LEVEL (see esp_log_level.h),
#   so all ESP_LOGx macros evaluate to dead code and are compiled out.
#   This avoids -Werror redefinition conflicts with components that define
#   LOG_LOCAL_LEVEL themselves (e.g. vfs, nvs_flash).
#
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# SDKCONFIG_DEFAULTS: explicitly list sdkconfig.defaults so that adding
# release overrides still preserves the base config.  (When SDKCONFIG_DEFAULTS
# is set manually, ESP-IDF does not auto-include sdkconfig.defaults.)
list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.defaults")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Release: overlay sdkconfig.release to set CONFIG_LOG_MAXIMUM_LEVEL=0
    list(APPEND SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.release")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PD-step-test)
