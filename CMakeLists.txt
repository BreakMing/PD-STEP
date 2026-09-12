# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

# DEBUG / RELEASE build type control
# Default: Debug (full ESP_LOG output)
# Release: set -DCMAKE_BUILD_TYPE=Release to disable all ESP_LOG
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PD-step-test)
