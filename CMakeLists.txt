cmake_minimum_required(VERSION 3.10)
project(NrfLibUART VERSION 1.0.0)

zephyr_library_named(NrfLibUART)

zephyr_library_include_directories(include)
target_include_directories(NrfLibUART INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
zephyr_library_sources(src/lib_uart.cpp)
zephyr_library_sources(src/periphery/lib_dfr_c4001.cpp)
zephyr_library_sources(src/periphery/lib_ld2412.cpp)
