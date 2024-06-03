cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)
project(aicho)
pico_sdk_init()

add_executable(aicho)
pico_generate_pio_header(aicho ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
target_sources(aicho PRIVATE main.c)
target_link_libraries(aicho pico_stdlib hardware_pio)
pico_add_extra_outputs(aicho)
