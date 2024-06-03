cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)
project(aicho)
pico_sdk_init()

add_executable(aicho main.c)
target_link_libraries(aicho pico_stdlib)
pico_add_extra_outputs(aicho)
