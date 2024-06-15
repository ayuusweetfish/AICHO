cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)
project(aicho_relay)
pico_sdk_init()

add_executable(aicho_relay)
target_compile_definitions(aicho_relay PRIVATE PICO_FLASH_SIZE_BYTES=16777216)
target_include_directories(aicho_relay PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_sources(aicho_relay PRIVATE src/main.c src/usb_host.c)
target_link_libraries(aicho_relay
  pico_stdlib
  pico_multicore
  tinyusb_host
)
pico_add_extra_outputs(aicho_relay)
