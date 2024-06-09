cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)
project(aicho)
pico_sdk_init()

add_executable(aicho)
target_compile_definitions(aicho PRIVATE PICO_FLASH_SIZE_BYTES=16777216)
target_include_directories(aicho PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

pico_generate_pio_header(aicho ${CMAKE_CURRENT_LIST_DIR}/src/i2s_out.pio)
pico_generate_pio_header(aicho ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)
target_sources(aicho PRIVATE src/main.c src/usb_host.c)
target_link_libraries(aicho
  pico_stdlib
  pico_multicore
  hardware_dma
  hardware_flash
  hardware_pio
  tinyusb_host
)
pico_add_extra_outputs(aicho)
