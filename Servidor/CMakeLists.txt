cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(server C CXX ASM)

pico_sdk_init()

add_executable(server server.c lib/ssd1306.c)

pico_set_program_name(server "server")
pico_set_program_version(server "0.1")

pico_enable_stdio_uart(server 0)
pico_enable_stdio_usb(server 1)

target_link_libraries(server
        pico_stdlib
        pico_btstack_cyw43
        pico_cyw43_arch_none
        pico_btstack_ble
        hardware_adc
        pico_lwip_iperf
        pico_cyw43_arch_lwip_threadsafe_background
        hardware_i2c
        )

target_include_directories(server PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

pico_btstack_make_gatt_header(server PRIVATE "${CMAKE_CURRENT_LIST_DIR}/temp_sensor.gatt")

target_link_libraries(server 
        
        )

pico_add_extra_outputs(server)
