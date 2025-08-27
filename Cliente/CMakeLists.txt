# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(client C CXX ASM)

include_directories( ${CMAKE_SOURCE_DIR}/Lib) # Inclui os files .h na pasta lib

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(client client.c Lib/ssd1306.c)


pico_set_program_name(client "client")
pico_set_program_version(client "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(client 0)
pico_enable_stdio_usb(client 1)

# Add the standard library to the build
target_link_libraries(client
        pico_stdlib
        pico_cyw43_arch_none
        pico_btstack_ble
        pico_btstack_cyw43
        hardware_i2c)

# Add the standard include files to the build
target_include_directories(client PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(client PRIVATE
    RUNNING_AS_CLIENT=1
)

# Add any user requested libraries
target_link_libraries(client 
        
        )

pico_add_extra_outputs(client)


