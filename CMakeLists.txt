# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/siegeljb/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MegaSquirtTacho C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(MegaSquirtTacho MegaSquirtTacho.cpp megasquirt_simplified_dash_broadcast.c include/mcp2515/mcp2515.cpp)

pico_set_program_name(MegaSquirtTacho "MegaSquirtTacho")
pico_set_program_version(MegaSquirtTacho "0.1")

pico_enable_stdio_uart(MegaSquirtTacho 0)
pico_enable_stdio_usb(MegaSquirtTacho 1)

# Add any user requested libraries
target_link_libraries(MegaSquirtTacho
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_timer
        hardware_watchdog
        hardware_pwm
        )

pico_add_extra_outputs(MegaSquirtTacho)

