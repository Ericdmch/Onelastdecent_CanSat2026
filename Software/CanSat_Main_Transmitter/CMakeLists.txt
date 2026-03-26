cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(CanSat_Main_Transmitter C CXX ASM)

pico_sdk_init()

add_executable(CanSat_Main_Transmitter
    main.cpp
    bme680.cpp
    e220.cpp
    fc_link.cpp
    neopixel.cpp
)

pico_generate_pio_header(CanSat_Main_Transmitter ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_set_program_name(CanSat_Main_Transmitter "CanSat_Main_Transmitter")
pico_set_program_version(CanSat_Main_Transmitter "1.0")

# stdio: USB only for debug; UART0 is reserved for E220
pico_enable_stdio_uart(CanSat_Main_Transmitter 0)
pico_enable_stdio_usb(CanSat_Main_Transmitter 1)

target_include_directories(CanSat_Main_Transmitter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(CanSat_Main_Transmitter
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_timer
    hardware_watchdog
    hardware_clocks
    hardware_gpio
    pico_cyw43_arch_none
    hardware_pio
)

pico_add_extra_outputs(CanSat_Main_Transmitter)
