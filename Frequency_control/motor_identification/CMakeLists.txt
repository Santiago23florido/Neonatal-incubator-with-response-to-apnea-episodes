# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Initialize the SDK
include(pico_sdk_import.cmake)

project(motor_identification C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add motor identification executable
add_executable(motor_identification
    main.cpp
)

# Pull in common dependencies
target_link_libraries(motor_identification 
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
    pico_time
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(motor_identification 1)
pico_enable_stdio_uart(motor_identification 0)

# Create map/bin/hex file etc.
pico_add_extra_outputs(motor_identification)

# Add a compilation flag to enable the file system (for data logging)
target_compile_definitions(motor_identification PRIVATE PICO_STDIO_USB_ENABLE_CRLF_SUPPORT=1)