cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
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

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(read_encoder_ticks C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Cambia la extensión a .cpp para indicar que es código C++
add_executable(read_encoder_ticks read_encoder_ticks.cpp)

pico_set_program_name(read_encoder_ticks "read_encoder_ticks")
pico_set_program_version(read_encoder_ticks "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(read_encoder_ticks 0)   # Set to 1 if you need UART output
pico_enable_stdio_usb(read_encoder_ticks 1)    # Set to 1 for USB output

# Add the standard library to the build
target_link_libraries(read_encoder_ticks
        pico_stdlib
        hardware_pwm
        hardware_irq
        hardware_gpio
        hardware_timer     # Para funciones de temporización precisas
        pico_time          # Para funciones relacionadas con el tiempo
        m                  # Biblioteca matemática (para fabs y otras funciones)
        )

# Add the standard include files to the build
target_include_directories(read_encoder_ticks PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Create additional outputs like map/bin/hex/uf2 files
pico_add_extra_outputs(read_encoder_ticks)