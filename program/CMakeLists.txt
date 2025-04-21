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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
 
project(hammond_i2s C CXX ASM)

pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
        )

add_subdirectory(.. sequencer_synth)

target_link_libraries(${PROJECT_NAME} PRIVATE
        pico_stdlib
        hardware_interp
        sequencer_synth
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE
        # This is where you define the audio output.
        # Make sure to define one of the two lines only.
        # USE_AUDIO_PWM=1
        USE_AUDIO_I2S=1
        )

pico_set_program_name(${PROJECT_NAME} ${PROJECT_NAME})

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)