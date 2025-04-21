# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

set(TARGET_NAME "sequencer_synth")

if (NOT TARGET ${TARGET_NAME})
    add_library(${TARGET_NAME} INTERFACE)

    target_sources(${TARGET_NAME} INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/sound_pwm/sound_pwm.c
            ${CMAKE_CURRENT_LIST_DIR}/sound_i2s/sound_i2s.c
    )
    
    pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/sound_i2s/sound_i2s_16bits.pio)
    pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/sound_i2s/sound_i2s_8bits.pio)

    target_include_directories(${TARGET_NAME} INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/sound_i2s
            ${CMAKE_CURRENT_LIST_DIR}/sound_pwm
    )

    target_link_libraries(${TARGET_NAME} INTERFACE
        pico_stdlib
        hardware_pwm
        hardware_irq
        hardware_dma
        hardware_pio
    )
endif()
