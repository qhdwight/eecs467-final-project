cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk)
set(PICO_CMAKE_FILE ${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

if (EXISTS ${PICO_CMAKE_FILE})
    include(${PICO_CMAKE_FILE})

    project(mbot_pico C CXX ASM)

    # Initialize the SDK
    pico_sdk_init()

    add_subdirectory(rc)
    add_subdirectory(comms)

    add_executable(drive_pico drive/drive.cpp)
    target_link_libraries(drive_pico
            pico_stdlib
            pico_multicore
            pico_time
            hardware_pwm
            hardware_sync
            hardware_i2c
            hardware_flash
            rclib
            comms
    )
    pico_enable_stdio_usb(drive_pico 1)
    pico_enable_stdio_uart(drive_pico 0)
    pico_add_extra_outputs(drive_pico)
else ()
    message(STATUS "Pico SDK not found")
endif ()
