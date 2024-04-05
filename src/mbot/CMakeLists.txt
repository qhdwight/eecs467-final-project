cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mbot_pico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(rc)
add_subdirectory(comms)
