cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HelmswheelKing_Gimbal)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/User/gimbal_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User/gimbal_to_chassis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/RemoteControl/DT7.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Alg/PID/pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN/interface/can_bus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN/impl/can_bus_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN/impl/can_device_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART/interface/uart_bus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART/impl/uart_bus_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART/impl/uart_device_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Common/StateWatch/state_watch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Common/StateWatch/buzzer_manager.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Motor
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Motor/DM
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Common/StateWatch
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/RemoteControl
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/CAN/impl
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL/UART/impl
    ${CMAKE_CURRENT_SOURCE_DIR}/Alg/PID
    ${CMAKE_CURRENT_SOURCE_DIR}/User
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
