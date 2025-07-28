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
set(CMAKE_PROJECT_NAME C8T6_Test)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Drivers/Peter/Mdrv.h
        Drivers/Peter/Mdrv.c
        Drivers/Peter/MPU6050.c
        Drivers/Peter/MPU6050.h
        Drivers/Peter/Display/OLED/Oled.c
        Drivers/Peter/Display/OLED/Oled.h
        Drivers/Peter/Display/OLED/Oledfont.c
        Drivers/Peter/Display/OLED/Oledfont.h
        Drivers/Peter/motion_control.c
        Drivers/Peter/motion_control.h
        Drivers/Peter/Global.h
        Drivers/Peter/Display/LED/LED.c
        Drivers/Peter/Display/LED/LED.h
        Drivers/Peter/Ganv_Grayscale_Sensor.c
        Drivers/Peter/Ganv_Grayscale_Sensor_Config.h
        Drivers/Peter/Eight_LineWalking.c
        Drivers/Peter/Eight_LineWalking.h
        Drivers/Peter/PID.c
        Drivers/Peter/PID.h
        Drivers/Peter/VL53L1/VL5311H.c
        Drivers/Peter/VL53L1/VL5311H.h
        Drivers/Peter/VL53L1/VL53L1X_api.c
        Drivers/Peter/VL53L1/vl53l1_platform.c
        Drivers/Peter/VL53L1/vl53l1_error_codes.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Drivers/Peter
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
