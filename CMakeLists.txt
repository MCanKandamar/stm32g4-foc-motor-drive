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

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 015_BLDC_FOC_RTOS_MotorDrive)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    App/Src/isr.cpp
    App/Src/adc_measurements.cpp
    App/Src/scrutiny.cpp
    motor_control/inc/filters.hpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add hardware-independent motor control library
add_subdirectory(motor_control)

# CANopen stack (CANopenNode + STM32 FDCAN driver + Object Dictionary)
add_subdirectory(canopen)

# Scrutiny debugger: scrutiny_build_config.hpp is made from the .in template with these values
set(SCRUTINY_ENABLE_DATALOGGING           ON)
set(SCRUTINY_SUPPORT_64BITS               ON)
set(SCRUTINY_SUPPORT_PROTECTED_REGIONS    ON)
set(SCRUTINY_REQUEST_MAX_PROCESS_TIME_US  100000)
set(SCRUTINY_COMM_RX_TIMEOUT_US           50000)
set(SCRUTINY_COMM_HEARTBEAT_TIMEOUT_US    5000000)
set(SCRUTINY_PROTOCOL_VERSION_MAJOR       1)
set(SCRUTINY_PROTOCOL_VERSION_MINOR       0)
set(SCRUTINY_DATALOGGING_MAX_SIGNAL       16)
set(SCRUTINY_DATALOGGING_ENCODING         SCRUTINY_DATALOGGING_ENCODING_RAW)
set(SCRUTINY_DATALOGGING_BUFFER_32BITS    OFF)
set(INSTALL_FOLDER                        scrutiny)   # scrutiny's install() rules do not accept an empty target
add_subdirectory(scrutiny-embedded)
# Scrutiny is written for C++11; in C++20, '|' between enums gives a deprecated warning (an error with -Werror).
# PUBLIC: also applies to App code that includes the scrutiny headers.
target_compile_options(scrutiny-embedded PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Application source files
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app.cpp
    App/Src/Supervisor.cpp
    App/Src/StartupSeq.cpp
    App/Src/Monitor.cpp
    App/Src/canopen_app.c
    App/Src/canopen_bridge.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    motor_control
    scrutiny-embedded
    canopen
)

# Scrutiny post-build: .sfd (variable map) + <project>_tagged.elf with the firmware ID inside
# Flash the tagged ELF to the board. If the scrutiny CLI is not installed, this step is skipped and the build still works.
# HINTS is searched before PATH: the installer version (same as the GUI) wins over the older pip version
find_program(SCRUTINY_CMD scrutiny HINTS "C:/Program Files (x86)/Scrutiny Debugger")
if(SCRUTINY_CMD)
    include(scrutiny-embedded/cmake/scrutiny.cmake)
    get_filename_component(TOOLCHAIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
    scrutiny_postbuild(${CMAKE_PROJECT_NAME}
        SCRUTINY_CMD            ${SCRUTINY_CMD}
        CPPFILT                 ${TOOLCHAIN_DIR}/arm-none-eabi-c++filt${CMAKE_HOST_EXECUTABLE_SUFFIX}
        TAGGED_EXECUTABLE_NAME  ${CMAKE_PROJECT_NAME}_tagged.elf   # cannot get the extension from the target, so it is given by hand
        METADATA_PROJECT_NAME   "BLDC FOC"
        INSTALL_SFD)
else()
    message(WARNING "scrutiny bulunamadı: .sfd ve tagged ELF üretilmeyecek")
endif()
