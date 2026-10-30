# Hardware-independent FOC library.
# Builds from the same sources on the PC and on the STM32.
#
#   Header files: inc/*.hpp   →  to use: #include "pi_controller.hpp"
#   Source files: src/*.cpp
#
# All .cpp files in src/ are found automatically; when you add a new file
# you do not need to change this file.
file(GLOB MOTOR_CONTROL_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

if(MOTOR_CONTROL_SOURCES)
    add_library(motor_control STATIC ${MOTOR_CONTROL_SOURCES})
    target_include_directories(motor_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
    target_compile_options(motor_control PRIVATE
        -Wall -Wextra -Wpedantic
        -Wdouble-promotion   # catch hidden float to double promotion (the Cortex-M FPU is float only)
        -Wfloat-conversion   # catch hidden double -> float conversions
    )
else()
    # No .cpp yet: a library that only shares header files.
    add_library(motor_control INTERFACE)
    target_include_directories(motor_control INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
endif()
