# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(auk-firmware VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(AUK_WERROR "Treat warnings as errors" ON)
option(AUK_SANITIZE "Build tests with address and UB sanitizers" OFF)

# Warnings are part of the build contract, not decoration. -Wconversion in
# particular earns its keep in firmware, where a silent narrowing of a sensor
# value is a bug you find on the robot rather than at the keyboard.
set(AUK_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    -Wformat=2
    -Wcast-qual
    -Wold-style-cast
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Wnull-dereference
)
if(AUK_WERROR)
    list(APPEND AUK_WARNINGS -Werror)
endif()

# ---------------------------------------------------------------- the library
# Portable and hardware-free: no platform headers, no allocation after startup,
# no exceptions, no RTTI. This is the part that goes on the microcontroller.
add_library(auk STATIC
    src/core/command.cpp
    src/control/differential_drive.cpp
    src/control/odometry.cpp
    src/control/slew_limiter.cpp
)
target_include_directories(auk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(auk PRIVATE ${AUK_WARNINGS})
