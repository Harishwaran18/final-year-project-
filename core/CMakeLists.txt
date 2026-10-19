# Standalone (ROS-free) build of the SIL AEB algorithm cores plus their
# GoogleTest suites.
#
# This is NOT the ROS 2 workspace entry point: `colcon build` discovers each
# package through its own package.xml and CMakeLists.txt, and never reads this
# file. This project exists so the numerical core (EKF, geometry, TTC/FSM, CAN
# codec, CARLA frame maths) can be built and tested on a plain host without a
# ROS 2 installation, which is what the CI "core tests" job does.
#
#   cmake -S . -B build
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(sil_aeb_core_tests VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SIL_ENABLE_WERROR "Treat compiler warnings as errors" ON)
option(SIL_ENABLE_SANITIZERS "Build with ASan/UBSan" OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
if(SIL_ENABLE_WERROR)
  add_compile_options(-Werror)
endif()
if(SIL_ENABLE_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

include(cmake/core_tests.cmake)
