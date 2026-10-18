# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.28)
project(OpenHorologyStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OHS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(OHS_ASAN "Build with AddressSanitizer and UBSan" OFF)
option(OHS_BUILD_TESTS "Build the test suite" ON)

find_package(OpenCASCADE REQUIRED)
find_package(tomlplusplus REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(CLI11 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ohs_options INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(ohs_options INTERFACE -Wall -Wextra -Wpedantic)
  if(OHS_WARNINGS_AS_ERRORS)
    target_compile_options(ohs_options INTERFACE -Werror)
  endif()
  if(OHS_ASAN)
    target_compile_options(ohs_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(ohs_options INTERFACE -fsanitize=address,undefined)
  endif()
endif()

add_subdirectory(src)

if(OHS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
