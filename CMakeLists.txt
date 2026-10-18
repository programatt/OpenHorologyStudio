# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.28)
project(OpenHorologyStudio VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # the static libraries also go into the Python module

option(OHS_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(OHS_ASAN "Build with AddressSanitizer and UBSan" OFF)
option(OHS_BUILD_TESTS "Build the test suite" ON)
option(OHS_BUILD_PYTHON "Build the Python extension module (needs nanobind)" OFF)

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

# ---- Packages: cpack -G DEB | TGZ (Linux), DragNDrop (macOS), WIX (Windows) ----------------------------
set(CPACK_PACKAGE_NAME "ohs")
set(CPACK_PACKAGE_VENDOR "Open Horology Studio contributors")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Watch movement design and analysis")
set(CPACK_PACKAGE_CONTACT "Open Horology Studio <noreply@openhorologystudio.invalid>")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_GENERATOR "TGZ")
include(CPack)
