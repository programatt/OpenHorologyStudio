# SPDX-License-Identifier: Apache-2.0
find_package(Catch2 3 REQUIRED)
include(Catch)

add_executable(ohs_tests
  unit/kinematics_test.cpp
  unit/model_test.cpp
  unit/profile_test.cpp
  unit/rigging_test.cpp
  unit/geometry_test.cpp
  integration/pipeline_test.cpp
  integration/cli_test.cpp)
target_include_directories(ohs_tests PRIVATE ${PROJECT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ohs_tests PRIVATE ohs_app ohs_kinematics Catch2::Catch2WithMain nlohmann_json::nlohmann_json)
target_compile_definitions(ohs_tests PRIVATE OHS_CLI_PATH="$<TARGET_FILE:ohs>")
add_dependencies(ohs_tests ohs)
catch_discover_tests(ohs_tests DISCOVERY_MODE PRE_TEST
  PROPERTIES ENVIRONMENT "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp")
