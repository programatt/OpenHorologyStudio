# SPDX-License-Identifier: Apache-2.0
find_package(Catch2 3 REQUIRED)
include(Catch)

# ---- Test plugins: the example and variants that exercise the loader's refusals ------------------------
function(ohs_test_plugin name source)
  add_library(${name} MODULE ${source})
  target_include_directories(${name} PRIVATE ${PROJECT_SOURCE_DIR}/include)
  target_compile_definitions(${name} PRIVATE ${ARGN})
  set_target_properties(${name} PROPERTIES PREFIX "lib" C_VISIBILITY_PRESET hidden)
  target_link_libraries(${name} PRIVATE m)
endfunction()
set(OHS_EXAMPLE_PLUGIN_SOURCE ${PROJECT_SOURCE_DIR}/examples/plugins/min_hole.c)
ohs_test_plugin(ohs_plugin_example ${OHS_EXAMPLE_PLUGIN_SOURCE})
ohs_test_plugin(ohs_plugin_abi_v2 ${OHS_EXAMPLE_PLUGIN_SOURCE} MIN_HOLE_ABI_MAJOR=2)
ohs_test_plugin(ohs_plugin_abi_newer_minor ${OHS_EXAMPLE_PLUGIN_SOURCE} MIN_HOLE_ABI_MINOR=9)
ohs_test_plugin(ohs_plugin_many ${CMAKE_CURRENT_SOURCE_DIR}/plugins/test_plugins.c)
ohs_test_plugin(ohs_plugin_error ${CMAKE_CURRENT_SOURCE_DIR}/plugins/test_plugins.c TEST_PLUGIN_ERROR)
ohs_test_plugin(ohs_plugin_no_entry ${CMAKE_CURRENT_SOURCE_DIR}/plugins/test_plugins.c TEST_PLUGIN_NO_ENTRY)
ohs_test_plugin(ohs_plugin_incomplete ${CMAKE_CURRENT_SOURCE_DIR}/plugins/test_plugins.c TEST_PLUGIN_INCOMPLETE)

add_executable(ohs_tests
  unit/kinematics_test.cpp
  unit/m1_test.cpp
  unit/dynamics_test.cpp
  unit/mesh_test.cpp
  unit/fea_test.cpp
  unit/fea_analysis_test.cpp
  unit/gear_test.cpp
  unit/report_test.cpp
  unit/manufacturing_test.cpp
  unit/report_builder_test.cpp
  unit/structural_cases_test.cpp
  unit/lessons_test.cpp
  unit/real_file_findings_test.cpp
  unit/plugin_test.cpp
  unit/model_test.cpp
  unit/profile_test.cpp
  unit/rigging_test.cpp
  unit/geometry_test.cpp
  integration/pipeline_test.cpp
  integration/animation_test.cpp
  integration/cli_test.cpp)
target_include_directories(ohs_tests PRIVATE ${PROJECT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ohs_tests PRIVATE ohs_app ohs_kinematics ohs_dynamics ohs_mesh ohs_fea ohs_gear ohs_cases ohs_report ohs_manufacturing ohs_reports ohs_lessons ohs_plugins
                      ohs_explain ohs_animation Catch2::Catch2WithMain nlohmann_json::nlohmann_json)
target_compile_definitions(ohs_tests PRIVATE
  OHS_CLI_PATH="$<TARGET_FILE:ohs>"
  OHS_EXAMPLE_PLUGIN="$<TARGET_FILE:ohs_plugin_example>"
  OHS_ABI_V2_PLUGIN="$<TARGET_FILE:ohs_plugin_abi_v2>"
  OHS_ABI_NEWER_MINOR_PLUGIN="$<TARGET_FILE:ohs_plugin_abi_newer_minor>"
  OHS_MANY_PLUGIN="$<TARGET_FILE:ohs_plugin_many>"
  OHS_ERROR_PLUGIN="$<TARGET_FILE:ohs_plugin_error>"
  OHS_NO_ENTRY_PLUGIN="$<TARGET_FILE:ohs_plugin_no_entry>"
  OHS_INCOMPLETE_PLUGIN="$<TARGET_FILE:ohs_plugin_incomplete>")
if(OHS_ASAN)
  target_compile_definitions(ohs_tests PRIVATE OHS_SANITIZED=1)
endif()
add_dependencies(ohs_tests ohs ohs_plugin_example ohs_plugin_abi_v2 ohs_plugin_abi_newer_minor ohs_plugin_many ohs_plugin_error ohs_plugin_no_entry ohs_plugin_incomplete)
catch_discover_tests(ohs_tests DISCOVERY_MODE PRE_TEST
  PROPERTIES ENVIRONMENT "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp" SKIP_RETURN_CODE 4)

# ---- Python and documentation tests ---------------------------------------------------------------------
find_package(Python 3.10 COMPONENTS Interpreter)
if(Python_Interpreter_FOUND)
  add_test(NAME python_cad_addin_preset
           COMMAND ${Python_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/python -p "test_preset.py" -v)
  set_tests_properties(python_cad_addin_preset PROPERTIES ENVIRONMENT "OHS_CLI=$<TARGET_FILE:ohs>;LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp")
  add_test(NAME documentation_commands_exist
           COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/check_docs.py --ohs $<TARGET_FILE:ohs>
                   --docs ${PROJECT_SOURCE_DIR}/docs --readme ${PROJECT_SOURCE_DIR}/README.md)
  add_test(NAME package_contents
           COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/python/test_package.py ${CMAKE_BINARY_DIR}/CPackConfig.cmake)
  add_test(NAME perf_compare_script
           COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/python/test_perf_compare.py)
  if(OHS_BUILD_PYTHON)
    add_test(NAME python_api
             COMMAND ${Python_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/python -p "test_api.py" -v)
    set_tests_properties(python_api PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python;OHS_CLI=$<TARGET_FILE:ohs>")
  endif()
endif()

if(OHS_BUILD_GUI)
  add_executable(ohs_gui_tests gui/gui_test.cpp)
  target_link_libraries(ohs_gui_tests PRIVATE ohs_gui Catch2::Catch2WithMain)
  add_test(NAME gui_scripted_flow COMMAND ohs_gui_tests)
  set_tests_properties(gui_scripted_flow PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen;LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp")
endif()
