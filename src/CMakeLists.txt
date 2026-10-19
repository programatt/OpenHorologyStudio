# SPDX-License-Identifier: Apache-2.0
set(OHS_OCCT_LIBS
  TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
  TKShHealing TKMesh TKXSBase TKIGES TKLCAF TKCDF TKCAF TKXCAF TKBinXCAF TKBin TKBinL TKXDESTEP)

function(ohs_library name)
  add_library(${name} STATIC ${ARGN})
  target_include_directories(${name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${name} PUBLIC ohs_options)
endfunction()

ohs_library(ohs_model model/model.cpp model/rig_io.cpp)
target_link_libraries(ohs_model PRIVATE tomlplusplus::tomlplusplus)

ohs_library(ohs_rigging rigging/rigging.cpp)
target_link_libraries(ohs_rigging PUBLIC ohs_model)

ohs_library(ohs_kinematics kinematics/kinematics.cpp)
target_link_libraries(ohs_kinematics PUBLIC ohs_model)

ohs_library(ohs_dynamics dynamics/calibre.cpp dynamics/torque_chain.cpp dynamics/oscillator.cpp dynamics/detailed.cpp dynamics/chrono_adapter.cpp
            dynamics/timekeeping.cpp dynamics/timegrapher.cpp dynamics/bench.cpp dynamics/study.cpp)
target_link_libraries(ohs_dynamics PUBLIC ohs_kinematics PRIVATE tomlplusplus::tomlplusplus)
if(OHS_WITH_CHRONO)
  target_compile_definitions(ohs_dynamics PUBLIC OHS_WITH_CHRONO)
  # Chrono's config file exports variables, not targets. Only chrono_adapter.cpp includes Chrono
  # headers, so Chrono's compile flags (-march=native, -fopenmp: they change Eigen's alignment) stay on it.
  find_library(OHS_CHRONO_ENGINE ChronoEngine HINTS ${CHRONO_LIB_DIR} ${Chrono_DIR}/../.. REQUIRED)
  target_include_directories(ohs_dynamics SYSTEM PRIVATE ${CHRONO_INCLUDE_DIRS})
  set_source_files_properties(dynamics/chrono_adapter.cpp PROPERTIES COMPILE_OPTIONS "${CHRONO_CXX_FLAGS}")
  target_link_libraries(ohs_dynamics PRIVATE ${OHS_CHRONO_ENGINE} ${CHRONO_LIBRARIES})
endif()

ohs_library(ohs_mesh mesh/mesh2d.cpp mesh/tetmesh.cpp)
target_link_libraries(ohs_mesh PUBLIC ohs_model)

# The material catalogue is embedded from data/materials/materials.toml at configure time.
file(READ ${PROJECT_SOURCE_DIR}/data/materials/materials.toml OHS_MATERIALS_TOML)
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/materials_data.cpp
     CONTENT "namespace ohs::fea {\nextern const char* const kMaterialsToml;\nconst char* const kMaterialsToml = R\"TOML(${OHS_MATERIALS_TOML})TOML\";\n}\n")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/data/materials/materials.toml)
ohs_library(ohs_fea fea/material.cpp fea/element.cpp fea/fea.cpp fea/analysis.cpp fea/results.cpp fea/calculix.cpp fea/mfem_adapter.cpp ${CMAKE_CURRENT_BINARY_DIR}/materials_data.cpp)
target_link_libraries(ohs_fea PUBLIC ohs_mesh Eigen3::Eigen PRIVATE tomlplusplus::tomlplusplus)
if(OHS_MFEM_FOUND)
  target_compile_definitions(ohs_fea PUBLIC OHS_WITH_MFEM)
  target_include_directories(ohs_fea SYSTEM PRIVATE ${OHS_MFEM_INCLUDE} ${OHS_HYPRE_INCLUDE})
  target_link_libraries(ohs_fea PUBLIC ${OHS_MFEM_LIB} MPI::MPI_CXX)
endif()
if(HDF5_FOUND)
  target_compile_definitions(ohs_fea PUBLIC OHS_WITH_HDF5)
  target_include_directories(ohs_fea SYSTEM PUBLIC ${HDF5_INCLUDE_DIRS})
  target_link_libraries(ohs_fea PUBLIC ${HDF5_HL_LIBRARIES} ${HDF5_LIBRARIES})
endif()

ohs_library(ohs_gear gear/gear.cpp)
target_link_libraries(ohs_gear PUBLIC ohs_mesh)

ohs_library(ohs_report report/drawing.cpp report/document.cpp)

ohs_library(ohs_explain explain/explain.cpp)
target_link_libraries(ohs_explain PUBLIC ohs_kinematics)

ohs_library(ohs_geometry geometry/profile.cpp geometry/step_io.cpp geometry/cad_io.cpp geometry/netgen_mesh.cpp geometry/features.cpp
            geometry/procedural.cpp geometry/section.cpp)
target_include_directories(ohs_geometry SYSTEM PUBLIC ${OpenCASCADE_INCLUDE_DIR})
target_link_directories(ohs_geometry PUBLIC ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(ohs_geometry PUBLIC ohs_model ohs_mesh ${OHS_OCCT_LIBS})

# The DFM rule sets are embedded from data/dfm/rules.toml at configure time.
file(READ ${PROJECT_SOURCE_DIR}/data/dfm/rules.toml OHS_DFM_TOML)
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dfm_data.cpp
     CONTENT "namespace ohs::manufacturing {\nextern const char* const kDfmRulesToml;\nconst char* const kDfmRulesToml = R\"TOML(${OHS_DFM_TOML})TOML\";\n}\n")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/data/dfm/rules.toml)
ohs_library(ohs_manufacturing manufacturing/gear_export.cpp manufacturing/tolerance.cpp manufacturing/dfm.cpp manufacturing/collision.cpp manufacturing/part_drawing.cpp
            ${CMAKE_CURRENT_BINARY_DIR}/dfm_data.cpp)
target_link_libraries(ohs_manufacturing PUBLIC ohs_geometry ohs_gear ohs_report ohs_kinematics PRIVATE tomlplusplus::tomlplusplus)

ohs_library(ohs_cases cases/cases.cpp cases/verification.cpp)
target_link_libraries(ohs_cases PUBLIC ohs_fea ohs_gear ohs_geometry)

ohs_library(ohs_reports reports/report_builder.cpp)
target_link_libraries(ohs_reports PUBLIC ohs_manufacturing ohs_cases ohs_dynamics ohs_explain ohs_rigging ohs_report ohs_plugins)

ohs_library(ohs_app app/import.cpp)
target_link_libraries(ohs_app PUBLIC ohs_geometry ohs_rigging PRIVATE tomlplusplus::tomlplusplus spdlog::spdlog)

ohs_library(ohs_animation animation/scene.cpp animation/gltf_animation.cpp animation/usd_animation.cpp)
target_link_libraries(ohs_animation PUBLIC ohs_geometry ohs_kinematics PRIVATE nlohmann_json::nlohmann_json)

ohs_library(ohs_lessons lessons/lessons.cpp)
target_link_libraries(ohs_lessons PUBLIC ohs_cases ohs_dynamics ohs_explain ohs_rigging ohs_manufacturing ohs_fea ohs_geometry)

# Plugin loader: loads shared libraries that implement include/ohs/plugin.h.
ohs_library(ohs_plugins plugins/plugin_loader.cpp)
target_include_directories(ohs_plugins PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ohs_plugins PUBLIC ohs_manufacturing ${CMAKE_DL_LIBS})

# The command line as a library, so the Python module can run it in-process.
ohs_library(ohs_cli cli/main.cpp cli/dynamics_cli.cpp cli/fea_cli.cpp cli/manufacturing_cli.cpp cli/lessons_cli.cpp cli/cli_run.cpp)
target_link_libraries(ohs_cli PUBLIC ohs_app ohs_reports ohs_cases ohs_kinematics ohs_dynamics ohs_explain ohs_animation ohs_lessons ohs_plugins
                      CLI11::CLI11 nlohmann_json::nlohmann_json spdlog::spdlog)
target_compile_definitions(ohs_cli PUBLIC OHS_VERSION="${PROJECT_VERSION}" OHS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(ohs cli/entry.cpp)
target_link_libraries(ohs PRIVATE ohs_cli)

add_executable(ohs_perf ${PROJECT_SOURCE_DIR}/tests/perf/perf_main.cpp)
target_link_libraries(ohs_perf PRIVATE ohs_lessons ohs_reports ohs_manufacturing ohs_fea ohs_gear ohs_dynamics)

# ---- Qt desktop application (optional) --------------------------------------------------------------
if(OHS_BUILD_GUI)
  find_package(Qt6 6.5 REQUIRED COMPONENTS Widgets)
  set(CMAKE_AUTOMOC ON)
  # The scene is Qt-free (tessellated parts, pose, exploded view, section) and tested without a display.
  ohs_library(ohs_gui_scene gui/scene.cpp)
  target_link_libraries(ohs_gui_scene PUBLIC ohs_geometry ohs_kinematics ohs_model)
  ohs_library(ohs_gui gui/viewport.cpp gui/plot.cpp gui/job_queue.cpp gui/main_window.cpp)
  target_link_libraries(ohs_gui PUBLIC ohs_gui_scene ohs_cli Qt6::Widgets)
  # Analyses run as worker processes of the command line tool, found next to the GUI when installed and
  # in the build tree otherwise.
  add_dependencies(ohs_gui ohs)
  target_compile_definitions(ohs_gui PRIVATE OHS_CLI_EXECUTABLE="$<TARGET_FILE:ohs>")
  add_executable(ohs-gui gui/gui_main.cpp)
  target_link_libraries(ohs-gui PRIVATE ohs_gui)
  install(TARGETS ohs-gui RUNTIME DESTINATION bin)
endif()

# ---- Python extension (optional) --------------------------------------------------------------------
if(OHS_BUILD_PYTHON)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir OUTPUT_VARIABLE OHS_NANOBIND_DIR
                  OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE OHS_NANOBIND_RESULT)
  if(NOT OHS_NANOBIND_RESULT EQUAL 0)
    message(FATAL_ERROR "OHS_BUILD_PYTHON needs nanobind: pip install nanobind")
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${OHS_NANOBIND_DIR}")
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_native NB_STATIC python/module.cpp)
  target_link_libraries(_native PRIVATE ohs_cli)
  set(OHS_PYTHON_OUT ${CMAKE_BINARY_DIR}/python/ohs)
  set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${OHS_PYTHON_OUT})
  foreach(config IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY_${config} ${OHS_PYTHON_OUT})
  endforeach()
  # The pure-Python half of the package sits next to the extension in the build tree.
  file(GLOB OHS_PYTHON_FILES ${PROJECT_SOURCE_DIR}/python/ohs/*.py ${PROJECT_SOURCE_DIR}/python/ohs/*.json)
  add_custom_target(ohs_python_package ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${OHS_PYTHON_OUT}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OHS_PYTHON_FILES} ${OHS_PYTHON_OUT}
    DEPENDS _native)
  # scikit-build-core installs relative to the wheel root; a system package uses the distro Python path.
  if(SKBUILD)
    set(OHS_PY_DEST ohs)
  else()
    set(OHS_PY_DEST lib/python3/dist-packages/ohs)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/ohs/ DESTINATION ${OHS_PY_DEST} FILES_MATCHING PATTERN "*.py" PATTERN "*.json")
  endif()
  install(TARGETS _native LIBRARY DESTINATION ${OHS_PY_DEST})
endif()

# ---- Install and packaging ---------------------------------------------------------------------------------
install(TARGETS ohs RUNTIME DESTINATION bin)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/ DESTINATION share/ohs/data PATTERN "*.step" EXCLUDE PATTERN "*.stp" EXCLUDE)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts/thirdparty/ DESTINATION share/ohs/scripts/thirdparty)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ohs DESTINATION include)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/examples/ DESTINATION share/ohs/examples)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/integrations/ DESTINATION share/ohs/integrations)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/docs/ DESTINATION share/doc/ohs)
install(FILES ${PROJECT_SOURCE_DIR}/LICENSE.md ${PROJECT_SOURCE_DIR}/README.md ${PROJECT_SOURCE_DIR}/CHANGELOG.md DESTINATION share/doc/ohs)
if(NOT OHS_BUILD_PYTHON)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/ohs DESTINATION share/ohs/python PATTERN "__pycache__" EXCLUDE)
endif()
