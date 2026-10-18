# SPDX-License-Identifier: Apache-2.0
set(OHS_OCCT_LIBS
  TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
  TKShHealing TKMesh TKXSBase TKLCAF TKCDF TKCAF TKXCAF TKBinXCAF TKBin TKBinL TKXDESTEP)

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

ohs_library(ohs_dynamics dynamics/calibre.cpp dynamics/torque_chain.cpp dynamics/oscillator.cpp dynamics/detailed.cpp
            dynamics/timekeeping.cpp dynamics/timegrapher.cpp dynamics/bench.cpp dynamics/study.cpp)
target_link_libraries(ohs_dynamics PUBLIC ohs_kinematics PRIVATE tomlplusplus::tomlplusplus)

ohs_library(ohs_explain explain/explain.cpp)
target_link_libraries(ohs_explain PUBLIC ohs_kinematics)

ohs_library(ohs_geometry geometry/profile.cpp geometry/step_io.cpp geometry/features.cpp
            geometry/procedural.cpp)
target_include_directories(ohs_geometry SYSTEM PUBLIC ${OpenCASCADE_INCLUDE_DIR})
target_link_directories(ohs_geometry PUBLIC ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(ohs_geometry PUBLIC ohs_model ${OHS_OCCT_LIBS})

ohs_library(ohs_app app/import.cpp)
target_link_libraries(ohs_app PUBLIC ohs_geometry ohs_rigging PRIVATE tomlplusplus::tomlplusplus spdlog::spdlog)

ohs_library(ohs_animation animation/gltf_animation.cpp)
target_link_libraries(ohs_animation PUBLIC ohs_geometry ohs_kinematics PRIVATE nlohmann_json::nlohmann_json)

add_executable(ohs cli/main.cpp cli/dynamics_cli.cpp)
target_link_libraries(ohs PRIVATE ohs_app ohs_kinematics ohs_dynamics ohs_explain ohs_animation CLI11::CLI11 nlohmann_json::nlohmann_json spdlog::spdlog)
target_include_directories(ohs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(ohs PRIVATE OHS_VERSION="${PROJECT_VERSION}" OHS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
install(TARGETS ohs RUNTIME DESTINATION bin)
