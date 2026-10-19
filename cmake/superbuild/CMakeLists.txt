# SPDX-License-Identifier: Apache-2.0
# Superbuild of the optional libraries OHS can use, each fetched at a pinned commit and installed into one prefix.
#
#   cmake -S cmake/superbuild -B build/superbuild -DCMAKE_INSTALL_PREFIX=$PWD/build/deps
#   cmake --build build/superbuild --parallel
#   cmake --preset release -DCMAKE_PREFIX_PATH=$PWD/build/deps -DOHS_WITH_CHRONO=ON \
#         -DChrono_DIR=$PWD/build/deps/lib/cmake/Chrono
#
# Choose parts with -DOHS_SUPERBUILD_MFEM / _CHRONO / _NETGEN = ON or OFF (MFEM and Chrono are on by
# default, Netgen is off: OHS reaches Netgen through its Python module, `pip install netgen-mesher`).
# The pins are commit hashes, not branches or tags, so the build does not move under you. Update them
# together with docs/install.md when a library is upgraded.
cmake_minimum_required(VERSION 3.28)
project(ohs_superbuild NONE)

include(ExternalProject)
option(OHS_SUPERBUILD_MFEM "Build MFEM (serial, no MPI) for `ohs fea --mfem`" ON)
option(OHS_SUPERBUILD_CHRONO "Build Project Chrono (core only) for `--engine chrono`" ON)
option(OHS_SUPERBUILD_NETGEN "Build Netgen with its OpenCASCADE support" OFF)

# ---- pins -------------------------------------------------------------------------------------------------
set(OHS_MFEM_TAG    "v4.5.2")
set(OHS_MFEM_COMMIT "00b2a0705f647e17a1d4ffcb289adca503f28d42")  # mfem/mfem, tag v4.5.2
set(OHS_CHRONO_TAG    "9.0.1")
set(OHS_CHRONO_COMMIT "2617649bf687456328a122b63dc0bb64df91394a")  # projectchrono/chrono, tag 9.0.1
set(OHS_NETGEN_TAG    "v6.2.2401")
set(OHS_NETGEN_COMMIT "2ff886457fe899685d9d826c8a29f58a31889dda")  # NGSolve/netgen, tag v6.2.2401

set(common_args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

if(OHS_SUPERBUILD_MFEM)
  ExternalProject_Add(mfem
    GIT_REPOSITORY https://github.com/mfem/mfem.git
    GIT_TAG ${OHS_MFEM_COMMIT}
    GIT_SHALLOW OFF
    # MFEM 4.5.2 uses uint64_t and uintptr_t without including <cstdint>, which GCC 13 no longer provides indirectly.
    CMAKE_ARGS ${common_args} "-DCMAKE_CXX_FLAGS=-include cstdint" -DBUILD_SHARED_LIBS=ON -DMFEM_USE_MPI=OFF -DMFEM_ENABLE_EXAMPLES=OFF
               -DMFEM_ENABLE_MINIAPPS=OFF -DMFEM_ENABLE_TESTING=OFF
    USES_TERMINAL_DOWNLOAD ON)
endif()

if(OHS_SUPERBUILD_CHRONO)
  ExternalProject_Add(chrono
    GIT_REPOSITORY https://github.com/projectchrono/chrono.git
    GIT_TAG ${OHS_CHRONO_COMMIT}
    GIT_SHALLOW OFF
    CMAKE_ARGS ${common_args} -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKING=OFF -DBUILD_GMOCK=OFF
               -DENABLE_MODULE_IRRLICHT=OFF -DENABLE_MODULE_VSG=OFF -DENABLE_MODULE_OPENGL=OFF
               -DENABLE_MODULE_VEHICLE=OFF -DENABLE_MODULE_PYTHON=OFF -DENABLE_MODULE_POSTPROCESS=OFF
    USES_TERMINAL_DOWNLOAD ON)
endif()

if(OHS_SUPERBUILD_NETGEN)
  ExternalProject_Add(netgen
    GIT_REPOSITORY https://github.com/NGSolve/netgen.git
    GIT_TAG ${OHS_NETGEN_COMMIT}
    GIT_SHALLOW OFF
    GIT_SUBMODULES_RECURSE ON
    CMAKE_ARGS ${common_args} -DUSE_OCC=ON -DUSE_GUI=OFF -DUSE_PYTHON=OFF -DUSE_NATIVE_ARCH=OFF
    USES_TERMINAL_DOWNLOAD ON)
endif()
