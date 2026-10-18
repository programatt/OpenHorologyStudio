# Install

## From a release package

Release packages are built with CPack: a `.deb` for Debian and Ubuntu, a `.tar.gz` for any Linux,
`.dmg` and `.msi` on macOS and Windows. They contain the `ohs` program, the material, DFM and
lesson data, the plugin header and the Python package.

```
sudo apt install ./ohs_1.0.0_amd64.deb
ohs --version
```

## From source

You need CMake 3.28+, Ninja, a C++20 compiler (GCC 13+, Clang 17+, Apple Clang 16, or Visual
Studio 2022 17.10+), Git and Python 3.10+.

On Ubuntu 24.04:

```
sudo apt install cmake ninja-build g++ git python3 \
    libocct-data-exchange-dev libocct-modeling-algorithms-dev libocct-foundation-dev \
    libocct-modeling-data-dev libocct-ocaf-dev libocct-visualization-dev libeigen3-dev \
    libcli11-dev libspdlog-dev nlohmann-json3-dev libtomlplusplus-dev catch2
./build.sh               # release build and the whole test suite
./build.sh --asan        # the same under AddressSanitizer and UBSan
```

`build.sh` prints the install hint for whatever is missing. The `ohs` binary ends up in
`build/release/src/ohs`.

### Project Chrono (optional)

Chrono 9 is not packaged for Debian or Ubuntu and has no usable PyPI wheel. Build the core module
and point OHS at it:

```
git clone --depth 1 --branch 9.0.1 https://github.com/projectchrono/chrono.git
cmake -S chrono -B chrono-build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/chrono \
      -DBUILD_DEMOS=OFF -DBUILD_DEMOS_BASE=OFF -DBUILD_DEMOS_COSIMULATION=OFF -DBUILD_DEMOS_FEA=OFF \
      -DBUILD_DEMOS_ROBOT=OFF
cmake --build chrono-build && cmake --install chrono-build
cmake -S . -B build/chrono -G Ninja -DOHS_WITH_CHRONO=ON -DChrono_DIR=$HOME/chrono/lib/cmake/Chrono
```

`ohs verify levels --engine chrono` then runs the detailed model on Chrono. On a machine whose
clocksource is `hpet` (check `/sys/devices/system/clocksource/clocksource0/current_clocksource`)
it is several times slower, because Chrono's profiling timers call `clock_gettime` every step and
each call is a system call.

### Desktop application

```
sudo apt install qt6-base-dev          # or point CMAKE_PREFIX_PATH at any Qt 6.5+ (aqtinstall works)
cmake --preset gui && cmake --build --preset gui && ctest --preset gui
build/gui/src/ohs-gui
```

The `gui` preset turns on `OHS_BUILD_GUI`. The scripted window test runs on Qt's offscreen platform,
so it needs no display. See [Desktop application](desktop.md).

### Python

The Python package is built with nanobind when it is found:

```
pip install nanobind scikit-build-core
cmake --preset release -DOHS_BUILD_PYTHON=ON
cmake --build --preset release
PYTHONPATH=build/release/python python3 -c "import ohs; print(ohs.__version__)"
```

### Packages

```
cmake --build --preset release
cpack --config build/release/CPackConfig.cmake -G DEB      # or TGZ
```

## Checking your install

```
ohs verify physics          # re-runs the structural solvers against analytical answers
ohs verify levels demo.hsproj   # compares the two timekeeping models on a project
```

Both exit with 0 when everything agrees and 1 when something is off, so they work in scripts.
