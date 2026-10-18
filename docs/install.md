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
