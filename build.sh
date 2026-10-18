#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# One-command build and test: ./build.sh [--debug|--asan] [--jobs N] [--no-test]
set -euo pipefail
cd "$(dirname "$0")"

preset=release
jobs=""
run_tests=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug) preset=debug ;;
    --asan) preset=asan ;;
    --jobs) jobs="$2"; shift ;;
    --no-test) run_tests=0 ;;
    -h|--help) sed -n '3p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

missing=0
need() { command -v "$1" >/dev/null 2>&1 || { echo "missing: $1  ($2)" >&2; missing=1; }; }
need cmake "need CMake 3.28+: pip install cmake, or your package manager"
need ninja "apt install ninja-build"
need g++ "apt install g++-13, or use clang++ 17+"
need git "apt install git"
need python3 "apt install python3"
[[ $missing -eq 0 ]] || exit 1

cmake --preset "$preset" || {
  echo >&2
  echo "Configure failed. On Debian/Ubuntu the dependencies are:" >&2
  echo "  sudo apt install libocct-data-exchange-dev libocct-modeling-algorithms-dev libocct-foundation-dev \\" >&2
  echo "    libocct-modeling-data-dev libocct-ocaf-dev libocct-visualization-dev libeigen3-dev libcli11-dev \\" >&2
  echo "    libspdlog-dev nlohmann-json3-dev libtomlplusplus-dev catch2 libtbb-dev libhdf5-dev" >&2
  exit 1
}
cmake --build --preset "$preset" ${jobs:+--parallel "$jobs"}
if [[ $run_tests -eq 1 ]]; then
  ctest --preset "$preset"
fi
echo "ohs binary: build/$preset/src/ohs"
