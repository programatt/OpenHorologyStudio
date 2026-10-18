# Open Horology Studio (OHS)

Watch movement design and analysis. Model in any CAD tool, export STEP, and OHS rigs the
assembly into a working movement. See `plan.md` for the full design.

**Status: M4.** Manufacturing (`ohs gear`, `draw`, `tolerance`, `dfm`, `collisions`, `report`), structural analysis (`ohs fea`, `materials`, `thermal`, `verify physics`) and timekeeping simulation (`ohs timekeeping`, `simulate`, `timegrapher`, `whatif`, `bench`) is in; see plan.md. Earlier milestones: STEP import (Open CASCADE), tooth counting from the
B-rep, part classification, mobile grouping, gear-mesh inference, TOML rig/project files,
rigid-ratio train kinematics (beat rate, per-mobile speeds) and the `ohs` CLI. Not yet: UI,
dynamics, FEA, tolerance/DFM, Python bindings, Windows/macOS build scripts.

## Build

```
./build.sh              # release build + tests  (--debug, --asan, --jobs N, --no-test)
```

Needs CMake 3.28+, Ninja, GCC 13+ or Clang 17+, and on Ubuntu 24.04:
`sudo apt install libocct-data-exchange-dev libocct-modeling-algorithms-dev libocct-foundation-dev
libocct-modeling-data-dev libocct-ocaf-dev libocct-visualization-dev libcli11-dev libspdlog-dev
nlohmann-json3-dev libtomlplusplus-dev catch2`. (apt provides OCCT 7.6; the plan targets 8.0.)

## Try it without any CAD file

```
build/release/src/ohs dev gen-movement -o /tmp/train.step
build/release/src/ohs import /tmp/train.step -o /tmp/train.hsproj --beat-rate 25200
build/release/src/ohs rig check /tmp/train.hsproj
build/release/src/ohs kinematics /tmp/train.hsproj
```

Exit codes: 0 ok, 1 a check failed, 2 usage or input error. Every command takes `--json`.

## OM10

`ohs thirdparty om10 fetch --file <your downloaded .stp>` installs it under your user data
directory and imports it. OM10 files are CC BY-SA 3.0 and are never committed here
(`.gitignore` blocks `*.stp`/`*.step`).
