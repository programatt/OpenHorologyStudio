# Open Horology Studio (OHS)

Watch movement design and analysis. Model in any CAD tool, export STEP, and OHS rigs the assembly
into a working movement, then animates it, simulates how it keeps time, checks that it is strong
enough, checks that you can make it with your tools, and teaches you as you go.
See [`plan.md`](plan.md) for the design and the roadmap, and [`docs/`](docs/index.md) for the
user guide.

**Status: 1.0.** A complete command-line and Python tool (`ohs`):

| You want to | Command |
| --- | --- |
| Import a CAD assembly and rig it | `ohs import`, `ohs rig check`, `ohs rig set` |
| Read and animate the going train | `ohs kinematics`, `ohs explain`, `ohs animate` |
| See how it keeps time | `ohs torque`, `ohs timekeeping`, `ohs simulate`, `ohs timegrapher`, `ohs whatif` |
| Calibrate against your timegrapher | `ohs bench compare`, `ohs bench calibrate`, `ohs calibre init` |
| Check strength | `ohs fea` (tooth bending, shock, modes), `ohs materials`, `ohs thermal` |
| Check you can make it | `ohs tolerance`, `ohs dfm`, `ohs collisions` |
| Make a part | `ohs gear` (STEP, DXF, SVG, PDF), `ohs draw` |
| Write it up | `ohs report` (HTML and PDF) |
| Learn by doing | `ohs lesson list`, `ohs lesson show`, `ohs lesson check` |
| Trust the solvers | `ohs verify physics --full`, `ohs verify levels` |

Not in 1.0: the full graphical interface (`ohs-gui` is a first, basic one: [`docs/desktop.md`](docs/desktop.md)), VTK rendering (OHS's own solvers stand in for MFEM and VTK; MFEM is linked only as an independent
cross-check, `ohs fea --mfem`, Netgen is an optional out-of-process 3D mesher, `--mesher netgen`, and Chrono is an
optional adapter for the escapement, `-DOHS_WITH_CHRONO=ON`) and escapements other than the Swiss lever and the spring detent (reduced model). [`docs/limitations.md`](docs/limitations.md) is the full list.

## Build

Everything below was run on Ubuntu 24.04 with GCC 13; each command is what produced the binaries.

```
# 1. Dependencies (apt provides OCCT 7.6; the plan targets 8.0)
sudo apt install cmake ninja-build g++ python3 \
  libocct-data-exchange-dev libocct-modeling-algorithms-dev libocct-foundation-dev \
  libocct-modeling-data-dev libocct-ocaf-dev libocct-visualization-dev libeigen3-dev libcli11-dev \
  libspdlog-dev nlohmann-json3-dev libtomlplusplus-dev catch2 libtbb-dev libhdf5-dev
sudo apt install qt6-base-dev          # for the desktop app

# 2. Command line tool: release build and the test suite
./build.sh --jobs 14                   # -> build/release/src/ohs

# 3. Desktop app (Qt 6 Widgets): same script, GUI preset
./build.sh --gui --jobs 14 --no-test   # -> build/gui/src/ohs-gui (and build/gui/src/ohs)
```

Run them with `build/release/src/ohs --help` and `build/gui/src/ohs-gui` (add a project:
`build/gui/src/ohs-gui movement.hsproj`; `File, Open the demonstration movement` needs no CAD file).
`./build.sh --help` lists the other builds (`--debug`, `--asan`, `--tsan`, `--coverage`). The GUI tests run
offscreen: `ctest --preset gui` after a `--gui` build.

Optional libraries are used when found at configure time, and `ohs fea` says so when a feature is missing:
`libmfem-dev` (`--mfem`, `--solver mfem`), `libvtk9-dev` (`--vtu-compressed`), `libhdf5-dev` (`--hdf5`),
`ffmpeg` at run time (video export) and `pip install netgen-mesher` (`--mesher netgen`). Chrono and a
serial MFEM can be built at pinned commits with `cmake/superbuild`. The Python package
(`-DOHS_BUILD_PYTHON=ON`, needs nanobind), packages (`cpack`), Windows (`build.ps1`) and macOS presets are
described in [`docs/install.md`](docs/install.md).

## Try it without any CAD file

```
build/release/src/ohs dev gen-movement -o train.step
build/release/src/ohs import train.step -o train.hsproj --beat-rate 25200
build/release/src/ohs kinematics train.hsproj
build/release/src/ohs timekeeping train.hsproj
build/release/src/ohs lesson show hairspring
```

Exit codes: 0 ok, 1 a check failed, 2 usage or input error. Every command takes `--json`.

```python
import ohs
p = ohs.Project.import_step("train.step", "train.hsproj", beat_rate=25200)
print(p.timekeeping()["power_reserve_h"])
```

## Extending

- **Python API 1.0** (frozen): [`docs/python.md`](docs/python.md)
- **Plugin ABI 1.0** (DFM rule sets in any language): [`docs/plugins.md`](docs/plugins.md),
  [`include/ohs/plugin.h`](include/ohs/plugin.h), [`examples/plugins/`](examples/plugins/)
- **CAD add-ins** (Fusion 360, SolidWorks): [`integrations/`](integrations/README.md)

## OM10

`ohs thirdparty om10 fetch --file <your downloaded .stp>` installs it under your user data
directory and imports it. OM10 files are CC BY-SA 3.0 and are never committed here
(`.gitignore` blocks `*.stp`/`*.step`). The [OM10 tutorial](docs/om10.md) walks through a whole
modification study with the guided lessons.

## Licence

Apache-2.0. See [`LICENSE.md`](LICENSE.md) and [`CHANGELOG.md`](CHANGELOG.md).
