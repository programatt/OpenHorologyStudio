# Changelog

All notable changes to Open Horology Studio. The format follows Keep a Changelog; versions follow
Semantic Versioning, with the Python API and the plugin ABI versioned as described in
`docs/python.md` and `docs/plugins.md`.

## Unreleased

Found by running the plan's gates on the real OM10 file.

### Added
- `ohs-gui`: undo and redo for every project edit, by snapshots of the project file (so undo restores
  the file byte for byte), with *declare mesh* as a second undoable edit and an Edit history dock.
- Cycloidal gear profiles: `ohs gear --profile cycloidal --rolling-radius`, a cycloid flank fit on
  import (`profile = "cycloidal"` and `rolling_radius` in the project file), Explain, DFM undercut
  and the tooth-bending case follow the profile.
- `ohs import` and every command that re-reads the CAD file accept IGES (`.igs`, `.iges`) and DXF
  (`.dxf`) as well as STEP; `geometry::import_cad` dispatches by extension.
- `ohs animate -o file.usda` exports the animation as OpenUSD text; the glTF and USD writers share
  their keyframe and tessellation code.
- `ohs fea --hdf5` writes the mesh and result fields to an HDF5 file (optional `libhdf5`).
- `tsan`, `coverage` and `tidy` CMake presets (`./build.sh --tsan|--coverage`), with ThreadSanitizer,
  gcovr coverage and clang-tidy/cppcheck jobs in CI.
- Optional Project Chrono adapter for the level-3 simulation (`-DOHS_WITH_CHRONO=ON`,
  `--engine chrono` on `ohs simulate --level detailed` and `ohs verify levels`): the balance and
  escape wheel as Chrono shafts with its torsion spring and timestepper.
- `ohs-gui` is a basic Qt 6 desktop application (optional, `-DOHS_BUILD_GUI=ON`), with
  a 3D viewport, rig list, Explain panel, animation timeline, analyses as queued jobs, one plot,
  undo of role changes and a `--snapshot` option that renders the viewport to a PNG.
- The dial axis is found from the geometry (`ohs import --up auto`, the default) and stored in the
  project file; `--up` on other commands now defaults to the stored axis.
- `ohs rig mesh` and `[[mesh]]` preset rules declare meshes the geometry does not show.
- Without a barrel, the train starts at the far end of the gear chain from the escape wheel.
- OM10 preset: escape wheel, barrel and the barrel mesh.

### Fixed
- Only round pivot and screw holes are clamped in the shock case; windows and slots were clamped
  too, which made skeletonised parts look much stronger than they are.
- The collision report printed 10^30 mm for pairs that never come within the search range.
- A wrong `--view` mapping in the viewport (top and side were swapped).
- The OM10 tutorial: escape wheel tooth count, mobile numbers, and part names.

## 1.0.0

First release.

### Added
- **Import and rig** (M0, M1): STEP AP242 import through Open CASCADE, tooth counting, role
  classification, mobile grouping, mesh inference, involute flank fitting, TOML project files,
  train kinematics, animated glTF export, the Explain output.
- **Timekeeping** (M2): torque chain, event-driven reduced escapement model, detailed contact model
  as an independent cross-check, virtual timegrapher, positions, power reserve, what-if studies,
  bench calibration against timegrapher readings.
- **Structural analysis** (M3): mesher for flat parts, 4- and 10-node tetrahedra, static, modal,
  transient, nonlinear and contact solvers, materials catalogue, tooth-bending and shock cases,
  CalculiX adapter, `ohs verify physics`.
- **Manufacturing** (M4): tolerance Monte Carlo, DFM rule sets for your own processes, collision
  sweep, involute gear generator (STEP, DXF, SVG, PDF), part drawings, HTML and PDF reports.
- **Learning and release** (M5): seven guided lessons with checked exercises, the frozen Python
  API 1.0, plugin ABI 1.0 for DFM rules, CAD add-in role export, documentation site and generated
  CLI reference, CPack packages, benchmarks.

### Not in this release
See `docs/limitations.md`: only a basic Qt desktop application (`ohs-gui`, optional), no MFEM/Netgen and Chrono only as an optional escapement adapter (OHS's own detailed
model, finite elements and mesher stand in for them), no 3D volume meshing, involute gears only,
Swiss lever escapement only, Windows and macOS builds configured but not exercised.
