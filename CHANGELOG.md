# Changelog

All notable changes to Open Horology Studio. The format follows Keep a Changelog; versions follow
Semantic Versioning, with the Python API and the plugin ABI versioned as described in
`docs/python.md` and `docs/plugins.md`.

## Unreleased

Found by running the plan's gates on the real OM10 file.

### Added
- `ohs fea --solver mfem`: MFEM solves the shock case's linear system; natural frequencies run on Netgen meshes.
- `ohs tolerance --pmi` reads the dimensional tolerances of a STEP AP242 file into the tolerance table.
- `cmake/superbuild`: Chrono and MFEM fetched and built at pinned commits (run on Linux, GCC 13); OHS finds a
  serial MFEM.
- `ohs fea --vtu-compressed` writes a binary, zlib-compressed VTU through VTK (optional `libvtk9-dev`); the
  text VTU files are now checked by opening them with VTK's own reader.
- A spring detent (chronometer) escapement in the reduced timekeeping model (`escapement = "detent"` and
  `passing_spring_ratio` in a calibre): one impulse per oscillation, a passing spring on the return
  swing, a timegrapher that hears one tick per oscillation. Checked against an energy balance per
  oscillation; the detailed and Chrono models refuse it.
- `ohs-gui` viewport: a colour field plot of a shock result on its part (von Mises or displacement, with a
  scale), tooth force arrows at the meshes of the train, and motion trails.
- `fea::read_vtu`, the reader for the VTU files `ohs fea --vtu` writes.
- `ohs-gui` exports the animation as a video with ffmpeg (File menu, or `--video out.mp4` on the command line).
- `ohs fea --case shock --mesher netgen` meshes the whole B-rep with Netgen (out of process, from
  `pip install netgen-mesher`), so features through the thickness are kept; `section` stays the default.
- `ohs fea --mfem`: the shock case re-solved with MFEM (optional `libmfem`, linked serially) as an
  independent check; it agrees with OHS to about 1e-8 on the test blocks and 1e-6 on a real part.
- `ohs-gui` runs analyses as worker processes (the `ohs` program) in a journalled queue: a running job
  can be cancelled by killing its worker, a crash cannot take the window down, jobs unfinished at
  exit run again when the project is opened, and finished jobs keep their output.
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
