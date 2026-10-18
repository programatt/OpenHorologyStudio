# Changelog

All notable changes to Open Horology Studio. The format follows Keep a Changelog; versions follow
Semantic Versioning, with the Python API and the plugin ABI versioned as described in
`docs/python.md` and `docs/plugins.md`.

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
See `docs/limitations.md`: only a basic Qt desktop application (`ohs-gui`, optional), no Chrono/MFEM/Netgen (OHS's own detailed
model, finite elements and mesher stand in for them), no 3D volume meshing, involute gears only,
Swiss lever escapement only, Windows and macOS builds configured but not exercised.
