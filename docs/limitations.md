# Limitations

OHS 1.0 is a complete command-line and Python tool. This page lists what it does not do, so you
know where to be careful.

## Not built yet

- **No graphical user interface.** The plan's Qt/OCCT/VTK desktop UI (viewer, exploded and section
  views, live animation timeline, Explain panel, plots, job queue) is not part of 1.0. Animation
  exports to glTF, structural results to VTU for ParaView, reports to HTML and PDF.
- **No Chrono, MFEM or Netgen.** The detailed timekeeping model, the finite-element stack and the
  mesher are OHS's own (ADR 2 and 3). The detailed model is an independent cross-check of the
  reduced one, not a full multibody contact simulation of every mobile. An adapter for the real
  libraries can sit behind the same interfaces later.
- **No general 3D meshing.** Parts are meshed from their mid-height section and extruded; features
  that vary through the thickness are not seen by the mesher or by the DFM rules.
- **No cycloidal gear profiles.** Gears are fitted and generated as involutes.
- **Escapements:** the Swiss lever only. The architecture is ready for others.
- **No HDF5 results**, no USD export, no IGES/DXF import, no STEP PMI tolerances: results are JSON,
  CSV and VTU; tolerances come from a TOML table.
- **No mainspring coil contact or hairspring coil model.** The mainspring is a torque curve and the
  hairspring a stiffness from `E b h^3 / 12 L`.
- **Windows and macOS** builds and installers are configured (CMake presets, CPack, CI workflow)
  but were not exercised in the environment that produced 1.0; the Linux build is the tested one.

## Defaults are not your movement

The calibre parameters, tolerance table, DFM limits and material values that ship are typical
figures. A result is only as good as the numbers you give it: calibrate against your timegrapher
(`ohs bench calibrate`), measure your machines, and replace the defaults.

## Known modelling limits

- Collisions are swept in 2D from the assembled pose, over one tooth pitch or one revolution.
- The reduced timekeeping model treats the lever and roller as an ideal kinematic coupling with
  an efficiency, a drop and an unlocking friction; it does not model banking, safety action or
  shock.
- Tooth loads use the torque of the mobile at the pitch radius and a tip load, which is the
  conservative single-tooth case.
- Linear tetrahedra are available but lock in bending; use the default quadratic ones for
  stresses.
