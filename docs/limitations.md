# Limitations

OHS 1.0 is a complete command-line and Python tool. This page lists what it does not do, so you
know where to be careful.

## Not built yet

- **A basic graphical user interface.** `ohs-gui` ([Desktop application](desktop.md)) has a 3D
  viewport, exploded and cut-away views, an animation timeline, the Explain panel and a button for
  every analysis. It is not the plan's full UI: the viewport is drawn with Qt's painter, not the
  OpenCASCADE OpenGL viewer or VTK, so there are no PBR materials, motion trails, force arrows,
  field plots or mode-shape viewport and no MP4 export; there is one plot (rate against amplitude);
  undo covers edits of the project file (role changes and declared meshes), not views; and a job that was running when the window closed starts
  again from the beginning on the next open (jobs run in worker processes and are journalled, but the
  solvers do not checkpoint). It is built
  only with `-DOHS_BUILD_GUI=ON`, and has been exercised on Linux only (offscreen scripted test).
- **Chrono is optional and covers the escapement only; MFEM is a cross-check, no Netgen.** The detailed
  timekeeping model can run on Project Chrono (`-DOHS_WITH_CHRONO=ON`, `--engine chrono`), but as
  two one-degree-of-freedom shafts with the wheel/pallet contact written as a force law, not as
  rigid bodies with collision geometry, and not for the whole train. The finite-element stack and
  the mesher are OHS's own (ADR 3). MFEM (optional, found at configure time) is linked only to
  re-solve a shock case with its own elements and solver (`ohs fea --mfem`: one material, no ties); it
  does not replace the solver, and Netgen is used only as an out-of-process mesher.
- **3D meshing is optional.** By default parts are meshed from their mid-height section and extruded,
  so features that vary through the thickness are not seen by the mesher or by the DFM rules.
  `ohs fea --case shock --mesher netgen` meshes the B-rep with Netgen (an external Python module,
  `pip install netgen-mesher`), but only the shock case uses it, hole clamping still comes from the
  mid-height section, and OHS has no mesher of its own for general solids.
- **Cycloidal gears** are generated and fitted (one rolling circle for both flanks), but the Lewis
  hand calculation and the tolerance stack-up's contact ratio use involute formulas, and a cycloid
  fit is only tried for flanks the involute fit rejects.
- **Escapements:** the Swiss lever only. The architecture is ready for others.
- **No STEP PMI tolerances:** they come from a TOML table. HDF5 (`ohs fea --hdf5`) holds the
  structural fields only; the timekeeping results are JSON and CSV.
- **IGES and DXF import are plain.** IGES files carry no assembly names, so parts are called `part`,
  `part#2` and so on (rename them with a preset). A DXF is a flat drawing: closed contours are
  extruded into parts, with the thickness from the DXF (group 39) or 1 mm, and 3D DXF entities,
  splines, ellipses and blocks are ignored.
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
  Everything outside the gear train is treated as stationary, so the pallet fork, which really
  swings, overlaps the escape wheel at rest: read those pairs as the escapement, not as a clash.
- The dial axis is found from part shapes (the axis along which parts are thinnest). A movement
  whose parts are mostly pins and shafts can fool it: pass `--up`.
- A smooth barrel drum has no teeth to find, so its mesh with the centre pinion cannot be inferred:
  declare it (`ohs rig mesh`, or a `[[mesh]]` rule in a preset).
- The reduced timekeeping model treats the lever and roller as an ideal kinematic coupling with
  an efficiency, a drop and an unlocking friction; it does not model banking, safety action or
  shock.
- Tooth loads use the torque of the mobile at the pitch radius and a tip load, which is the
  conservative single-tooth case.
- Linear tetrahedra are available but lock in bending; use the default quadratic ones for
  stresses.
