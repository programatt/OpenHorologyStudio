# Limitations

OHS 1.0 is a complete command-line and Python tool. This page lists what it does not do, so you
know where to be careful.

## Not built yet

- **A basic graphical user interface.** `ohs-gui` ([Desktop application](desktop.md)) has a 3D
  viewport, exploded and cut-away views, an animation timeline, the Explain panel and a button for
  every analysis. It is not the plan's full UI: the viewport is drawn with Qt's painter, not the
  OpenCASCADE OpenGL viewer or VTK, so there are no PBR materials, and the field plot covers the
  shock case only, with no mode-shape display; MP4 export needs ffmpeg; there is one chart (rate against amplitude);
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
  is an option for the shock case (`--solver mfem`) and a cross-check, not the default, Netgen is used only as an out-of-process mesher, and VTK is linked
  only for its file readers and writers (`--vtu-compressed`), not for display.
- **3D meshing is optional.** By default parts are meshed from their mid-height section and extruded,
  so features that vary through the thickness are not seen by the mesher or by the DFM rules.
  `ohs fea --case shock --mesher netgen` meshes the B-rep with Netgen (an external Python module,
  `pip install netgen-mesher`), but only the shock case uses it, hole clamping still comes from the
  mid-height section, and OHS has no mesher of its own for general solids.
- **Cycloidal gears** are generated and fitted (one rolling circle for both flanks), but the Lewis
  hand calculation and the tolerance stack-up's contact ratio use involute formulas, and a cycloid
  fit is only tried for flanks the involute fit rejects.
- **Escapements:** the Swiss lever and, in the reduced model only, a spring detent (chronometer) escapement
  described by calibre parameters. There is no co-axial, Robin, cylinder, verge or other escapement, no
  contact model of the detent, and the rig does not recognise escapement geometry in CAD.
- **STEP PMI tolerances** are read for diameters and lengths only (`ohs tolerance --pmi`); it was tested on files
  OpenCASCADE itself writes, not on files from a CAD system. Geometric tolerances (flatness, position) and
  datum references are not read; they come from the TOML table. HDF5 (`ohs fea --hdf5`) holds the
  structural fields only; the timekeeping results are JSON and CSV.
- **IGES and DXF import are plain.** IGES files carry no assembly names, so parts are called `part`,
  `part#2` and so on (rename them with a preset). A DXF is a flat drawing: closed contours are
  extruded into parts, with the thickness from the DXF (group 39) or 1 mm, and 3D DXF entities,
  splines, ellipses and blocks are ignored.
- **No automatic winding.** The rotor (oscillating weight) is recognised and given the `rotor` role,
  by name or because it spans the movement with its mass to one side, but it is not animated and the
  winding chain is not simulated. The geometric guess can mistake a wide bridge over one side of the
  movement for a rotor; it is marked with confidence 0.5, so check it in the rig list.
- **The date ring is indexed, not simulated.** The calendar ring (`date_ring` role) is always a mobile of its
  own and steps one tooth for each turn of the wheel that turns once in 24 h (found from the train's speeds,
  within 3 %), or for each 24 h of clock time with a warning when there is none. Its internal teeth are
  counted from the part (31 is assumed when none are found). The step is a fixed 15 % of the day (3.6 h)
  eased in and out; the finger, the jumper and their contact are not modelled, so there is no jumper
  overshoot, no instantaneous-change mechanism and no date-change torque. A part named "date wheel jumper",
  "finger" or "driving ..." is not taken for the ring. It has no pivots, so the tolerance study leaves it out.
- **The stem, levers, springs and bearings are posed, not simulated.** Four roles cover the small parts of
  the keyless work and the date: `stem` (a long round rod lying in the plane of the movement), `lever`
  (a click, pawl, jumper, setting lever or yoke), `spring` (a click, jumper or setting spring, bent wire or
  cut from plate) and `bearing` (a brass tube or bush in a plate). Names say which, as in `winding stem` or
  `click spring`; an unnamed part is guessed from its shape (confidence 0.4 to 0.6), so check them in the rig list.
  - The stem turns about its own axis, 6 turns for each turn of the barrel and the other way, as it would
    wind it. The ratio is typical, not read from the model, and nothing models the clutch, the winding
    wheels, pulling the stem out or setting the hands.
  - A lever swings about the hole that puts its far end on the teeth of a wheel, a ratchet or a date ring it
    overlaps in height (the pivot and the far end are estimated from the part's hole and bounding box), and
    goes over one tooth at a time: up the face over 70 % of the pitch, back down over the rest, through the
    tooth depth over its length. A spring is fixed at one end and its other end follows the lever it presses
    on (or the teeth, if it presses on them) as a rigid link turned about the fixed end, not a bent beam, so it
    has no stiffness, force or fatigue. A lever or spring that rests on no toothed part, or whose pivot the
    estimate gets wrong, is drawn at rest. The pawls of the automatic winding do not move, as the rotor
    does not. The ratchet of a running movement does not turn (the click holds it), but a ratchet on the
    barrel mobile is drawn turning with it, so its click rides the teeth as if the movement were being wound.
  - A bearing is fixed. It is told from a jewel by name or by being a tube longer than it is wide, not a
    thin washer, and the tolerance study does not count it as a pivot hole.
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
