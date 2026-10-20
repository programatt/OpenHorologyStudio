# Changelog

All notable changes to Open Horology Studio. The format follows Keep a Changelog; versions follow
Semantic Versioning, with the Python API and the plugin ABI versioned as described in
`docs/python.md` and `docs/plugins.md`.

## Unreleased

Found by running the plan's gates on the real OM10 file.

### Performance
Measured on the 166-part OM10 file (18.7 MB STEP) and the generated demonstration movement, Release build, 16 threads.
- `ohs import` takes 16 s instead of 140 s on OM10 and 3.7 s instead of 42 s on the demonstration movement. The hole
  search built a solid classifier for every cylindrical face (104 s of the OM10 import); it now builds one per part.
  The outline of a straight extrusion is taken from its bottom face instead of a boolean section, and the features
  of the parts are extracted on threads. Instances of one shape (screws, jewels, pins share one `TShape`) are taken by
  one thread in turn, since OCCT writes into the shapes it works on. The fit residuals of a wheel differ in the
  sixth decimal place, as the outline is sampled from exact edges.
- A geometry cache beside the project (`<project>.ohscache`): the parts as OCCT's binary BRep and the display mesh,
  written by `ohs import` and the first open, and used while the CAD file is unchanged (size, time and a hash of its
  first and last megabyte are checked). A missing or stale cache is never an error. Opening OM10 takes 0.1 s instead
  of 10 s; `ohs dfm`, `collisions`, `animate`, `draw` and `fea --case shock` no longer read the STEP file again
  (8.7 s on OM10). A file given with `--step` never replaces the cache.
- The part sections of `ohs dfm`, `collisions` and `report` are cut on threads: 12 s to 1.5 s, 13 s to 1.9 s, and the
  OM10 report, which crashed before, takes 4 s.
- A rig edit in the desktop application no longer freezes the window: refilling the part table measured every row on
  every insert (4.4 s per edit at 400 parts, 0.9 s at 166; now 30 ms). A rig edit no longer waits for queued jobs:
  the project is saved through a temporary file and a rename, and each job reads a copy of the project made when it
  was queued. Two jobs run at once, so a quick analysis does not wait behind a long one.
- The part sections are cached too (`sections.bin`, keyed by the CAD file, part and tolerances): a second `ohs dfm`
  takes 0.8 s, a second `ohs collisions` 1.3 s. A role edit in the window runs on the scene's own rig (9 ms at 400 parts).
  Another placement of a shape that differs by a translation only gets the first one's features, moved.
- Timekeeping and the timegrapher run once for their text and their plot or screen (`--json-out FILE`), not twice.
- Opening the demonstration movement builds it once per version and opens it from its cache afterwards
  (43 s the first time and every time before, 0.06 s after). The import in the window shows its progress
  ("Importing: part 37 of 166") and a *Cancel* button stops it between parts.
- The OpenGL viewport keeps every part's triangles in a static buffer and moves them with a matrix per part (the turn
  of the train, the swing of a lever, the exploded view) and a clip plane in the shader, instead of rebuilding and
  uploading the whole vertex buffer when anything moves: 16 ms of CPU per frame on OM10 becomes 0.1 ms.
- Picking and the zoom toward the selection no longer project every triangle: a click is 1.3 ms on OM10 (15 ms
  before), the wheel 0.001 ms (13.5 ms). Frames drawn while the view moves use a coarse mesh and skip triangles under a
  pixel: 27 ms instead of 52 ms on OM10 with the painter.
- `Movement::mobile_index`, a name index when reading a rig, and preset patterns compiled once, take the cubic
  scans out of the rigging for large assemblies.

### Added
- `ohs rig set` takes several parts (`ohs rig set P a b c --role wheel`) or `--assign PART=ROLE` repeated, `ohs rig
  label` takes `--set PART=LABEL` and `ohs rig mesh` takes `--pair A=B`, each one edit that writes nothing unless
  all of it applies. The Rig tab's part list takes a multiple selection, and *Set role* gives every selected part the
  role as one undoable edit.
- An OpenGL viewport, chosen in *View, Renderer* (Automatic, OpenGL or Software; the `OHS_RENDERER` environment
  variable overrides it). Automatic uses the graphics card when there is one and Qt's painter when there is not
  (including a software rasteriser such as llvmpipe); OpenGL falls back to the painter when no context can be made.
  The triangles stay in a vertex buffer and the card projects, shades and depth-tests them, so orbiting is several
  times faster (69 to over 500 frames per second on the demonstration movement), and it no longer shows the painter's
  depth-sort errors. The painter is also faster: the drawn triangles are kept until the scene changes. Snapshots and
  video still use the painter. `ohs_viewport_bench` measures both.
- `ohs rig unmesh` and a *Remove mesh* button take away a mesh declared by hand; the gears are fitted again without it.
- A timegrapher screen in the Results panel: the beats of the virtual timegrapher run drawn on a grey case with a
  blue LCD like a generic timegrapher, beside the unchanged text output. `ohs --json timegrapher` also reports the
  beat rate and the lift angle.
- A *Beat rate* box in the Rig tab and `ohs rig beat-rate`, to set the project's beat rate, which a project imported
  from the window did not have; *Solve the train* sends you to it when it is missing.
- `ohs rig solve` and the Rig tab's *Solve the train*: from the escape wheel and the beat rate, name the wheels
  of the seconds, minute and hour hands and find what drives the loose end of the train, working out the teeth of a
  barrel drum whose teeth were not found.
- A search box above the Rig tab's part list, and clearer picking: a prompt in the viewport, a cross cursor, Esc to
  cancel, and the reason in the status bar when a picked mesh cannot be declared.
- The train starts at the barrel. A mesh with a ratchet, click, crown, winding or clutch part (by name or label) is
  the winding work and not a link of the going train, and `ohs rig winding` (the *Not in the train* button in the Rig
  tab) leaves any other mesh out of the train, or puts it back; the choice survives `rig auto`.
- Part labels. `ohs rig label` gives a part a name of your own, stored in the project; the model tree, the part list,
  the status bar and the viewport show it in place of the part name, and hovering shows the part name.
- Seeing and finishing the rig. `ohs rig auto` runs the classification and mesh inference again on an imported
  project (roles you set and meshes you declared are kept; `--reset` forgets them), `ohs rig suggest [--apply]`
  lists, and declares, pairs of toothed parts that probably mesh although the strict test rejected them, and
  `ohs rig check` ends with the train from the barrel to the escape wheel, says where it breaks and warns
  when it does. `ohs rig walk` lists the train mesh by mesh from the barrel and what is missing. `ohs-gui`'s Rig
  tab walks the train one mesh at a time (the viewport shows only the step: the parts walked so far, the two
  gears of the mesh with an arrow from driver to driven, the rest faded), stops at the first gap with the pairs
  that would fill it (declare one, or click the partner in the viewport), and has a To do list, the suggested
  meshes and *Re-run auto-rig* (all undoable); *Show the whole rig instead* colours each mobile and draws every
  mesh, the suggestions and the parts to check. `rigging::re_rig`, `suggest_meshes`, `train_chain`,
  `walk_train` and `describe_chain` are the library calls.
- Four roles for parts that had no type: `stem` (the winding stem), `spring` (click, jumper and setting springs,
  of wire or plate), `lever` (clicks, pawls, jumpers, setting levers, yokes) and `bearing` (brass tubes and
  bushes in plates). They are recognised by name ("winding stem", "click spring", "setting lever", "bush", ...;
  a bare "lever" is now a lever, "pallet lever" still the pallet fork) or, for unnamed CAD parts, by shape and
  by what they touch: a long round rod in the plane of the movement, a short hollow cylinder, a thin
  perforated plate whose far end rests on the teeth of a wheel or a date ring, a strip or bent wire pressing on
  one. Part features now record the oriented bounding box (`axis`, `size`) and the round holes of a part
  (`holes`), and the rig file stores them.
- The stem, levers and springs are animated, in the viewport and in the glTF and USD exports (about an axis of
  their own, which for the stem lies in the plane of the movement): the stem turns with the winding of the
  barrel, a lever goes over each tooth of the wheel it rides on, and a spring bends with the lever it presses.
  `ohs kinematics` lists them, and `ohs explain` says what each does.
- A `date_ring` role for the calendar ring (date wheel): recognised by name ("date wheel", "date ring",
  "quantième", ...) or by internal teeth with none outside, found from the section of the part
  (`PartFeatures::inner_teeth`, `inner_tip_radius`, `inner_root_radius`). It is a mobile of its own even when
  concentric with the train and animates as a stepped ring: one tooth for each turn of the 24-hour wheel (or
  24 h of clock time). The Fusion 360 / SolidWorks add-ins and their docs know the role.
- A beam finite-element model of a flat hairspring (`dynamics/hairspring.hpp`): torque and side force against
  collet angle, geometrically nonlinear; a library model, not yet wired into the oscillator.
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
- `ohs fea` and `ohs report` crashed on OM10 (a segmentation fault in the sparse factorisation) when a part's fitted
  module was far too small: the mesh of 600,000 unknowns was solved before it was checked against the size limit. The
  limit is now checked first and the command reports it.
- A gear fit whose pressure angle was 45 degrees or more (a doubtful module put the pitch circle far from the real one)
  was stored, and `ohs fea` then refused the part ("the pressure angle must be between 0 and 45 degrees"). Such a fit
  is no longer stored, and older projects with one are read as having no angle.
- Meshing the parts of a movement on threads corrupted the shared triangulation of instances of one shape (a double
  free on OM10); parts are now grouped by shape so that no two threads touch one.
- A round part whose teeth are on a rim at one end, with a smooth tube between (a barrel modelled in one piece), is
  now counted: when no teeth are found at mid-height and the part is wider than that outline, other heights are
  tried. The OM10's barrel is found to have 107 teeth.
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
