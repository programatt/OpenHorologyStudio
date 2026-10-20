# Performance enhancement plan

A review of where Open Horology Studio makes the user wait, in the desktop app (`ohs-gui`) and the command line
(`ohs`), with measurements and a plan to remove the waits. The focus is on actions that freeze the window or hold
up the next step: importing, opening a project, rig edits, and the analyses that read geometry. Where computing
something once at import or first open would make later actions faster, that is called out.

## How this was measured

- Machine: 16 hardware threads, AMD Radeon (Renoir) integrated GPU, Ubuntu, Release build (`-O3`) from
  `build/gui` at commit `f293d50`.
- Two movements:
  - **Demo**: `ohs dev gen-movement`, 15 parts, 8.4 MB STEP, 15k display triangles.
  - **OM10**: `om10-00001_20220701_va_01_3.stp`, 166 parts (107 distinct shapes), 18.7 MB STEP, 259k display
    triangles, imported with `--preset data/thirdparty/om10/rig.toml --beat-rate 25200`.
- Wall-clock times of the `ohs` commands; small C++ harnesses linked against the built libraries to time the stages
  inside import, project open and the viewport; the `ohs_viewport_bench` for frame rates; and gdb stack sampling
  for the rig-edit freeze. The harnesses are not in the repository. Section 8 proposes adding the useful ones to
  `ohs_perf`.

## 1. Where the time goes today

### Command line

| Command | Demo | OM10 | What dominates |
| --- | ---: | ---: | --- |
| `ohs import` | 41.4 s | **136.4 s** | feature extraction, one part at a time (126.5 s on OM10) |
| `ohs dfm` | 2.6 s | 12.2 s | STEP read (8.7 s), then sections one part at a time |
| `ohs collisions` | 2.7 s | 13.2 s | STEP read, then sections one part at a time |
| `ohs animate` | 2.7 s | 10.3 s | STEP read and tessellation |
| `ohs fea --case shock` | – | 12.2 s | STEP read (all 166 parts, to use one) |
| `ohs draw` | – | 8.5 s | STEP read (all 166 parts, to draw one) |
| `ohs report` | 9.4 s | crash after 20 s | see 1.4 |
| `ohs verify levels` | – | 2.8 s | the detailed contact model (genuine work) |
| `ohs timekeeping` | 0.41 s | 0.38 s | nine settled runs (already on threads) |
| every rig, kinematics, torque, explain command | 0.06 s | 0.08 s | process start-up, 64 ms of it (see 7) |

Every command that needs geometry reads the whole STEP file again. That is 8.7 s on OM10 before any analysis starts.

### Inside `ohs import` (OM10, 166 parts)

| Stage, summed over all parts | Time | Note |
| --- | ---: | --- |
| STEP read (`import_step`) | 8.7 s | OCCT's STEP translator |
| Hole finding (`find_holes`) | **104.3 s** | a new `BRepClass3d_SolidClassifier` for every vertical cylindrical face |
| Mid-height section (`BRepAlgoAPI_Section`) | 3.8 s | (8.3 s on the demo, where the prism shortcut makes it 0.2 s) |
| Everything else (mass, boxes, outline, teeth, flank fits) | ~9 s | |
| Slowest part, OM10-00138 (1,112 faces) | 42.2 s | nearly all hole finding |

On the demo, hole finding is 28.7 of the 36 s of feature extraction. Each gear's tooth flanks are hundreds of
cylindrical faces, and each one rebuilds a classifier over the whole solid.

### Desktop app (OM10)

| Action | Time the window is blocked | What dominates |
| --- | ---: | --- |
| Open a project (`Scene::load`) | 10.0 s (off the UI thread) | STEP read 8.7 s, tessellation ~1.3 s on one thread |
| Import from the GUI | ~146 s (off the UI thread) | `ohs import` (136 s), then the STEP is **read a second time** for the scene |
| Open the demonstration movement | ~43 s every time | regenerates and re-imports the demo from scratch |
| Any rig edit: role, label, mesh, winding, beat rate, re-rig, declare all, solve, undo, redo | **0.85 to 1.04 s** (UI thread) | `refresh_panels` filling the part table: quadratic, see 3.1 |
| ... of which the command and the rig reload | ~22 ms | `rig` command 11 ms in process, `Scene::reload_rig` 11 ms |
| Click a part (pick on press and on release) | 2 × 15 ms | CPU projection and sort of all 259k triangles |
| One wheel step with a part selected | 13.5 ms | `screen_position` projects and sorts every triangle |
| Orbit or animate, OpenGL | 60 fps (vsync) | the vertex buffer is rebuilt every animation frame, see 4.1 |
| Orbit or animate, software painter | 6.6 fps (205 ms per full-quality frame) | project, sort and fill 259k triangles |
| Hide/show a role, walk the train, select a part | 15 ms, 17 ms, 0.2 ms | fine |

The demo hits 60 fps with both renderers and its rig edits are fast. OM10 is the realistic case, and the problems
grow with the part count and the triangle count.

### Bugs found along the way

- **Parallel tessellation of OM10 crashes** (`double free or corruption`) when every part is meshed on its own thread.
  The 166 parts share 107 underlying shapes (`TShape`): screws, jewels and pins are instances of one shape, so two
  threads write the same triangulation. Meshing only the distinct shapes in parallel works (0.92 s against 3.0 s
  serial). Every parallelisation below must account for this.
- **`ohs report` segfaults on OM10**, and so does `ohs fea bench.hsproj --part OM10-00197:2` run on its own (inside
  `Eigen::internal::minimum_degree_ordering`, from `fea::solve_reduced` in `cases::run_tooth_case`). This depends on
  the input, not on threads. It is not a performance problem, but it blocks the report on OM10 and should be fixed
  first. A daemon (section 7) would make a crash like this cost more.
- `ohs fea --part OM10-00149:1` and `OM10-00146:1` fail with "the pressure angle must be between 0 and 45 degrees":
  the gear fit stored an out-of-range angle.

## 2. Import: make the first step fast (biggest win)

### 2.1 Reuse one solid classifier per part in `find_holes` (`src/geometry/features.cpp`)

`BRepClass3d_SolidClassifier classifier(shape, inside, 1e-7)` builds a new solid explorer for every cylindrical face.
Construct it once per part with `BRepClass3d_SolidClassifier classifier(shape)` and call
`classifier.Perform(inside, 1e-7)` for each probe point.

- Measured: OM10 104.3 s → **5.0 s**, demo 28.7 s → **1.0 s**, with **identical hole lists**.
- Further option: decide most faces from the face orientation (the outward normal of a hole's face points at its
  axis) and classify only the faces where the normal is ambiguous. The normal agreed with the classifier on 754/754
  demo faces but only on 4,402/4,452 OM10 faces, so use it as a pre-filter, never on its own.
- Cheap further cut: a hole must be inside the part's outline, so skip cylinders whose radius is above the bore
  radius found later, or reorder so the outline is sampled first.

### 2.2 Use the prism shortcut for the outline section (`sample_outline`)

`section_region` already recognises straight extrusions (`prism_outline` in `src/geometry/section.cpp`) and takes
the outline from the bottom face. `sample_outline` in `features.cpp` always runs the general boolean
`BRepAlgoAPI_Section`. Share `prism_outline` (move it into `section.hpp`) and sample the prism's wires.

- Measured on the demo: 8.3 s → 0.2 s. On OM10 the parts are mostly not plain prisms, so the gain is small there
  (3.8 → 3.5 s). The barrel's retry over ten heights also benefits.

### 2.3 Extract features on threads (`src/app/import.cpp`)

`app::import_movement` calls `extract_features` one part after another. `ohs::parallel_for` already exists and
keeps the output in part order.

- Measured with today's code: demo 36 s → 8.9 s, OM10 126 s → 11.3 s (bound by the slowest part). Results identical.
- With 2.1 applied the slowest OM10 part drops from 42 s to about 1 s, so import should end up near
  **STEP read + ~3 s, about 12 s for OM10 (from 136 s)** and about 3 s for the demo (from 41 s).
- Thread safety: give each thread distinct `TShape`s. Feature extraction only reads geometry and passed the test,
  but meshing on shared shapes crashed (1.4). Before the parallel loop, copy any part whose `TShape` has already
  been seen (`BRepBuilderAPI_Copy`), or group parts by `TShape` and run the groups on threads. Run the
  `--tsan` preset on the OM10 import before merging.
- Instances of one shape with the same orientation could reuse each other's features (only the centre and the
  holes move with the placement): 166 parts, 107 shapes. This is a later refinement once the cache below exists.

### 2.4 Report progress and allow cancelling

Even at 12 s, the GUI's import shows only an indeterminate bar. Pass a progress callback through `import_movement`
("part 37 of 166") to the status bar, and check a cancellation flag between parts. The command line can print the
same with `-v`.

## 3. Rig edits: stop freezing the window

### 3.1 Fill the part table without the quadratic re-layout (`MainWindow::refresh_panels`)

gdb sampling of repeated label edits on OM10: 36 of 40 samples were in
`refresh_panels → QTableWidget::setItem → dataChanged → QTableView::visualRect → QHeaderView::sectionPosition →
sizeHintForColumn`, laying out the text of every row. The part table's columns 1–3 are `QHeaderView::ResizeToContents`,
so every `setItem` re-measures the whole column. `setUpdatesEnabled(false)` stops the painting but not this
measuring. With 166 rows this costs ~850 ms per edit, and it grows with the square of the part count.

Fix, smallest first:
1. During the rebuild, set every section to `QHeaderView::Interactive` (or `Fixed`), fill the table, restore
   `ResizeToContents` and call `resizeColumnsToContents()` once. The tree's `ResizeToContents` role column gets the
   same treatment.
2. Better: replace `QTableWidget`/`QTreeWidget` with a `QAbstractTableModel` over `Scene::movement()` (and a tree
   model). An edit then becomes `dataChanged` for the rows that changed, or one `beginResetModel/endResetModel`,
   and the filter becomes a `QSortFilterProxyModel` instead of `setRowHidden` on every row.
3. Rebuild only what changed. A label edit changes one row's text, a beat-rate change touches no panel, and a role
   change can move rows between mobiles. `reload_rig` can compare the old and new `Movement` and refresh only the
   affected panels.

Expected: about 880 ms → **under 30 ms** per edit on OM10. That covers every button on the Rig tab, undo/redo, and the
F2 rename in the tree.

### 3.2 Batch edits as one command

A user correcting several roles pays a whole edit cycle each time (CLI run, file write, reload, review, panels).
Allow several assignments in one call (`ohs rig set project A --role wheel B --role pinion`), and give the
GUI's part table multi-select with "Set role" applied to the selection as one undoable edit. The same goes for
labels and for declaring several meshes.

### 3.3 Do not make rig edits wait for queued jobs (`MainWindow::run_command`)

`run_command` calls `wait_for_jobs()` before any rig edit: a busy loop over `processEvents` until every queued
analysis has finished. With a 12 s DFM or a 20 s report queued, a role change blocks for that long. The wait exists
so that a job never reads a half-written project file. Remove the reason instead:
- Write the project atomically: `save_rig` truncates and rewrites in place. Write to `path.tmp` and `rename`
  (`QSaveFile` on the GUI side). A reader then always sees the old or the new file, never half of one. This also
  fixes a latent corruption risk if the process dies mid-save.
- Give each job a snapshot: copy the project file to `<project>.jobs/<n>.hsproj` when the job is submitted and run
  it on that copy. The result then matches the rig the user asked about, even if they edit while it is queued.

With both, rig edits never wait, and the queue keeps running in the background.

### 3.4 Keep the edit path in memory (later)

Each edit is: CLI parses TOML (7 ms) → edits → writes TOML (4 ms) → GUI reads the file again to snapshot it →
`reload_rig` parses it again → `update_review` runs `check_rig`, `train_chain`, `suggest_meshes` and `walk_train`.
That is ~22 ms on OM10 and fine for now. If the panels become fast (3.1), the next step is to run the rigging
function on the scene's own `Movement`, save once, and keep the undo snapshots as `Movement` values instead of
re-reading the file.

## 4. The viewport

### 4.1 Upload the geometry once, move parts with matrices (`viewport_gl.cpp`)

Today the OpenGL path rebuilds and uploads the whole vertex buffer whenever `Scene::revision()` or `look_` changes:
on every animation frame, every timeline drag, every exploded-view or cut-away change, every selection and every
walk step. On OM10 one rebuild is `draw_list` (3.3 ms: transforms all 259k triangles on the CPU), then the
repack into `Vertex` with normals and colours, then a 21 MB upload. It holds 60 fps on this GPU, but it is the CPU
that sets the limit, and it scales linearly with the triangle count.

Instead:
- Upload each part's triangles once, in its own pivot frame (what `ScenePart::triangles` already stores), into one
  static buffer with per-part ranges. Normals once, too.
- Per frame, per part: a model matrix (the mobile's rotation about its pivot, the Rodrigues rotation of an
  articulation as a matrix, the explode lift as a translation) and a colour, as uniforms. 166 small draw calls, or
  one call with a per-part matrix texture.
- Cut-away: a clip plane in the shader (`discard` above the plane), not a CPU filter.
- Selection, walk and rig colours change uniforms only. No re-upload.
- Field overlay: its own buffer, uploaded when the field changes.

Result: an animation frame costs a few hundred uniforms instead of a 21 MB rebuild, and selection becomes free.
Model-scale headroom grows by an order of magnitude.

### 4.2 Picking and wheel zoom without projecting everything (`viewport.cpp`)

`pick()` projects and depth-sorts every triangle (15 ms on OM10), and a click calls it twice (press, to decide turn
or pan, and release). `wheelEvent` calls `screen_position`, which does the same for one part's mean (13.5 ms per
wheel step, on the OpenGL path too). Options:
- Keep the projected list from the press for the release (same view, same revision).
- `screen_position`: work out each part's world-space centroid once per `draw_list` and project one point.
- `pick`: per-part screen bounding boxes first, then test only the triangles of the parts under the cursor, nearest
  first. No full sort is needed to find the front-most hit; a depth comparison per candidate is enough. Under
  OpenGL, an ID-buffer pick (render part indices to an offscreen buffer and read one pixel) is exact and costs about
  one frame.

### 4.3 Software painter for large models (lower priority)

6.6 fps on OM10. Auto picks OpenGL on hardware GPUs, so this mostly matters on machines without one and in
snapshots and video export. Options, in order of value:
- Coarse meshes during interaction: tessellate once more at a larger deflection (0.1) and draw that in `fast_` frames.
  The fine mesh comes back when the view settles. Cache both meshes (5.2).
- Skip triangles smaller than a pixel after projection.
- Project on threads (`parallel_for` over triangle chunks) and use a parallel sort. Painting stays single-threaded.

## 5. Opening a project: compute once at import, reuse afterwards

Opening a project in the GUI and every geometry command start by reading the STEP file (8.7 s on OM10). Read once at
import, the geometry can be kept in forms that load in a fraction of that.

### 5.1 A geometry cache beside the project

At import, write `<project>.cache/` (or `<project>.ohscache`) with:
- `parts.brep`: every imported part's shape in OCCT's binary format (`BinTools::Write` of a compound, names in a
  sidecar list). Measured on OM10: write 0.21 s, **read 0.19–0.22 s against 8.7 s for the STEP (~40×)**; demo 25 ms
  against 2.3 s.
- `manifest.json`: STEP path, size, modification time and a content hash (a hash of the first and last MB plus the
  size is enough to catch a changed file cheaply), the OCCT version, the dial axis, and the schema version of the
  cache.

Add one function, for example `geometry::load_parts(project_path, movement)`, that returns the cached parts when the
manifest matches and otherwise reads the STEP and rewrites the cache. Every geometry user moves to it: `Scene::load`,
`cmd_animate`, `load_parts` in `manufacturing_cli.cpp` (dfm, collisions, draw), `fea --case shock`, `build_report`,
`tolerance --pmi` (PMI still needs the STEP, so keep that path). A command run with `--step <file>` keeps
reading that file.

Effect on OM10: `dfm`, `collisions`, `animate`, `fea --case shock`, `draw` each lose ~8.5 s. GUI open goes from 10 s to
about 1.5 s. The cache is invalidated automatically when the CAD file changes, and `ohs import` overwrites it.

Keep the cache out of version control (`.gitignore` already blocks STEP files; add `*.ohscache`/`*.cache/`), and
make a missing or unreadable cache a silent fallback, never an error.

### 5.2 Cache the display mesh

`Scene::load` tessellates every part at deflection 0.02 (~1.3 s on OM10, serial). Store each part's triangles,
already in the pivot frame `Scene` uses, as flat float arrays in `mesh.bin` (OM10: 259k triangles, 9.3 MB, read in
4 ms). `Scene::load` then needs neither the STEP nor the BRep. **Project open becomes the TOML parse plus a 9 MB
read: well under a second, from 10 s.** The BRep is loaded lazily, only when a field plot or an analysis needs it.
Store the coarse interaction mesh from 4.3 next to it.

When there is no cache, tessellate the distinct `TShape`s on threads: 3.0 → 0.9 s on OM10 (1.4 explains why only the
distinct ones).

### 5.3 Do not read the STEP twice on a GUI import

`load_async` runs `ohs import` (which reads the STEP and extracts features) and then `Scene::load`, which reads the
same STEP again. With 5.1 the import writes the cache, so the second read is 0.2 s. Better still, the import path
builds the display mesh while the shapes are in memory and writes `mesh.bin` at the same time, so the scene loads
straight from it.

### 5.4 Cache the part sections

DFM, collisions and the report each compute every part's mid-height section with `section_region` (OM10: 13.2 s
summed when run one part at a time), each with slightly different tolerances (dfm 0.002/0.002, report 0.001/0.002,
collisions `tol*0.5/tol`), and none shares with the others.
- Agree on one deflection/simplify pair, or key the cache by them.
- Compute the sections on threads wherever they are built: `cmd_dfm`, `collision_parts` and the report's `sections()`
  lambda all loop serially, though the DFM rules after them are already parallel.
- Store them in the cache (`sections.bin`, keyed by part fingerprint, height and tolerances) the first time they are
  needed, or at import as an option (`ohs import --precompute` / a background job the GUI starts after an
  import). Later DFM/collision runs then need neither the STEP nor the BRep.

### 5.5 Ship or cache the demonstration movement

`File > Open the demonstration movement` regenerates the STEP and re-imports it into the temp directory every time
(~43 s now, ~5 s after section 2). The demo is deterministic. Generate it once into the user data directory with
its cache, keyed by the OHS version, and open it from there afterwards, or ship the project and the cache in the
package.

## 6. Analyses and the job queue

### 6.1 One run for the text and the plot

The Timekeeping button queues `timekeeping` and then `--json timekeeping`, and the Timegrapher button queues
`timegrapher` and `--json timegrapher --csv`. Each pair runs the same simulation twice. Run only the JSON job and
format the Results text from the JSON in the GUI, or add `--json-out <file>` so one run prints the text and writes
the JSON. That halves the wait for both buttons.

### 6.2 Load only the part an analysis needs

`fea --case shock` and `draw` read all 166 parts to use one (8.5 s of the 12.2 s and 8.5 s). With the cache this is
moot, but even the STEP path can stop at the named part: the XCAF reader can transfer one label.

### 6.3 Jobs and the queue

Jobs run one at a time as separate `ohs` processes, so a queued DFM, collisions and report run in sequence, each
re-reading geometry. After 5.1 the re-read is cheap. Allowing two jobs at once (or as many as the cores allow,
given that DFM, timekeeping and the report already use threads) helps when a user queues several quick analyses
behind a long one. The journal and result handling already cope with out-of-order completion.

## 7. A resident `ohs` process (daemon) and IPC

You asked whether a daemon version of the CLI, reached over an IPC channel, would help if process start-up is the
main issue. Start-up is a small part of the cost. The daemon's real value is keeping parsed geometry in memory.

**What start-up costs.** `ohs --version` takes 64 ms; the binary loads 143 shared libraries (OCCT, VTK, MFEM,
OpenMPI, HDF5, TBB). A rig command takes 77–86 ms in all, so start-up is ~80 % of a cheap command. That matters
for scripts with hundreds of commands and for the GUI's job queue. It is not why the user waits: the multi-second
waits are the STEP read (8.7 s), feature extraction, tessellation and sections, and every new process repeats all
of them. The GUI already runs rig commands in-process (`cli::run_captured`), so they pay no start-up at all.

**What a daemon adds on top of the disk cache.** With 5.1–5.4 a geometry command loads its inputs in ~0.2–0.5 s
instead of 9 s. A daemon that keeps the `Movement`, the BRep parts, the sections and the train analysis in memory
cuts that to nothing, plus the 64 ms start-up. So it is worth having, but **after** the caches, which deliver about
95 % of the gain without the operational risk.

Design, if it is built:
- `ohs serve` listens on a per-user Unix domain socket (`$XDG_RUNTIME_DIR/ohs.sock`; a named pipe on Windows via
  `QLocalServer`/`QLocalSocket` or Boost.Asio). Requests are line-delimited JSON: `{"args": [...], "cwd": "..."}`;
  replies `{"exit_code", "out", "err"}`, with streamed progress lines for long jobs.
- The `ohs` client tries the socket first and runs in-process when there is no daemon, so nothing breaks without one.
  A thin client binary (`ohs-client`, or a mode of `ohs` that links only the socket code) also avoids loading the
  143 libraries. Otherwise the client still pays the start-up the daemon was meant to remove.
- The daemon keeps a session per project: the parsed rig, parts, display mesh, sections, train analysis, keyed by
  the project file's and the STEP's size/mtime/hash. Rig edits update it in place; a changed file on disk
  invalidates it.
- `run_captured` swaps the global `std::cout`/`std::cerr` buffers and holds a global mutex, so today only one command
  can run at a time in a process. A concurrent daemon needs the commands to write to a passed-in stream. That is a
  mechanical refactor of `cli/*.cpp` (they all print through `std::cout`) and is worth doing anyway, as it also
  frees the Python binding from serialising every call.
- Crash isolation is the main risk. The OM10 FE crash in 1.4 would take the whole warm session down. Run each
  request (or each analysis kind) in a forked worker that inherits the warm state copy-on-write (fork after load,
  on Linux and macOS), and have the client restart the daemon when it disappears. Cancellation, which the job queue
  does today by killing the process, then still works: kill the worker.
- The GUI's `JobQueue` sends jobs to the daemon instead of spawning `ohs`. Its journal, resume after a restart and
  cancellation stay as they are.

Recommendation: do sections 2, 3 and 5 first. Then measure scripted bulk use (for example a lesson check over many
projects, or an OM10 study running hundreds of `whatif` commands). If start-up still shows, add the daemon with the
fork-per-request design above.

## 8. Smaller items

- `Movement::mobile_of` and `find_component` are linear scans, called inside O(n²) loops in `infer_structure`,
  `suggest_meshes`, `check_rig` and the viewport's `draw_rig`, which makes them O(n³). This is cheap at 166 parts
  (`rig auto` 83 ms including start-up), but a component→mobile index rebuilt by `infer_structure` and a name→index
  map make the rigging scale to thousand-part assemblies.
- `from_toml`'s `index_of` is a linear scan per reference, quadratic in parts; use a map.
- `classify` compiles every preset regex again for every part; compile the presets once per run.
- `cmd_dfm`, `collision_parts` and the report find each part's shape by scanning all STEP parts for every component
  (O(n²) string compares); build a name→part map once.
- `MainWindow::select_part` loops the whole tree and table to find the row; cheap now, and free with the model in
  3.1.

## 9. Order of work and the gains to expect

| Step | Changes | OM10 before → after (estimated where not measured) |
| --- | --- | --- |
| 1 | Classifier reuse in `find_holes` (2.1) | import 136 s → ~37 s (holes 104 → 5 s, measured) |
| 2 | Part table without quadratic layout (3.1, option 1) | every rig edit, undo, redo 0.9 s → <0.05 s |
| 3 | Parallel feature extraction with distinct shapes (2.3) and prism outlines (2.2) | import ~37 s → ~12 s |
| 4 | Geometry cache and loader (5.1, 5.3) | open 10 s → ~1.5 s; dfm, collisions, animate, shock, draw each −8.5 s |
| 5 | Display mesh cache and parallel tessellation (5.2) | open → <1 s |
| 6 | Atomic project save and per-job snapshots (3.3) | rig edits no longer wait for queued jobs (could be tens of seconds) |
| 7 | One run for text and plot (6.1), demo cached (5.5) | timekeeping/timegrapher buttons half the time; demo 43 s → <1 s on later opens |
| 8 | Static GPU buffers with per-part matrices (4.1), faster pick and wheel (4.2) | no per-frame 21 MB rebuild; click 30 → ~1 ms; wheel 13.5 → <1 ms |
| 9 | Section cache and parallel sections (5.4), batch edits (3.2), progress (2.4) | dfm/collisions → about the rule time; bulk role fixes in one edit |
| 10 | Daemon with fork-per-request (7), only if measurements still call for it | −64 ms per command, warm geometry |

Fix the `run_tooth_case` crash (1.4) before step 4's benchmark of `ohs report`, so the report can be measured on OM10.

## 10. Keeping it fast

- Add benchmarks to `tests/perf/perf_main.cpp` for what this review found: `extract_features` on the demo's
  largest wheel, `find_holes` alone, `import_movement` on the demo, the cache round trip, `Scene::load` from the
  cache, `draw_list`, and the part-table refresh in the GUI tests (offscreen) for a 500-part synthetic movement. That
  last one would have caught 3.1.
- `tests/perf/baseline.json` has no import or GUI numbers today, so the slow import went unnoticed. Record the new
  ones there.
- Run the `--tsan` preset over the import and open paths whenever a loop gains threads, with OM10 (shared shapes)
  rather than the demo (none). The demo does not reproduce the shared-shape crash.
- Results must stay independent of the thread count, as the existing `parallel_for` users guarantee: write into
  slot *i*, assemble in order.

## 11. Status

Done and measured on the same machine and files as section 1 (OM10 unless noted; `ohs` Release, 16 threads):

| Section | What | Result |
| --- | --- | --- |
| 2.1, 2.2, 2.3 | One classifier per part, prism outlines, features on threads grouped by `TShape` (`geometry/shape_groups.hpp`) | `ohs import` 140 s → 16 s; demo 42 s → 3.7 s. Rig identical (61 mobiles, 10 meshes); fit residuals differ in the sixth decimal. No ThreadSanitizer report on the OM10 import, `dfm` or `collisions` |
| 2.4 | Progress callback and cancel flag through `import_movement`; status bar text and a Cancel button | tested through `cli::set_import_hooks` |
| 3.1 | Part table filled with fixed column widths, sized once at the end | rig edit at 400 parts 4.4 s → 30 ms (a test with a 400-part movement fails at the old speed) |
| 3.2 | `rig set P… --role R`, `--assign PART=ROLE`, `rig label --set`, `rig mesh --pair`; multi-select *Set role* | all or nothing; one undo step |
| 3.3 | `save_rig` and `restore_project` write atomically; jobs read `<stem>.job<N><ext>` copies of the project; no wait in `run_command`; two jobs at once | an edit with a DFM queued returns at once |
| 4.1 | OpenGL: static per-part buffers, model matrix and colour per part, clip plane in the shader, separate field buffer; `Scene::part_poses()` is the single source of poses (also used by `draw_list`) | CPU per animation frame 16 ms → 0.1 ms; GPU 8 ms. Painter/GL parity test on a real display for pose, explode, cut-away, selection and field |
| 4.2 | Pick tests only the parts under the cursor; the wheel projects one centroid | 15 ms → 1.3 ms; 13.5 ms → 0.001 ms |
| 4.3 | Coarse mesh (0.1 mm, 1.2 rad) and no sub-pixel triangles for frames drawn while interacting; projection on threads | interactive frame 52 ms → 27 ms; a full-quality frame is unchanged (about 300 ms, paint-bound) |
| 5.1, 5.2, 5.3 | `<project>.ohscache/` with `parts.brep`, `manifest.txt`, `mesh.bin` (fine and coarse), keyed by the CAD file's size, time and a first/last-megabyte hash; `geometry::load_parts` used by every geometry command and the scene; the import writes it | open 10 s → 0.1 s; `dfm`/`collisions`/`shock`/`draw`/`animate` lose the 8.7 s STEP read; shock case 0.56 s |
| 5.4 | Sections on threads in `dfm`, `collisions` and `report`; name → part maps | `dfm` 12.2 s → 1.5 s, `collisions` 13.2 s → 1.9 s, `report` crash → 4 s; output byte-identical to before |
| 5.5 | The demonstration movement is built once per version in the user data directory, and put back as imported on each open | 43 s → 0.06 s |
| 6.1 | `--json-out` on `timekeeping` and `timegrapher`; one job | one run instead of two |
| 6.3 | `JobQueue::set_max_parallel`; the window uses 2 | tested |
| 8 | `Movement::mobile_index`, name index in `from_toml`, preset regexes compiled once | |
| 10 | `ohs_perf` has extract-features, import, STEP read and cache round trip; baseline recorded; `OHS_BENCH_PROJECT` for the scene bench | |

Bugs: the segmentation fault of `ohs report` and `ohs fea --part OM10-00197:2` was a tooth mesh of 611,712 unknowns
solved before the size limit was checked (the limit now comes first; the part's module of 0.19 is what made it large). The
out-of-range pressure angle (57° and 45.7°) is no longer stored by the gear fit, and readers ignore one in an older
project. The shared-shape tessellation crash cannot occur: the scene's tessellation groups by shape, and a test pins the
grouping.

Added afterwards:

| Section | What | Result |
| --- | --- | --- |
| 5.4 | `sections.bin` in the cache (`geometry::SectionCache`), keyed by the CAD fingerprint, part, axis, height and both tolerances; used by `dfm`, `collisions` and `report` | warm `dfm` 1.65 s → 0.76 s, `collisions` 1.9 s → 1.3 s; output identical |
| 2.3 | `geometry::translate_features`: another placement of a shape that differs by a translation only gets the first one's features, moved (centre, heights, holes, fingerprint) | tested against a full extraction. OM10 has few such pairs (10 of 166), so the import time does not change |
| 3.4 | A role edit in the window runs on the scene's own rig (`Scene::reload_rig(Movement)`), is written once and logged as the `ohs rig set` it stands for | rig edit at 400 parts 27 ms → 9 ms |

Not done, and why:
- **6.2 (read one part from the STEP).** The cache makes it moot, as the plan says: a command that needs one part no longer
  reads the STEP file at all (`fea --case shock` 12.2 s → 0.3 s). The XCAF reader cannot transfer one label of a flat assembly
  more cheaply than the cache loads the lot.
- **7 (the daemon).** The plan says to build it only if start-up still shows after sections 2, 3 and 5. A rig command on
  OM10 takes 128 ms in all (62 ms of it the process start), the window runs rig commands in-process, and `dfm`
  is 0.8 s warm, so start-up is not what a user waits for. Revisit for scripts of hundreds of commands.
