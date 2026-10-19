# Open Horology Studio (OHS) – Watch Movement Design & Analysis Software Plan

Oct 17, 2026 · @Matt

## Summary

Build **Open Horology Studio (OHS)** (CLI `ohs`) as a C++20 application on Open CASCADE, Project Chrono and MFEM, with a Qt 6 desktop UI, a scriptable CLI and Python bindings, released under Apache-2.0. Designers keep modelling in Fusion 360, SolidWorks or any CAD tool, export STEP AP242, and OHS turns that geometry into a *working movement*: it recognises wheels, pinions and the escapement, infers gear meshes and pivots, then animates, simulates timekeeping, runs FEA and checks manufacturability.

**Goals**

- Import CAD assemblies (STEP, IGES, BREP, STL, glTF) and "rig" them into a horological model with minimal manual work.
- Animate the going train, keyless works and escapement from that model, in real time.
- Simulate timekeeping: torque chain, balance amplitude, rate, isochronism, beat error, power reserve, positional error.
- Run structural analysis (static, modal, nonlinear springs, shock) and manufacturing analysis (tolerance stack-up, DFM rules, collision sweep).
- Same capabilities from GUI, CLI and Python; reproducible, scriptable jobs.
- One-command source build on Linux, macOS and Windows from a `main` checkout, with a full automated test suite.
- Fetch and analyse the openmovement OM10 without ever committing its files to the repository.

**Non-goals for 1.0**

- Being a full parametric CAD system. Light editing only (parameters of generated gears, simple feature tweaks); heavy modelling stays in your CAD tool.
- Reading proprietary native files (.f3d, .sldprt, Parasolid .x\_t). These need vendor SDKs; STEP AP242 is the interchange path.
- CAM/toolpath generation.

## Technology stack and licensing

C++20 is the core language because every serious open-source CAD kernel, multibody engine and FEA library is C++; any other language would spend its life in FFI glue. Rust was considered for safety, but OCCT, Chrono and MFEM have no mature Rust bindings. Python is the scripting and automation layer, not the compute layer.

| Concern | Choice | License | How it is used |
| --- | --- | --- | --- |
| Core language | C++20 (selected C++23 features where GCC 13, Clang 17, MSVC 17.10 and Apple Clang 16 all agree, e.g. `std::expected`) | n/a | All engines and apps |
| CAD kernel, STEP/IGES/glTF I/O | [Open CASCADE Technology](https://en.wikipedia.org/wiki/Open_Cascade_Technology) 8.0.x | LGPL-2.1-only + OCCT exception | B-rep geometry, data exchange, interactive viewer, tessellation |
| Multibody dynamics with contact | [Project Chrono](https://pypi.org/project/projectchrono/) 10.x | BSD-3 | High-fidelity train and escapement verification, shock transients |
| Finite elements | [MFEM](https://github.com/mfem/mfem/) 4.10 | BSD-3 | Linear/nonlinear elasticity, modal, thermal; optional CUDA/HIP |
| Meshing | Netgen (via OCC integration) | LGPL-2.1 | Tet/curved high-order meshes from B-rep |
| Linear algebra | Eigen, hypre | MPL-2.0, Apache-2.0/MIT | Small dense math; parallel AMG and eigensolvers |
| Parallelism | oneTBB; optional MPI | Apache-2.0 | Task graph for jobs, parameter sweeps, Monte Carlo |
| Desktop UI | Qt 6 (Widgets + Qt Quick panels) | LGPL-3.0 (dynamic link) | Main window, docking, property editors |
| Results visualisation | VTK 9 | BSD-3 | Contour plots, deformed shapes, mode shapes |
| Storage | HDF5 + JSON (schema-versioned) | BSD-style, MIT | Project and results files |
| CLI / logging / JSON | CLI11, spdlog, fmt, nlohmann/json | BSD/MIT | CLI parsing, structured logs |
| Python bindings | nanobind | BSD-3 | `import ohs` scripting and plugins |
| Testing | Catch2 v3, RapidCheck, Google Benchmark, Qt Test | BSL-1.0, BSD, Apache-2.0, LGPL | See Testing section |

**Project license: Apache-2.0.** It is compatible with every dependency above when LGPL libraries (OCCT, Netgen, Qt) are dynamically linked, which the build does by default. Apache-2.0 also gives contributors and users an explicit patent grant.

**GPL tools stay out of process.** [Gmsh is GPL-2-or-later](https://gitlab.onelab.info/lrp/gmsh/-/commit/d4b25ed90cd6d5900c06a05e30960fc44a7e499d) and CalculiX is GPL-2; linking either would force the whole binary to GPL. Both are supported only as optional external executables behind an adapter that writes input files and reads results, exactly as the [Gmsh maintainers describe for non-linked use](https://onelab.info/pipermail/gmsh/2019/013348.html). CalculiX doubles as an independent cross-check for MFEM results.

Compliance is automated: every source file carries an SPDX header, CI runs `reuse lint`, and each release ships an SPDX SBOM plus the OCCT attribution notice its exception requires.

## System architecture

The system is layered so that GUI, CLI and Python are interchangeable front ends over one set of application services, and every engine reads and writes the same horological model.

&#91;embedded content: system architecture · 5 layers, 6 engines\]

Engines never call each other directly; they exchange data through the model and HDF5 results, which keeps each one testable alone and lets a third-party plugin (a new DFM rule set or an external solver) slot in through the plugin ABI. Heavy jobs run in separate worker processes managed by the job queue, so a solver crash cannot take down the UI.

## Horological domain model

The domain model is what separates this from a generic CAE tool: geometry is just geometry until it is *rigged* into horological parts and the joints between them. Every engine (animation, dynamics, FEA, DFM) reads the same model.

**Core entities**

- **Movement**: the root; holds frames (main plate as ground), calibre metadata (beat rate, nominal power reserve, hand positions) and the rig.
- **Component** = B-rep shape + role + material + mass properties. Roles: plate, bridge, barrel, arbor, mainspring, wheel, pinion, escape wheel, pallet fork, balance, hairspring, roller, jewel, screw, keyless part, motion-works part, date part, rotor.
- **Mobile**: components that rotate together (e.g. third wheel + third pinion + arbor), each with one rotation axis.
- **Joints**: pivot-in-jewel (radial and end shake), gear mesh (profile type, module, centre distance, backlash), escapement contacts (locking, impulse, guard pin, banking), spring (hairspring collet/stud, mainspring arbor/bridle), click/ratchet.
- **Load cases and drive**: mainspring torque curve, crown input, gravity orientation (dial up/down, crown positions), shocks.

**Rigging an imported assembly**

1. **Classify** each solid. Heuristics: periodic radial features → toothed part (tooth count from the period), spiral → hairspring or mainspring, small cylinders with central hole and ruby-like naming or material → jewels. Names and colours from STEP AP242 are used as hints.
2. **Group mobiles** by coaxiality and contact/press fits.
3. **Infer joints**: coaxial bore in a plate or bridge → pivot; two toothed parts whose pitch circles are tangent within tolerance → mesh; lever geometry adjacent to escape wheel and roller → escapement.
4. **Fit gear parameters**: recover module, addendum, and whether the profile is horological cycloidal (per the NIHS 20-25 practice) or involute, by fitting the extracted tooth flank.
5. **Review**: the Rig panel lists every inference with a confidence score; the user confirms or corrects, and the result is saved as a small rig file next to the geometry.

The rig file (TOML) is the stable, human-readable contract. It can be written by hand, generated by a Fusion 360 or SolidWorks add-in, or produced by the classifier.

**Materials library** ships with typical horological materials (leaded brass, hardened carbon steel, Ni-Fe hairspring alloys, Co-Ni mainspring alloys, synthetic ruby, monocrystalline silicon). Property values carry a cited source and temperature range, and users can override any value per project.

## Simulation

Simulation runs at three fidelity levels so that a designer gets answers in milliseconds while iterating and high-fidelity verification overnight. Each level is validated against the one above it.

| Level | What it models | Engine | Typical use |
| --- | --- | --- | --- |
| 1. Kinematic | Rigid gear ratios, hand rates, escapement geometry (lift angles, drop, lock) | In-house, closed form | Real-time animation; ratio and beat-rate checks |
| 2. Reduced-order dynamics | Balance + hairspring oscillator, lever and escape wheel as event-driven impacts; torque chain with per-mesh efficiency | In-house ODE with event detection (root finding on contact functions) | Amplitude, rate, isochronism, power reserve, sweeps of thousands of designs |
| 3. Full multibody with contact | All mobiles as rigid bodies, tooth-level contact, friction, pivot clearances | Project Chrono (SMC/NSC contact) | Verifying level 2, wear-prone contacts, shock tests |

**Outputs and the questions they answer**

- **Torque chain**: mainspring torque vs. turns → torque at each mobile → torque at the escape wheel, including mesh and pivot losses. Gives power reserve and where energy is lost.
- **Amplitude vs. state of wind**: balance amplitude from fully wound to let down.
- **Isochronism**: rate (s/day) vs. amplitude, from hairspring geometry, escapement disturbance and poise errors.
- **Positional error**: rate in dial up/down and the four crown positions, from balance unbalance under gravity.
- **Virtual timegrapher**: simulated acoustic events (unlock, impulse, drop) rendered as the familiar trace, with beat error and amplitude readouts, so results compare directly with a real timegrapher on a physical movement.
- **Shock**: drop and lateral shocks as transients in Chrono, reporting peak pivot loads, banking contact and whether the escapement trips.

**Numerics**

- Escapement contacts are stiff and short relative to the beat period. Level 2 uses event location instead of tiny fixed steps: integrate smoothly between events, locate contact onset by root finding, apply impulse or a short compliant phase, continue. This keeps long runs (hours of simulated time for power-reserve curves) tractable.
- Level 3 uses Chrono's smooth contact with sub-millisecond steps only around events, run on a TBB thread pool for parameter sweeps.
- All simulations are deterministic for a given seed and thread count; results are written to HDF5 with the full input hash for traceability.

**Animation** is driven by the same solvers. The UI plays kinematic motion live and can replay recorded level 2 or 3 runs with a time-scale slider (e.g. one beat stretched across ten seconds), with contact forces drawn as vectors.

## Analysis: FEA, tolerance and manufacturing

Analyses are set up from the rig, not by hand: the tool already knows which faces are pivots, which teeth mesh and what torque flows through them, so it generates boundary conditions and loads automatically. Users can still edit any load case.

**Structural FEA (MFEM, meshed with Netgen)**

| Analysis | Typical target | Notes |
| --- | --- | --- |
| Linear static | Tooth root bending, arbor and pivot stress, bridge deflection under end-shake loads | Loads from the torque chain; compared against Lewis/Hertz hand calcs in reports |
| Modal | Natural frequencies of bridges, balance, rotor | Flags modes near beat frequency harmonics |
| Geometric nonlinear | Mainspring winding torque curve, hairspring stiffness and centre-of-gravity shift | Large rotation, small strain; mainspring torque curve feeds back into the simulation |
| Contact (frictionless/penalty) | Press-fit of jewels and wheels on arbors, tooth contact stress | Analytical Hertz used as default; FE contact for detailed checks |
| Transient | Shock (drop test) on balance staff pivots and shock-absorber settings | Uses Chrono for rigid motion, MFEM submodel for local stress |
| Thermal / thermoelastic | Rate change from temperature through hairspring and balance expansion | Material-property driven |

High-order curved elements (MFEM supports arbitrary order) matter here: tiny fillets at tooth roots are where stresses peak, and high-order elements resolve them without millions of linear tets. Solvers run on CPU by default, with an optional CUDA/HIP build for large meshes.

**Tolerance analysis**

- Monte Carlo stack-up of centre distances, pivot diameters, jewel holes and plate flatness, using tolerances read from STEP AP242 PMI where present, or from a project tolerance table.
- Outputs: distribution of mesh depth (engagement), backlash, end shake and side shake per mobile, with the share of builds that bind or under-engage.
- Sensitivity ranking: which dimension drives each failure, so tolerances can be tightened only where needed.

**Design-for-manufacturing rules**

Rules are a plug-in set, each with a severity and a fix hint, configurable per process:

- **Milling**: minimum internal corner radius vs. smallest available cutter, depth-to-tool-diameter ratio, minimum wall thickness, undercuts.
- **Wheel cutting**: tooth form achievable with a standard horological cutter or hob; minimum tooth count for the profile; pinion leaf count rules.
- **Wire EDM / LIGA / silicon DRIE (outsourced parts)**: 2.5D extrusion check, wire radius in internal corners, aspect ratio.
- **Jewelling**: press-fit interference from hole and jewel diameters, oil-sink presence, olive vs. flat hole.
- **Assembly**: screw access, part orientation keys, collision-free assembly order.

**Motion-aware checks** sweep the kinematic animation and report any clearance below a threshold or any collision across a full cycle of every mobile, including the date mechanism and keyless works in each crown position.

**Reports**: every analysis produces an HTML report (with PDF export) containing inputs, mesh statistics, convergence history, key results, hand-calc cross-checks and pass/fail against configurable criteria.

## CAD import/export and the Fusion 360 / SolidWorks workflow

STEP AP242 is the primary exchange format: it carries assembly structure, part names, colours and (when the exporter supports it) PMI tolerances, all of which the rigging step uses.

| Format | Import | Export | Purpose |
| --- | --- | --- | --- |
| STEP AP203 / AP214 / AP242 (.stp, .step) | Yes | Yes | Primary exchange with all major CAD tools; round-trips edited gears |
| IGES (.igs) | Yes | Yes | Legacy CAD |
| OCCT BREP (.brep) | Yes | Yes | Lossless internal cache |
| STL / OBJ / 3MF | Yes (mesh only) | Yes | Prototyping, 3D printing large-scale models, mesh-only parts |
| glTF 2.0 (.glb) | Yes | Yes | Web viewers, Blender, animation export with per-mobile keyframes |
| USD (.usdz/.usdc) | No | Yes (1.x) | High-end rendering pipelines |
| DXF (2D) | Yes | Yes | Plate layouts, wheel profiles for EDM/laser; small in-house writer to avoid GPL libraries |
| VTU / VTK, CSV, JSON | No | Yes | FEA results to ParaView; tabular results to spreadsheets |
| CalculiX .inp, Gmsh .msh | Yes | Yes | External solver/mesher adapters |
| Rig file (.toml) | Yes | Yes | Horological roles, joints, materials |

**Recommended workflow**

1. Model the movement in Fusion 360, SolidWorks, FreeCAD, Onshape or any CAD tool, keeping each part a separate body with a meaningful name.
2. Export the assembly as STEP AP242.
3. `ohs import movement.step` (or File → Import) builds the model and runs auto-rigging.
4. Confirm the rig in the UI; save the project.
5. After a CAD change, re-export and run **Update geometry**: parts are matched by name and shape fingerprint, so the rig, load cases and settings survive the change.

**Optional CAD add-ins** live in an `integrations/` folder, built and versioned separately: a Fusion 360 Python add-in and a SolidWorks C# add-in that export STEP plus a rig file written from the CAD tool's own joints, gear features and custom properties. They save the classification step entirely for users who adopt them.

**Gear generator.** Because gear profiles are central, the tool can generate exact cycloidal and involute wheels and pinions from parameters and export them as STEP or DXF back to the CAD tool, closing the loop.

## UI, rendering, animation and CLI

The GUI and CLI are thin shells over the same application-service layer; anything the GUI can do is a command that the CLI and Python can also run. Every GUI action is recorded as a command, which gives undo/redo and lets a session be saved as a replayable script.

**Desktop UI (Qt 6)**

- **Workspaces**: Import & Rig, Animate, Simulate, Analyse (FEA), Manufacture (tolerance/DFM), Report.
- **3D viewport**: OCCT's AIS viewer (OpenGL) for B-rep display, picking, sectioning, exploded views and PBR materials; VTK viewport for FEA fields and mode shapes.
- **Panels**: model tree (mobiles, joints), property editor, rig confidence list, job queue with progress and cancel, plots (rate vs. amplitude, torque chain, timegrapher), log console.
- **Animation**: timeline with play/scrub, time scaling, per-mobile visibility, motion trails, contact-force arrows, cut-away planes; export to MP4 (via platform encoders, so no FFmpeg licence issues are bundled) or glTF.
- **Rendering**: real-time PBR in the viewport; offline still renders with OCCT's built-in path tracer; for showcase renders, one-click export to Blender (glTF/USD) with materials preset for brass, steel, gold plating, ruby and perlage/Côtes de Genève textures.
- Long jobs run in worker processes, so a crashing solver never takes down the UI; the job queue survives restarts.
- Accessibility and HiDPI: keyboard-driven commands, scalable UI, light/dark themes.

**Learning and what-if studies**

The tool is built for someone learning watchmaking by reasoning rather than trial and error, without hiding the engineering underneath.

- **Explain panel**: select any part to see what it does, which joints it has, and the governing relationships (e.g. how hairspring stiffness and balance inertia set the frequency), with the actual numbers from your model plugged in.
- **What-if studies**: duplicate the baseline, change one thing (hairspring length, mainspring, escape wheel tooth count, a bridge's thickness), and get a side-by-side comparison of rate, amplitude, isochronism, power reserve, stresses and manufacturability, with the cause of each change explained.
- **Guided lessons** on OM10: step-by-step tutorials that run real analyses, from reading the gear train to designing and checking a modified part.
- **Two depths everywhere**: a simple view with sensible defaults and plain-language results, and an expert view exposing every solver setting, mesh and raw result.
- **Bench loop**: enter timegrapher readings or measured dimensions from your own parts, and the tool calibrates its model and shows where reality and prediction differ.

**CLI (`ohs`)**

```
ohs import movement.step -o movement.hsproj --auto-rig
ohs rig check movement.hsproj
ohs simulate movement.hsproj --level reduced --duration 48h --out run.h5
ohs fea movement.hsproj --case tooth-bending --order 2 --out fea.h5
ohs dfm movement.hsproj --process swiss-milling
ohs tolerance movement.hsproj --samples 10000 --seed 42
ohs report movement.hsproj --format html,pdf
ohs render movement.hsproj --view exploded --out hero.png
ohs export movement.hsproj --format gltf --animate 1beat
ohs job run study.toml            # batch study / parameter sweep
ohs thirdparty om10 fetch         # see OM10 section
```

- Every command supports `--json` for machine-readable output and returns meaningful exit codes, so it slots into CI or notebooks.
- Study files (TOML) describe parameter sweeps, e.g. varying hairspring length or escape wheel tooth count, and run in parallel.
- `ohs-py`: the same API from Python (`import ohs`), suitable for Jupyter and optimisation loops.

## OpenMovement OM10: fetch and import

The OM10 files cannot be fetched anonymously: openmovement serves them from a [download page that requires a free user account](https://openmovement.org/), and its [General Conditions](https://openmovement.org/general-conditions/) license all published plans under CC BY-SA 3.0. So the repository ships only the tooling and metadata, and each user pulls the data with their own account.

**What is known about OM10** (validation targets): hand-wound calibre, optional automatic module, 3.5 Hz (25,200 vph), about 50 hours power reserve, small seconds at 9 o'clock, provision for a date, Swiss lever escapement, published as STEP ([Worn & Wound](https://wornandwound.com/introducing-the-openmovement-om10-the-first-open-source-watch-movement/); [Hackaday](https://lemmy.world/comment/17467611)).

**The `fetch_om10` script** (`scripts/thirdparty/fetch_om10.py`, also exposed as `ohs thirdparty om10 fetch`)

1. Shows the CC BY-SA 3.0 terms and the openmovement attribution and marking requirements, and asks the user to accept.
2. Gets the single STEP file openmovement publishes (currently `om10-00001_20220701_va_01_3.stp`, no archive) in one of two ways:
   - `--file ~/Downloads/om10-00001_20220701_va_01_3.stp`: the user downloads it manually after logging in. This is the default and documented path, because it never breaks when the website changes.
   - `--login`: signs in with credentials from `OM_USERNAME`/`OM_PASSWORD` or an interactive prompt (never stored) and downloads the current `.stp`. Treated as best effort; ask openmovement for a stable download URL or written OK before relying on it.
3. Verifies the file against SHA-256 hashes in `data/thirdparty/om10/manifest.toml`, which records each known file name and hash. A new file name or unknown hash (a future revision) is accepted with a warning and recorded locally as a new revision.
4. Copies the file unchanged into the per-user data directory (e.g. `~/.local/share/ohs/thirdparty/om10/<file stem>/`, `%LOCALAPPDATA%` on Windows, `~/Library/Application Support` on macOS), never inside the source tree. `.gitignore` also blocks `*.stp`/`*.step` outside `tests/generated` as a second guard.
5. Runs `ohs import` on that one STEP file: the importer reads the whole movement as one assembly, keeping the product tree, part names and colours where the file carries them, applies the OM10 rig preset, then runs `ohs rig check` and prints where the project was written. If parts arrive unnamed or flattened, auto-classification fills the gaps and flags them for review.

**OM10 rig preset** (`data/thirdparty/om10/rig.toml`) maps OM10 part names to roles and joints. It holds only names and role assignments, no geometry or dimensions; tooth counts, centre distances and the rest are derived from the user's downloaded files at import time. That keeps the Apache-2.0 repository clean of CC BY-SA material.

**Built-in OM10 checks** once imported:

- Kinematics: train ratio gives 25,200 vph at the escapement and one revolution per minute at the small-seconds wheel.
- Torque chain and power reserve vs. the published \~50 h.
- Amplitude, isochronism and positional error from the reduced model, verified with Chrono.
- FEA tooth-bending and pivot checks, DFM report for the published part set.
- If a physical OM10 kit is available, its timegrapher readings can be loaded to calibrate friction parameters.

**Licensing of outputs.** Renders, exported meshes or modified geometry derived from OM10 are adaptations under CC BY-SA 3.0 and must be shared alike with attribution; the tool stamps these exports with the required notice. CI never uploads OM10-derived artefacts publicly; the OM10 integration test runs only where a maintainer has supplied the STEP file through a private cache.

## Repository layout and cross-platform build

One script per platform builds everything from a fresh `main` checkout: `./build.sh` on Linux and macOS, `.\build.ps1` on Windows. Both are thin wrappers over CMake presets, so CI and developers run exactly the same steps.

```
open-horology-studio/
├── build.sh / build.ps1        # one-command build + test
├── CMakeLists.txt, CMakePresets.json
├── vcpkg.json                  # pinned dependency manifest (baseline commit)
├── cmake/                      # superbuild for Chrono, MFEM, Netgen (pinned commits)
├── src/
│   ├── core/                   # units, math, errors, logging, job system
│   ├── geometry/               # OCCT wrappers, import/export, fingerprints
│   ├── model/                  # horological domain model, rig, materials
│   ├── rigging/                # classifiers, joint inference, gear fitting
│   ├── kinematics/  dynamics/  # levels 1-2; level 3 Chrono adapter
│   ├── mesh/  fea/             # Netgen + MFEM; external CalculiX/Gmsh adapters
│   ├── manufacturing/          # tolerance Monte Carlo, DFM rules
│   ├── render/  report/
│   ├── app/                    # application services, commands, undo, project I/O
│   ├── cli/  gui/  python/     # thin front ends
│   └── plugins/                # stable C ABI for third-party rules/solvers
├── tests/ (unit, integration, verification, regression, fuzz, gui, perf)
├── data/ (materials, DFM rule sets, thirdparty manifests - no OM10 files)
├── scripts/thirdparty/fetch_om10.py
├── integrations/ (fusion360-addin, solidworks-addin)
└── docs/ (user guide, ADRs, API reference)
```

**What `build.sh` / `build.ps1` do**

1. Check prerequisites and print exact install hints: CMake 3.28+, Ninja, Git, Python 3.10+, and a compiler (GCC 13+ or Clang 17+ on Linux, Xcode 16 on macOS, Visual Studio 2022 17.10+ on Windows).
2. Bootstrap vcpkg into `.vcpkg/` at the pinned baseline; dependencies (OCCT, Qt 6, VTK, Eigen, oneTBB, HDF5, Catch2, CLI11, spdlog, nanobind and more) build from source the first time, then come from the local binary cache.
3. Superbuild the libraries pinned by commit hash (Chrono, MFEM, Netgen) via CMake `ExternalProject`.
4. `cmake --preset <os>-release` → `cmake --build` → `ctest --preset <os>` → optional `cpack` installer (AppImage/.deb, .dmg, MSI).

Flags: `--debug`, `--asan`, `--no-gui` (headless CLI-only build, much faster), `--system-deps` (use distro/Homebrew packages instead of vcpkg), `--cuda`, `--jobs N`.

The first full build is long (Qt and OCCT dominate, expect hours); afterwards incremental builds take minutes. CI publishes the vcpkg binary cache so contributors can opt in to downloading prebuilt dependencies. Supported targets: Linux x86\_64/arm64, macOS arm64 (x86\_64 best effort), Windows x86\_64.

## Testing and engineering practices

For simulation software, "tests pass" must mean "the physics is right", not just "the code runs". The suite therefore has a verification tier that compares solvers with known analytical answers, alongside conventional unit and integration tests.

| Tier | What it proves | Examples | When it runs |
| --- | --- | --- | --- |
| Unit | Each function and class behaves | Gear geometry, unit conversions, rig inference rules, JSON schema round-trips | Every commit, < 2 min |
| Property-based | Invariants hold for random inputs | Gear ratio × reverse ratio = 1; mass properties invariant under rigid transforms; import→export→import is stable | Every commit |
| Integration | Modules work together | STEP import → rig → simulate → report on synthetic movements generated by the test suite | Every PR |
| Verification (physics) | Solvers converge to known answers | Cantilever and Lewis tooth bending, Hertz contact, spiral spring stiffness, harmonic oscillator period, mesh convergence rates, NAFEMS-style benchmarks; MFEM vs. CalculiX cross-check | Every PR (fast set), nightly (full) |
| Regression (golden) | Results do not drift | Stored HDF5 results with tolerances per quantity for reference movements | Every PR |
| Fuzzing | Importers survive hostile files | libFuzzer on STEP/IGES/DXF/rig parsers | Nightly, OSS-Fuzz once public |
| GUI | Key workflows work | Qt Test scripted import/rig/animate/report flow | Every PR (offscreen) |
| CLI end-to-end | Commands, exit codes, `--json` schema | Golden JSON outputs | Every PR |
| Performance | No slowdowns | Google Benchmark on meshing, assembly, solver throughput; results tracked over time with alerts at >10% regression | Nightly |
| OM10 integration | Real-world movement analyses end to end | Fetch from private cache → import → full analysis | Nightly on maintainer runners only |

The reference test movements are **generated procedurally** by the test suite (a simple going train with lever escapement built through OCCT), so CI never depends on third-party CAD files.

**Engineering practices**

- Trunk-based development on `main`, short-lived branches, required PR review, squash merges with Conventional Commits and SemVer releases.
- CI on GitHub Actions: Linux, macOS (arm64) and Windows; debug and release; ASan/UBSan and TSan jobs; coverage reporting with a ratchet.
- Static analysis: clang-tidy, clang-format, cppcheck, CodeQL; warnings as errors in CI.
- Supply chain: pinned dependencies, SBOM per release, signed release artefacts, Dependabot/Renovate for vcpkg baseline bumps.
- Architecture Decision Records in `docs/adr/` for each major choice (C++, OCCT, licensing, solver selection).
- Documentation: user guide (MkDocs), C++ API (Doxygen), Python API (generated from nanobind stubs), and a worked OM10 tutorial.
- Numerical reproducibility: deterministic seeds, input hashes recorded in every results file, and a `ohs verify` command that re-runs the verification suite on a user's machine.

## Roadmap

Claude builds Open Horology Studio alone, so the roadmap is ordered by milestones, not calendar months: each one is a vertical slice you can install and use on OM10, and each ends with a gate you check on your own machine. Progress is measured by gates passed, not time spent.

| Milestone | Status | Scope | Gate (you check it) |
| --- | --- | --- | --- |
| [x] **M0 Foundation: see your movement** | Completed (see notes) | Build scripts, CI on 3 OSes, project file, CLI skeleton. OM10 STEP import, 3D viewer, exploded and section views. | You build from a fresh clone and open OM10 in the viewer. |
| [x] **M1 Movement model: watch it run** | Completed (see notes) | Auto-rigging, gear fitting, kinematic animation. Explain panel: what each part does, gear ratio walkthrough. | OM10 animates; train gives 25,200 vph and 1 rpm seconds. |
| [x] **M2 Timekeeping: why it keeps time** | Completed (see notes) | Torque chain, reduced escapement model, virtual timegrapher. What-if studies: baseline vs. modified, Chrono verification. | Simulation agrees with Chrono and your timegrapher trend. |
| [x] **M3 Structural analysis: will it hold** | Completed (see notes) | Netgen meshing, MFEM static, modal, nonlinear; VTK results. Guided load-case setup, CalculiX cross-check, verification suite. | Benchmarks match hand calcs and converge as expected. |
| [x] **M4 Manufacturing: can you make it** | Completed (see notes) | Tolerance Monte Carlo, DFM rules for your own tools, collisions. Gear generator to STEP/DXF, part drawings, HTML/PDF reports. | You make one part from a tool-generated drawing. |
| [x] **M5 Release 1.0: learn by doing** | Completed (see notes) | Guided OM10 lessons, docs, installers, performance tuning. Python API freeze, plugin ABI 1.0, CAD add-ins. | You finish an OM10 mod study using only the guided lessons. |

Six milestones to 1.0, each usable on its own and closed by a gate you check.

**Status notes**

- **M0 completed** as a headless build: one-command `build.sh` with CMake presets, TOML project file, `ohs` CLI skeleton, STEP import through Open CASCADE, and the full test suite (also clean under ASan/UBSan). Since delivered: a basic Qt viewer with exploded and cut-away views (`ohs-gui`, see *Desktop application* below), `build.ps1`, a CI workflow and a vcpkg manifest (the last three configured, not exercised on Windows or macOS). Still not built: the OpenCASCADE OpenGL viewer and the vcpkg superbuild of Chrono, MFEM and Netgen. The "open OM10 in the viewer" gate passed on the real file (see *Gate runs*).
- **M1 completed** for the engine: auto-rigging with module-from-mesh gear fitting (addendum, dedendum, involute base circle and pressure angle), a manual `ohs rig set` correction, kinematic poses (smooth or one escapement step per beat), animated glTF export (`ohs animate`) and the Explain output (`ohs explain`, part facts and a gear-ratio walkthrough). Verified on the procedurally generated going train, which gives 25,200 vph and a 1 rpm fourth wheel. The live animation timeline and Explain panel have since been delivered in `ohs-gui`, and the gate passed on the real OM10 file (see *Gate runs*). Cycloidal flank fitting has since been added.
- **M5 completed** (ADR 5), which closes 1.0 as a command-line and Python product: seven guided lessons (`ohs lesson list|show|check`: train, timegrapher, hairspring, mainspring, escapement, jewelling, skeletonise) with checked exercises on a built-in demo movement or your own project, the frozen Python API 1.0 (nanobind over the in-process CLI, with an API snapshot test), plugin ABI 1.0 for DFM rule sets (C header, loader, example and refusal tests), CAD add-in role export (shared tested logic, Fusion 360 add-in, SolidWorks sketch), a MkDocs site with a generated CLI reference and a docs-lint test, CPack packages (DEB and TGZ tested), `build.ps1`, CI workflow, vcpkg manifest, benchmarks with a regression-compare script, and performance work (exact spline gear solids 11x faster, extrusion section shortcut 70x, `ohs report` 46 s to 10 s). **Not delivered at M5, and said so in `docs/limitations.md`:** the Qt desktop UI (delivered afterwards in basic form, see *Desktop application*), Chrono (delivered afterwards as an optional adapter), the real MFEM, Netgen and VTK libraries (still not integrated), a Fusion 360 or SolidWorks run of the add-ins, Windows and macOS builds and installers (configured, not exercised), HDF5 results, and the gate by a person (it was run by Claude with a stand-in calibre, see *Gate runs*). The automated stand-in for that gate is a test that applies every lesson's reference solution through the CLI and checks that wrong answers fail with advice.
- **M4 completed** for the engine (ADR 4): Monte Carlo tolerance stack-up (backlash, engagement, end and side shake, sensitivity ranking, seeded and thread-independent), data-driven DFM rule sets for your own processes (internal radius, wall, depth-to-diameter, cutter module, tooth and leaf counts, undercut, jewel press fit), a 2D motion sweep for overlaps and close approaches, an involute gear generator (cycloidal added later) to STEP, DXF, SVG and PDF, dimensioned part drawings, and HTML and PDF reports with pass/fail marks (`ohs gear|draw|tolerance|dfm|collisions|report`). Deferred: tolerances read from STEP AP242 PMI (a TOML table is used), 3D collision checks and sweeps beyond one revolution or tooth pitch, DFM rules for features that vary through the thickness, and the gate itself (you machine one part from a generated drawing).
- **M3 completed** with in-house tools in place of MFEM, Netgen and VTK (ADR 3): a 2D Delaunay/Ruppert mesher on part sections extruded to 4- and 10-node tetrahedra, static, modal, Newmark transient, total-Lagrangian nonlinear, thermoelastic, penalty contact and press-fit ties, VTU output, a CalculiX deck writer and result parser, an involute gear generator, guided tooth-bending (with the Lewis hand calculation) and shock cases, and `ohs verify physics [--full]`. CalculiX 2.21 is now installed and its cross-check has run: an extruded 2x1x1 block, fully clamped at one end with a point load, agrees with OHS within 1e-3 of the peak displacement (`CalculiX input deck and result parser` test). Deferred: a general 3D volume mesher, MFEM/Netgen/VTK adapters (the Ubuntu packages `libmfem-dev` 4.5.2, `netgen` 6.2 and `libvtk9-dev` 9.1 are installed but nothing links them), HDF5 results (`libhdf5-dev` is installed, unused), a coil-contact mainspring model. The gate on real OM10 parts passed (see *Gate runs*).
- **M2 completed** for the engine: torque chain (barrel to escape wheel with mesh losses), event-driven reduced escapement model, virtual timegrapher, positions, power reserve, what-if studies with per-change explanations, bench calibration against timegrapher CSVs, and `ohs torque|simulate|timegrapher|timekeeping|whatif|verify levels|bench|calibre`. The cross-check first used an independent fixed-step two-body contact model (ADR 2); `ohs verify levels` agrees within 2 % amplitude and 1 s/day. The optional Project Chrono 9.0.1 adapter (`-DOHS_WITH_CHRONO=ON`, `--engine chrono`) has since been added and agrees with the reduced model within the same tolerances, on the generated train and on OM10. It models the balance and escape wheel as Chrono shafts, not rigid bodies with collision geometry. Deferred: HDF5 results (JSON/CSV now), Chrono for the whole train or shock transients, and the gate on a physical OM10 timegrapher trace. The Qt rate-against-amplitude plot has been delivered.

**Open items after 1.0** (the single list; everything not here is done and tested on Linux)

- [x] Qt desktop application, basic (viewport, rig list, Explain, timeline, analyses, job queue, one plot, role undo)
- [x] Project Chrono adapter for the level-3 escapement model (optional build; shafts, not collision bodies)
- [x] CalculiX cross-check run against a real `ccx` (clamped block, within 1e-3)
- [x] Chrono half of the M2 gate, on the real OM10 file (0.33 % amplitude, 0.04 s/day)
- [x] MP4 export (`ohs-gui --video`, via ffmpeg)
- [ ] OpenCASCADE OpenGL / VTK viewport, PBR, motion trails, force arrows, field plots
- [x] GUI undo of every project edit (role changes and declared meshes, by file snapshots; views are not undoable)
- [x] Worker-process job queue that survives restarts and cancels running jobs (journalled; a running job restarts from the beginning, no solver checkpoints)
- [x] MFEM adapter as an independent cross-check of the static solver (`ohs fea --mfem`)
- [x] Netgen as an out-of-process 3D volume mesher for the shock case (`--mesher netgen`, via its Python module)
- [ ] A VTK adapter; MFEM as the solver itself; a 3D mesher of our own; Netgen for the other structural cases
- [ ] Chrono for the whole train, rigid bodies with collision geometry, shock transients
- [x] HDF5 results for the structural fields (`ohs fea --hdf5`; JSON, CSV and VTU as before)
- [x] USD export (`ohs animate -o x.usda`, checked with Pixar's reader)
- [x] IGES and DXF import (`ohs import x.igs|x.dxf`; DXF is a flat drawing, extruded)
- [ ] STEP PMI tolerances
- [x] Cycloidal gear profiles (generated, fitted, meshing-checked; Lewis and contact-ratio formulas stay involute)
- [ ] Escapements other than the Swiss lever; coil-contact mainspring and hairspring models
- [ ] Windows and macOS builds, installers and CI run (configured only); Fusion 360 and SolidWorks add-in runs
- [x] TSan (clean under `setarch -R`), coverage and static-analysis CI jobs (TSan run locally; coverage and clang-tidy jobs are configured, not run here)
- [ ] vcpkg superbuild of Chrono, MFEM and Netgen at pinned commits
- [ ] Human-run gates: OM10 timegrapher trend and bench calibration, machining one part from a drawing, an OM10 study done by a person from the lessons alone
- [ ] Trademark and domain check on the name; a sanctioned OM10 download URL from openmovement

**How a solo AI developer keeps this honest**

- **Tests before code.** Each task starts with failing tests, including physics checks against analytical answers, so correctness never rests on Claude's own judgement alone.
- **Small, reviewable steps.** Work lands as small pull requests on `main` with CI green on all three OSes; you can read, try or reject any of them.
- **Independent cross-checks** replace a second engineer: reduced model vs. Chrono, MFEM vs. CalculiX, every solver vs. hand calculations, and simulation vs. your bench measurements.
- **Written decisions.** Each design choice gets a short ADR, so you (or a future contributor) can see why, not just what.
- **You are the acceptance tester.** Every gate is something you do yourself, e.g. open OM10, compare a timegrapher reading, machine one part.

OM10 is the running example from M0 onward, and each milestone adds a guided lesson on it, so the tool teaches as it grows.

**Beyond 1.0: alternative escapements.** From M2 the escapement is a plug-in interface (geometry roles, contact events, impulse and locking model), with the Swiss lever as its first implementation. Later plug-ins add the co-axial, detent and constant-force escapements and silicon parts, each with its own verification tests and a what-if lesson comparing it with the lever on OM10.

## Risks and open questions

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Auto-rigging misclassifies parts in messy STEP files | Users spend time fixing the rig | Confidence scores, a fast manual rig editor, CAD add-ins that export the rig directly |
| Escapement dynamics are stiff and friction-sensitive; predictions disagree with real movements | Low trust in rate/amplitude results | Three fidelity levels cross-checked; calibrate friction against timegrapher data from a physical OM10 kit |
| Qt and OCCT make first builds long and fragile on some platforms | Contributor friction | vcpkg binary cache, `--no-gui` build, `--system-deps` path, CI on all three OSes from day one |
| openmovement changes its website or login | `--login` fetch breaks | Manual file import (`--file`) is the default; hashes in the manifest; ask openmovement for a stable URL |
| CC BY-SA content leaks into the Apache-2.0 repo | Licensing contamination | Rig preset holds names only; `.gitignore` guards; CI check rejecting STEP/IGES files outside `tests/generated` |
| FEA on sub-millimetre features needs very fine meshes | Slow runs | High-order elements, submodelling, optional GPU build |

**Open questions**

- Will openmovement endorse the integration and offer a sanctioned download URL? To be decided; manual `--file` import works either way.
- Run a trademark and domain check on "Open Horology Studio" before the first public release. "OHS" is a crowded acronym (e.g. occupational health and safety), so the full name should lead in docs and search.

**Settled**

- **Team**: none; Claude is the sole developer.
- **Audience**: an amateur watchmaker who needs both ease of use and real engineering depth (see Learning and what-if studies).
- **Workshop**: lathe, mill, wheel-cutting engine and cutters, 3D printer and timegrapher; no EDM. Default DFM rule sets target these processes; EDM, LIGA and silicon rules stay available for parts you outsource.
- **First lessons**: all five modification types — hairspring and balance, mainspring choice, escape wheel and lever geometry, jewelling, and skeletonising the gear bridge.
- **Alternative escapements**: supported by the architecture from the start, implemented after 1.0 (see Roadmap).
- **Name**: Open Horology Studio (OHS), CLI `ohs`, Python package `ohs`, because "Horology Studio" is already taken by an iOS app.

## Sources

- [openmovement.org home page (OM10 download, kit status)](https://openmovement.org/)
- [openmovement General Conditions (CC BY-SA 3.0 licence, marking rules)](https://openmovement.org/general-conditions/)
- [Worn & Wound: Introducing the openmovement OM10](https://wornandwound.com/introducing-the-openmovement-om10-the-first-open-source-watch-movement/)
- [Hackaday: Open Source Watch Movement (STEP files, 3.5 Hz)](https://lemmy.world/comment/17467611)
- [Open Cascade Technology (version, licence)](https://en.wikipedia.org/wiki/Open_Cascade_Technology)
- [Project Chrono (BSD-3)](https://pypi.org/project/projectchrono/)
- [MFEM (BSD-3, GPU support)](https://github.com/mfem/mfem/)
- [Gmsh licence (GPL-2-or-later)](https://gitlab.onelab.info/lrp/gmsh/-/commit/d4b25ed90cd6d5900c06a05e30960fc44a7e499d)
- [Gmsh mailing list on non-linked use](https://onelab.info/pipermail/gmsh/2019/013348.html)

- **Desktop application (added after M5)**: `ohs-gui` (Qt 6 Widgets, `-DOHS_BUILD_GUI=ON`, `cmake --preset gui`) gives the M0/M1 viewer gate its window: a 3D viewport with orbit, pan, zoom and picking, exploded and cut-away views, a model tree with visibility, the rig confidence list, the Explain dock, the animation timeline (smooth or stepped), one button per analysis, a log console and "save session as script". Verified by a scripted offscreen test (`gui_scripted_flow`, 151/151 with the GUI build, Qt 6.8.3 on Linux). **Also in the window:** a journalled job queue of worker processes with a Jobs dock (queued and running jobs can be cancelled; unfinished jobs resume after a restart), a rate-against-amplitude plot, and undo/redo of project edits (role changes, declared meshes). **Still not delivered:** the OpenCASCADE OpenGL/VTK viewport (the viewport is Qt-painter based), PBR, motion trails, force arrows, further plots, MP4 export; and the human-run gates (someone opening OM10 in the viewer, a physical timegrapher trend, machining a part, finishing the OM10 study) remain unperformed. Windows and macOS remain configured, not exercised.

**Gate runs on the real OM10 file** (`om10-00001_20220701_va_01_3.stp`, run by Claude, not by a person; results as measured):

- **M0, open OM10 in the viewer: passed.** 166 parts import and `ohs-gui --snapshot` renders the plates, date ring, stem, jewels, balance and train; the exploded side view separates the layers. It needed a fix first: the OM10's dial axis is Y, and the importer assumed Z (0 gear meshes, 148 of 166 parts unknown). It now finds the axis from the geometry and stores it in the project.
- **M1, 25,200 vph and 1 rpm seconds, and it animates: passed.** With the escape wheel and the barrel mesh declared in the OM10 preset (the geometry cannot show them), the root is the barrel (3.94 h per turn), the centre wheel turns once an hour, the fourth wheel at exactly 1 rpm (`--expect` passes), the escape wheel at 10.5 rpm with 20 teeth. `ohs animate` writes a valid glTF (4,201 keyframes, 8 animated parts) and the viewer frames at t=0 and t=7.5 s differ.
- **M2, simulation agrees with Chrono and your timegrapher trend: half done.** Project Chrono 9.0.1 (built from source) running the detailed model agrees with the reduced one on the OM10 train (437.2 against 438.6 degrees, 0.33 % amplitude, 0.04 s/day; `ohs verify levels --engine chrono`), as does OHS's own contact model. The comparison with a physical timegrapher was **not possible**: there is no timegrapher data, and the STEP file has no mainspring, balance or hairspring data, so the 437 degree amplitude with the reference calibre is not a prediction of the real movement (a barrel mesh that was missing had given 1,018 degrees).
- **M3, benchmarks match hand calculations: passed.** `ohs verify physics` agrees; on the OM10 centre wheel the root stress is 18 MPa against 6.4 MPa by Lewis. Running the lessons exposed a solver bug (windows were clamped like pivot holes), fixed with a regression test.
- **M4, make one part from a tool-generated drawing: drawing and file checked, part not made.** The drawing of the OM10 centre wheel (75 teeth, module 0.11, 8.544 mm, 0.2 mm thick) is correct apart from a dimension label that overlaps the rim, and a generated 75-tooth STEP re-imports with the exact tip and root radii. Nobody machined anything.
- **M5, finish an OM10 mod study using only the guided lessons: passed with a stand-in calibre.** All seven exercises were done through the CLI from the lessons' own advice (two failed first, as intended, and the advice fixed them). The mainspring exercise is impossible with the reference calibre (437 degrees is already above its 315 degree ceiling), so an assumed OM10 calibre with a lower barrel torque was used, labelled as not measured.
- **Not done by anyone:** a physical timegrapher trend; machining a part; the Windows and macOS builds. Chrono's step is slow on a machine whose clocksource is `hpet` (about 2 minutes for the OM10 check).
