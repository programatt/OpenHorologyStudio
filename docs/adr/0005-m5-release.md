# ADR 5: M5 release 1.0

- **Lessons are code with data in it.** Each lesson computes its explanation from the user's
  own movement (so the figures are real), and each exercise is checked by running the same
  analyses the commands use: an answer is compared with the engine's, a study is parsed and run
  through the what-if engine after checking that it only touches the parameters the lesson is
  about, a cutout is meshed and put through the shock analysis. Lessons work without a CAD file on
  a built-in demo movement. The automated stand-in for the M5 gate ("you finish an OM10 mod study
  using only the guided lessons") is a test that runs every lesson on the demo movement, applies
  each reference solution and checks it passes, and that a wrong answer fails with advice.
- **Python is a thin, frozen layer over the command line.** The extension module (nanobind) runs
  the CLI in-process with its output captured; the package turns each command into a method that
  returns the `--json` output. Python, the CLI and every future front end therefore share one
  implementation, and the JSON schemas are the contract. API 1.0 is recorded in
  `python/ohs/api_1_0.json`; a test compares the live API with it (removals and changed
  parameters fail, new keyword arguments with defaults are allowed).
- **Plugin ABI 1.0 is a C header.** `include/ohs/plugin.h` defines DFM rule sets with a
  `struct_size` on every structure, major/minor version checks on load and clear refusal messages.
  Only DFM rule sets exist in 1.0; solver and escapement plugins get their own entry points in
  later minor versions. Loading uses `dlopen` / `LoadLibrary`.
- **CAD add-ins export STEP plus a role preset.** The role logic is shared Python, tested without
  any CAD tool and through a real `ohs import`. The Fusion 360 add-in and the SolidWorks C# sketch
  could not be run here and are labelled accordingly.
- **Installers** come from CPack: DEB and TGZ are exercised (the package contents are tested);
  DragNDrop (macOS) and WIX (Windows) are configured but not built here.
- **Performance** is measured by `ohs_perf` (a small in-house harness, since Google Benchmark is
  not installed) which writes JSON; `scripts/perf_compare.py` compares two runs and flags slowdowns
  above 10 %. The tuning done for 1.0 targeted the slowest user-visible commands: report
  generation caches and parallelises the geometry work, and the timekeeping evaluation runs its
  independent steady states on threads. Results are deterministic whatever the thread count.
- **Documentation** is MkDocs plus a generated CLI reference; `scripts/check_docs.py` fails when a
  documented command or option does not exist, and runs as a test.
- **Not in 1.0:** the Qt desktop UI (no Qt in the build environment), the real Chrono, MFEM and
  Netgen libraries, 3D meshing, cycloidal profiles and alternative escapements; the limitations
  page says so plainly.
