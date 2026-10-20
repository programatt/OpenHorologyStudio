# Performance benchmarks

`ohs_perf [runs]` times the engines (best of `runs`, default 3) and prints JSON `{name: seconds}`.

```
build/release/src/ohs_perf 3 > current.json
python3 scripts/perf_compare.py tests/perf/baseline.json current.json --threshold 0.10
```

`baseline.json` was recorded on the development machine (16 threads) with the 1.0 release build
and is **machine dependent**: use it to see the shape of the numbers, and record your own baseline
on your CI runner to track regressions. Sub-millisecond benchmarks are noisy; a 10 % change on a
5 ms benchmark is not a regression. The script exits 1 when something is slower than the threshold.

What was tuned for 1.0, measured on the generated test movement:

| What | Before | After |
| --- | --- | --- |
| `gear_solid`, 75-tooth wheel (polygon vs exact splines and arcs) | 5.3 s | 0.47 s |
| `section_region` of that wheel (general plane intersection vs the extrusion shortcut) | 6.9 s | 0.04 s |
| `ohs report` on the generated movement (sections once and shared, DFM, collisions, tooth FE on threads) | 46 s | 10 s |
| `timekeeping_evaluate` (nine settled runs and the power-reserve search on threads) | 0.41 s | 0.25 s |

What was tuned after 1.0, on the 166-part OM10 file:

| What | Before | After |
| --- | --- | --- |
| `ohs import` (hole search with one classifier per part, outlines of prisms, features on threads) | 140 s | 16 s |
| `ohs import`, the generated movement | 42 s | 3.7 s |
| `ohs dfm` / `collisions` (geometry cache, sections on threads) | 12 s / 13 s | 1.5 s / 1.9 s |
| `ohs report` | crash after 20 s | 4 s |
| Open a project in `ohs-gui` (display mesh cache) | 10 s | 0.1 s |
| A rig edit in `ohs-gui`, 400 parts | 4.4 s | 30 ms |
| Click in the viewport / wheel step, 259k triangles | 15 ms / 13.5 ms | 1.3 ms / 0.001 ms |
| OpenGL viewport, CPU time of an animation frame | 16 ms | 0.1 ms |

`ohs_perf` has `extract_features_75_tooth_wheel`, `import_movement_generated`, `step_read_generated_movement`,
`part_cache_write` and `part_cache_read`; `ohs_gui_tests "bench: scene load*"` (a hidden test, with
`OHS_BENCH_PROJECT=project.hsproj` for a larger project) times the scene load, a frame, a click and the wheel, and
the 400-part rig edit is a regular test. Run the `tsan` preset on the OM10 import whenever a loop gains threads:
the demonstration movement has no shared shapes and does not show the crash. On a kernel that maps memory above
what ThreadSanitizer expects, start it as `setarch x86_64 -R ohs ...`.

Results do not depend on the number of threads: every parallel loop writes into a fixed slot per
item and the output is assembled in order.
