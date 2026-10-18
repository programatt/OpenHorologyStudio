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

Results do not depend on the number of threads: every parallel loop writes into a fixed slot per
item and the output is assembled in order.
