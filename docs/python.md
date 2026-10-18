# Python API

```python
import ohs

p = ohs.Project.import_step("movement.step", "movement.hsproj", beat_rate=25200)
print(p.kinematics()["escape_teeth"])

study = p.whatif(changes=[{"parameter": "hairspring_length_mm", "scale": 1.002}])
for row in study["metrics"]:
    print(f'{row["name"]:40s} {row["baseline"]:10.2f} {row["modified"]:10.2f}')
```

Every method runs the same engine as the `ohs` command and returns its `--json` output as a
dictionary (a `Result`, with `exit_code`, `ok`, `stdout` and `stderr` attached). Names, units and
JSON fields are those of the command line, so [the CLI reference](reference/cli.md) documents the
fields too. That also makes the API usable from Jupyter and optimisation loops.

- Exit code **0**: `result.ok` is true.
- Exit code **1**: a check failed (a tolerance run over its limit, a DFM error, a collision, a
  failed lesson). The result is returned with `ok` false; nothing is raised.
- Exit code **2**: bad input. `ohs.OhsError` is raised with the message of the command.

## Calls

| Call | Does |
| --- | --- |
| `Project.import_step(step, output, beat_rate=None, preset=None, up="z")` | import and rig |
| `Project(path)` | open a project |
| `.rig_check(strict=False)`, `.kinematics()`, `.explain(part=None, train=False)` | read the rig and the train |
| `.torque(wind=1.0, calibre=None)`, `.timekeeping(calibre=None)` | torque chain; amplitude, rate, positions, reserve |
| `.simulate(duration, level, position, wind, calibre)`, `.timegrapher(...)` | time-domain runs and the virtual timegrapher |
| `.whatif(changes=None, study=None, name, verify, calibre)` | baseline against a modified design |
| `.tolerance(samples, seed, table, fail_above)`, `.dfm(...)`, `.collisions(...)` | manufacturing checks |
| `.fea_tooth(part, ...)`, `.fea_shock(part, ...)` | structural checks |
| `.draw(part, output)`, `.report(prefix, ...)` | drawings and reports |
| `ohs.gear(teeth, module, ...)` | generate a wheel as STEP, DXF, SVG, PDF |
| `ohs.materials()`, `ohs.thermal(...)`, `ohs.verify_physics(full=False)` | catalogue, temperature, solver checks |
| `ohs.lessons()`, `ohs.lesson(id, project=None)`, `ohs.check_lesson(id, ...)` | guided lessons |
| `ohs.calibre_template(beat_rate)` | the default calibre as TOML text |
| `ohs.run(*args, json=True, check=True)` | any command, by its command-line arguments |

## Stability

API version 1.0 is frozen. The names, parameter names, order and defaults in
`python/ohs/api_1_0.json` change only in a new major version; the test suite compares the live
API with that snapshot. Minor versions may add new functions and new keyword arguments with
defaults. `ohs.API_VERSION` is `"1.0"`; `ohs.__version__` is the OHS release.

Anything the typed methods do not cover is reachable with `ohs.run("command", "--option", ...)`,
which is as stable as the command line.

## Building

See [install](install.md). The extension module is built with nanobind (`-DOHS_BUILD_PYTHON=ON`);
the package is `python/ohs`.
