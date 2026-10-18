# Concepts

## The horological model

A STEP file is geometry. OHS turns it into a **movement**:

- a **component** is one part with a role (plate, bridge, barrel, arbor, wheel, pinion, escape
  wheel, pallet fork, balance, hairspring, roller, jewel, screw), a material and its measured
  features (tooth count, outer radius, bore, thickness);
- a **mobile** is the set of components that turn together about one axis (a wheel, its pinion
  and its arbor);
- a **mesh** joins two toothed components of different mobiles whose pitch circles touch;
- the **train** is the chain of meshes from the barrel to the escape wheel.

Every analysis reads this one model. The **rig file** (the `.hsproj` project, TOML) is its stable,
human-readable form: you can write it by hand, generate it from a CAD add-in, or let `ohs import`
infer it and correct what it got wrong (`ohs rig set`).

## What the geometry cannot tell

Some inputs are not in a CAD model: the hairspring section, balance inertia, pivot friction, lift
angle, mainspring torque. These live in a **calibre file** (`ohs calibre init` writes the defaults
of a typical 3.5 Hz lever calibre). Tolerances and DFM limits are in their own files. Keeping them
apart means you can change a design in CAD without losing what you measured on the bench.

| File | Holds | Command |
| --- | --- | --- |
| `.step` | geometry | any CAD tool |
| `.hsproj` | the rig: roles, mobiles, meshes, gear fits | `ohs import`, `ohs rig set` |
| `calibre.toml` | balance, hairspring, escapement, mainspring, friction | `ohs calibre init`, `--calibre` |
| tolerance table | position, tooth thickness, shake tolerances | `--tolerances` |
| DFM rules | your processes and their limits | `--rules` |
| materials | properties with a source note | `--materials` |

## Three levels of timekeeping

| Level | What it models | Use |
| --- | --- | --- |
| Kinematic | rigid gear ratios and beat arithmetic | animation, ratio checks |
| Reduced | the balance oscillator with the lever as event-driven impacts | amplitude, rate, isochronism, sweeps |
| Detailed | balance and escape wheel as two bodies with a stiff damped contact, fixed small steps | cross-checking the reduced model |

`ohs verify levels` runs the reduced and detailed models side by side and fails when they differ
by more than 2 % amplitude or 1 s/day.

## Units and conventions

Millimetres, degrees, seconds, newtons. Structural analysis works in mm, N, MPa, tonne. The dial
plane is X/Y and Z is the dial axis (`--up` rotates other CAD conventions into it). Rates are in
seconds per day, positive when the watch gains.

## Determinism

Given the same inputs, seed and version, every command gives the same result, whatever the number
of threads. Tolerance runs use a counter-based random generator so sample *n* never depends on
scheduling; the virtual timegrapher's timing jitter is seeded too.

## Exit codes and JSON

Every command takes `--json`. Exit code 0 means success, 1 means a check failed (a tolerance run
above its limit, a DFM error, a collision), 2 means bad input. Scripts and the
[Python API](python.md) rely on this.
