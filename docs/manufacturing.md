# Manufacturing

## Generate a wheel

```
ohs gear --teeth 24 --module 0.3 --pressure-angle 20 --bore 1.0 --thickness 0.4 \
         --name wheel24 --material brass --step-out wheel24.step --dxf wheel24.dxf \
         --svg wheel24.svg --pdf wheel24.pdf
```

The STEP solid imports straight back into OHS (and into your CAD tool). The drawings show a front
view with tip, pitch, root and bore circles, a side view with the thickness, and the gear data a
cutter set-up needs. The PDF is at the largest standard scale that fits the sheet. `--profile-only`
writes just the outline and bore, without annotation, for laser, wire or wheel-cutting set-up.

The profile is an involute with a circular root fillet (`--fillet`, in modules). `--profile cycloidal`
generates the traditional horological tooth instead: an epicycloid addendum and a hypocycloid
dedendum, both traced by a rolling circle of `--rolling-radius` modules (default 3, which gives a
12-tooth pinion radial dedendum flanks). The wheel and pinion of a mesh must be cut with the same
rolling radius; the generator is checked by meshing a 30-tooth wheel with a 10-tooth pinion and
finding no overlap at any phase.

## Draw a part of your project

```
ohs draw movement.hsproj --part "train bridge" -o bridge.pdf     # or .svg, .dxf
```

Outline, round holes with centre marks and diameters, overall size, and hole positions measured
from the lower-left corner of the bounding box. Odd-shaped cut-outs are drawn as outlines and
listed as irregular.

## Tolerance stack-up

```
ohs tolerance movement.hsproj --samples 20000 --seed 1 --tolerances table.toml
```

Each tolerance is an independent random variable (normal with +- tol at three standard deviations,
or uniform). One sample is one virtual build. For each mesh you get the distribution of backlash,
the contact ratio, the share of builds that bind and that under-engage, and which dimension
drives the backlash; for each mobile the end shake and side shake and the share outside their
windows. Exit code 1 when more than `--fail-above` (default 2 %) of builds have a problem.

```toml
[tolerance]
position_tol = 0.010          # each coordinate of every pivot, +- mm
tooth_thickness_tol = 0.006
arbor_length_tol = 0.020
bridge_gap_tol = 0.015
nominal_end_shake = 0.030
end_shake_min = 0.010
end_shake_max = 0.080
```

The defaults are a workshop-grade table, not a measured capability of your machines.

## Design for manufacturing

```
ohs dfm --list
ohs dfm movement.hsproj --process mill,wheel-cutting
ohs dfm movement.hsproj --process mill --rules my-rules.toml
```

Rules are data: `data/dfm/rules.toml` lists, per process, the rules to run and their limits (the
end mills you own, the cutter modules you have, minimum wall). Copy it, edit it, and pass it with
`--rules`. Available rules:

| Rule | Checks |
| --- | --- |
| `internal_radius` | smallest concave corner (a hole is a concave feature of its own radius) against the smallest tool |
| `depth_to_diameter` | pocket depth over the largest cutter that fits the tightest feature |
| `min_wall` | thinnest wall across material |
| `module_cutter` | the module is within a tolerance of a cutter you own |
| `min_teeth`, `pinion_leaves` | tooth and leaf counts |
| `undercut` | involute undercut below 2 / sin^2(alpha) teeth; cycloidal undercut below 4 rolling radii (in modules) |
| `jewel_fit` | press-fit interference of jewel seats, 4 to 20 um by default |

Every rule reports a finding, passes included; errors name a fix. Exit code 1 when any rule
errors. Rules judge the section at mid-height, so features that vary through the thickness are
not seen. Your own rules in code: see [plugins](plugins.md).

## Collisions

```
ohs collisions movement.hsproj
```

Sweeps the moving train from the assembled pose: a meshing pair through one tooth pitch, any other
pair through one revolution of its faster mobile, and reports the closest approach or the deepest
overlap of every pair that shares a height range. Overlaps under 20 um count as contact within the
outline accuracy; exit code 1 on a real overlap. Poses beyond one revolution depend on the whole
train's common period and are not covered.

## Reports

```
ohs report movement.hsproj -o movement-report --format html,pdf
```

One document with a pass / warn / fail summary, then rig and train, timekeeping with an amplitude
chart, tolerance stack-up, DFM findings, collisions, tooth bending and the solver verification.
HTML is self-contained and follows your light or dark scheme; the PDF is written by OHS itself.
`--no-fea` and `--no-timekeeping` skip the slow sections. The exit code is 1 when the overall
result is a fail.
