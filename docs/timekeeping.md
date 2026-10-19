# Timekeeping

## The torque chain

```
ohs torque movement.hsproj --wind 1.0
```

Torque falls by the speed-up of every step and by the efficiency of every mesh. The command prints
the torque at each mobile, the loss in each mesh, and the power reaching the escape wheel.

## Amplitude, rate, isochronism, positions

```
ohs timekeeping movement.hsproj
ohs simulate movement.hsproj --duration 60s --position crown_up --wind 0.5
ohs simulate movement.hsproj --duration 48h -o rundown.json     # follows the unwinding
```

The reduced model integrates the balance with RK4 between escapement events (unlocking, impulse,
drop, turning point), locating each by bisection so torques switch at the exact instant. The
impulse start is an inelastic pick-up of the escape wheel, which is why the wheel's inertia costs
energy twice every beat. Runs of more than ten minutes are a sequence of settled states, one per
state of wind: the amplitude settles in about a minute while the spring unwinds over hours.

Orientation matters: lying flat, gravity has no torque on an unpoised balance and the pivots run
on their end stones; hanging, there is a gravity torque and the pivots rub on their sides.

## The virtual timegrapher

```
ohs timegrapher movement.hsproj --duration 30s --position dial_up --jitter-us 30 --seed 3
```

It reads only the simulated sounds, as a microphone would: rate from the slope of tick time
against beat number, amplitude from `lift / (2 sin(pi t / T))`, beat error from alternate
intervals, and the familiar tick and tock trace. `--csv` writes every beat.

## What-if studies

```toml
[study]
name = "bigger balance"

[[change]]
parameter = "balance_inertia_mgcm2"
scale = 1.02
```

```
ohs whatif movement.hsproj study.toml --verify
```

The baseline is regulated to zero rate (dial up, half wound) so a change shows as what it does.
Each change gets a sentence with the relation behind it and, for frequency changes, the predicted
and simulated rate shifts side by side. `--verify` also checks both designs with the detailed
model. Parameters are the field names of the calibre file.

## Escapements

The calibre's `escapement` is `swiss_lever` (the default) or `detent`, a spring detent as in a marine
chronometer. In the reduced model the lever pushes the balance in every swing, two beats per
oscillation, with tick and tock. The detent pushes it once per oscillation, in the positive swing
only: the escape wheel is released by the discharging pallet, advances a whole tooth (2 pi / teeth, not
pi / teeth) through the impulse, and is locked again; on the return swing the roller only brushes the
passing spring aside, which costs the balance `passing_spring_ratio` times the impulse torque over
the unlocking angle. Everything else, the train, the mainspring and the hairspring, is shared. Its
checks: the settled amplitude agrees with an independent energy balance per oscillation to 0.5 %, the
oscillation keeps the nominal period, and the energy budget closes. The detailed contact model and the
Chrono adapter model the Swiss lever only and refuse a detent calibre. The detent is a model of the
escapement's behaviour: nothing in the rig recognises detent geometry in a CAD file.

## Calibrating against your timegrapher

```
# readings.csv: hours,position,amplitude_deg,rate_s_per_day,beat_error_ms
ohs bench compare movement.hsproj readings.csv
ohs bench calibrate movement.hsproj readings.csv -o calibrated.toml
```

Calibration fits the pivot friction (and with `--fit-damping` the air damping) to your measured
amplitudes; rates are compared after removing the mean offset, because the regulator setting of a
real movement is arbitrary. Use `--calibre calibrated.toml` afterwards.

## Temperature

```
ohs thermal --hairspring nife_hairspring --balance brass_leaded
```

The rate change per kelvin of a hairspring and balance pair is `86400 (e/2 + 3 a_spring/2 - a_balance)`
seconds per day, with `e` the spring's thermoelastic coefficient and `a` the expansion coefficients.

## Cross-check

```
ohs verify levels movement.hsproj
```

Runs the detailed model beside the reduced one at the same state of wind and fails when they
differ by more than the tolerances (`--amplitude-tolerance`, `--rate-tolerance`).

`--engine chrono` (also on `ohs simulate --level detailed`) runs the detailed model on Project
Chrono instead of OHS's own integrator. It needs a build with `-DOHS_WITH_CHRONO=ON` (see
[Install](install.md)) and is much slower per step, so allow minutes rather than seconds.
