# ADR 2: M2 timekeeping models

- **Parameters live outside the rig.** Hairspring section, balance inertia, friction, escapement
  angles and the mainspring curve cannot be read from geometry, so they sit in a `calibre.toml`
  (`ohs calibre init` writes the defaults). The rig supplies what it does know: the gear train,
  the escape wheel tooth count and the beat rate. The defaults describe a typical 3.5 Hz lever
  calibre, tuned so a fully wound watch runs near 275° and the spring lasts about 50 hours on the
  generated test train; they are not OM10 values.
- **Level 2 is event-driven, not fixed-step.** The balance is integrated with RK4 between
  escapement events. Entering the unlock zone, entering and leaving the impulse, and each
  turning point are located by bisection on the step, so the piecewise-constant escapement
  torques switch at the exact instant. The impulse start is an inelastic pick-up of the escape
  wheel (reflected inertia `I_e (ΔΦ/Λ)^2`), which is why the wheel inertia costs energy twice per
  beat (pick-up and drop).
- **Level 3 is an independent stand-in for Chrono.** Chrono is not available in this
  environment, so the "agrees with Chrono" half of the M2 gate is checked against a second model
  written separately: balance and escape wheel as two bodies, fixed 3 µs steps, no event
  location, and the wheel/pallet interaction as a stiff damped one-sided contact. The impulse,
  inelastic pick-up, drop and lock arise from that contact. The models share parameters only.
  `ohs verify levels` fails (exit 1) when amplitude differs by more than 2 % or rate by more than
  1 s/day. Replacing this model by a Chrono adapter stays on the roadmap; the interface
  (`DetailedConditions` → events) is what an adapter would implement.
- **Quasi-static run-down.** Amplitude settles in about a minute while the spring unwinds over
  tens of hours, so power-reserve and run-down results are sequences of settled states, one per
  state of wind, not a single 48 h integration.
- **Virtual timegrapher** reads only the simulated sounds (impulse start = tick, drop): rate from
  the regression slope of tick time against beat number, amplitude from
  `lift / (2 sin(π t_lift / T))`, beat error from alternate intervals. Timing jitter uses a
  seeded Box-Muller on `mt19937_64`, identical on every platform.
- **What-if studies** regulate the baseline hairspring to zero rate (dial up, half wound), apply
  the changes to a copy, and attach a one-sentence cause per change that evaluates the governing
  relationship for this design (e.g. frequency ∝ √(k/I) predicts the rate change, which is shown
  beside the simulated one).
- **Bench loop** fits the pivot friction (optionally air damping) to measured amplitudes by
  golden-section search on the rms amplitude error, and reports rate and amplitude-trend
  agreement. Rates are compared after removing the mean offset because the regulator setting of
  a real movement is arbitrary.
- **Results** are JSON/CSV for now; HDF5 output arrives with the FEA results in M3.
