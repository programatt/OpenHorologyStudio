# CAD add-ins

OHS works from STEP files exported by any CAD tool. Optional add-ins for Fusion 360 and SolidWorks
export STEP together with a **role preset**, so OHS does not have to guess what each part is.

Give a part the custom property `OHS_ROLE` (for Fusion 360, an attribute in group `OHS` named
`role`) with one of `plate`, `bridge`, `barrel`, `mainspring`, `arbor`, `wheel`, `pinion`,
`escape_wheel`, `pallet_fork`, `balance`, `hairspring`, `roller`, `jewel`, `screw`, `rotor` (the
oscillating weight of an automatic). Parts without it
are classified from their names. Then:

```
ohs import movement.step -o movement.hsproj --preset movement.roles.toml --beat-rate 28800
ohs rig check movement.hsproj
```

The add-ins live in `integrations/` and are versioned separately from the core. The shared role
logic (`integrations/common/ohs_preset.py`) is tested, including an import through the real `ohs`.
The Fusion 360 add-in needs Fusion's own Python module and the SolidWorks add-in is a source sketch
that has not been built in this repository; see `integrations/README.md` for their status.

Without an add-in, name your parts so that a person can tell what they are (`third wheel`,
`escape pinion`, `train bridge`), keep each part a separate body, and export STEP AP242.
