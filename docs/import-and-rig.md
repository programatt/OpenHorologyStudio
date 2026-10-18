# Import and rig

## Import

```
ohs import movement.step -o movement.hsproj --beat-rate 28800
```

`--up x|y|z` names the CAD axis that is normal to the dial; `--preset roles.toml` applies name
based role rules (the [CAD add-ins](cad-addins.md) write these for you); `--min-teeth` sets the
fewest teeth that make a part a wheel.

What `import` does, in order:

1. **Classify** each solid. Names and colours in the STEP file are hints; the geometry decides the
   rest: a periodic outline gives a tooth count (Fourier fundamental of the mid-height section),
   a small cylinder with a hole is a jewel candidate, a wide thin part a plate.
2. **Group mobiles** by coaxiality.
3. **Infer meshes**: two toothed parts whose pitch circles are tangent within tolerance, with
   plausible addendum and dedendum for the module taken from the centre distance.
4. **Fit gear parameters**: module, addendum, dedendum, and whether the flank is an involute (with
   its base circle and pressure angle). Cycloidal flanks are not fitted yet.
5. **Score** every inference.

!!! note "Fused wheels and pinions"
    A wheel and pinion fused into one solid are measured at mid-height only. Split them in the CAD
    export, or correct the rig.

## Check and correct

```
ohs rig check movement.hsproj          # every part, role, confidence, gear fit and warning
ohs rig set movement.hsproj "third wheel" --role pinion
```

`rig check --strict` fails on warnings; use it in scripts. Setting a role re-infers mobiles and
meshes, and your choice is recorded with confidence 1 and source `user`.

## Read the train

```
ohs kinematics movement.hsproj --expect M4=1      # fails (exit 1) if M4 is not 1 rpm
ohs explain movement.hsproj --train               # mesh by mesh, with the ratios
ohs explain movement.hsproj "escape wheel"        # what a part does, with this movement's numbers
ohs animate movement.hsproj -o train.glb --stepped --duration 10
```

`animate` writes a binary glTF with one node per part and keyframes from the kinematic motion
(`--stepped` advances one escapement step per beat), viewable in any glTF viewer or Blender.
