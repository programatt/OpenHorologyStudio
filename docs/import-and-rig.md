# Import and rig

## Import

```
ohs import movement.step -o movement.hsproj --beat-rate 28800
```

STEP is the format to use for an assembly. `ohs import` also reads IGES (`.igs`, `.iges`: every solid
becomes a part named `part`, `part#2`, ... because IGES has no assembly names) and DXF (`.dxf`): the
closed contours of a flat drawing (lines, arcs, circles, polylines with bulges, in the file's
`$INSUNITS`) are chained, contours inside another become holes, and each outer contour is extruded
into a part called `profile`. The thickness is the DXF's own (group 39) or 1 mm. A DXF made by
`ohs gear --dxf ... --profile-only` comes back as the same wheel, tooth count and module included.

`--up x|y|z|auto` names the CAD axis that is normal to the dial. The default, `auto`, takes the axis
along which the parts are thinnest (wheels, plates and bridges are flat), and the project file
remembers it, so later commands (`animate`, `dfm`, `report`, `draw`, `fea`) need no `--up`; give it
only to override. `--preset roles.toml` applies name based role rules (the
[CAD add-ins](cad-addins.md) write these for you); `--min-teeth` sets the fewest teeth that make a
part a wheel.

A preset can also declare meshes the geometry does not show, such as a smooth barrel drum driving
the centre pinion:

```toml
[[rule]]
match = "^barrel gear$"
role = "barrel"

[[mesh]]
a = "^barrel gear$"
b = "^centre pinion$"
```

What `import` does, in order:

1. **Classify** each solid. Names and colours in the STEP file are hints; the geometry decides the
   rest: a periodic outline gives a tooth count (Fourier fundamental of the mid-height section),
   a small cylinder with a hole is a jewel candidate, a wide thin part a plate.
2. **Group mobiles** by coaxiality.
3. **Infer meshes**: two toothed parts whose pitch circles are tangent within tolerance, with
   plausible addendum and dedendum for the module taken from the centre distance.
4. **Fit gear parameters**: module, addendum, dedendum, and whether the flank is an involute (with
   its base circle and pressure angle) or a cycloid (with its rolling circle). A flank is called
   cycloidal only when the involute fit fails, the cycloid fit is within 0.02 module and its pitch
   circle agrees with the one the module gives. Wheels with very many teeth are nearly the same in
   both profiles and are reported as involute.
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

When two toothed parts mesh but the geometry does not show it, declare the mesh:

```
ohs rig mesh movement.hsproj "barrel gear" "centre pinion"
```

Both parts need a role that rotates and a tooth count. A declared mesh is kept when the rig is
re-inferred, and is marked `manual` in the project file.

Without a barrel in the rig, the train starts at the far end of the gear chain from the escape
wheel and `kinematics` says so; pass `--root` to choose another start.

## Read the train

```
ohs kinematics movement.hsproj --expect M4=1      # fails (exit 1) if M4 is not 1 rpm
ohs explain movement.hsproj --train               # mesh by mesh, with the ratios
ohs explain movement.hsproj "escape wheel"        # what a part does, with this movement's numbers
ohs animate movement.hsproj -o train.glb --stepped --duration 10
```

`animate` writes a binary glTF with one node per part and keyframes from the kinematic motion
(`--stepped` advances one escapement step per beat), viewable in any glTF viewer or Blender.
Give the output a `.usda` extension (`-o train.usda`) to get the same motion as an OpenUSD text file
(millimetres, Z up, one `rotateZ` time sample per keyframe) for USD tools, Blender or Omniverse.
