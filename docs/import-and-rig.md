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
   a short hollow cylinder is a `bearing` (a brass tube or bush), a long round rod lying in the plane
   of the movement the `stem`, a wide thin part a plate. A thin perforated plate whose far end rests
   on the teeth of a wheel, ratchet or date ring is a `lever` (a click or jumper), and a strip or a
   bent wire pressing on one a `spring`; these guesses carry a confidence of 0.5 or less. The stem,
   levers and springs move on axes of their own, not as mobiles of the train (see
   [Limitations](limitations.md)).
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

## Many commands: the resident process

Every `ohs` call loads its libraries (about 60 ms) before it does anything, which is most of the time of a rig command.
For a script of hundreds of commands, run `ohs serve` once and call `ohs-client` in place of `ohs`:

```
ohs serve --warm movement.hsproj &        # reads the project's geometry once, then waits on a Unix socket
ohs-client rig check movement.hsproj       # 29 ms on the 166-part OM10, against 128 ms
ohs-client dfm movement.hsproj
```

`ohs-client` takes the same words as `ohs`, passes its working directory, and prints what the command printed and returns
its exit code. Each request runs in a forked child of the daemon, so a command that crashes does not stop it, and the
child starts with the libraries and the warm geometry loaded. With no daemon listening, `ohs-client` runs `ohs`
itself, so a script works either way (`OHS_NO_DAEMON=1` forces that). The socket is `$XDG_RUNTIME_DIR/ohs.sock`, or
`--socket`/`$OHS_SOCKET`, readable by you only. Linux and macOS; not Windows.

## Check and correct

```
ohs rig check movement.hsproj          # every part, role, confidence, gear fit and warning
ohs rig set movement.hsproj "third wheel" --role pinion
ohs rig set movement.hsproj Part_01 Part_02 Part_03 --role screw       # one role for several parts, in one edit
ohs rig set movement.hsproj --assign Part_04=wheel --assign Part_05=pinion
```

`ohs rig label --set PART=LABEL` and `ohs rig mesh --pair A=B` do the same for labels and meshes. Each is one edit:
if one part is not there, nothing is written.

A part name from a CAD file is often a number. `ohs rig label movement.hsproj Part_017 "Third wheel"` gives
the part a label, stored in the project, that the desktop application shows in place of the name (hover over it
to see the name); leave the label out to remove it.

The train runs from the barrel to the escape wheel. A mesh that turns the barrel instead of being turned by it,
the ratchet or click wheel of the winding work, is not a link of it: meshes with a part whose name or label
says ratchet, click, crown, winding or clutch are left out automatically. For any other mesh that is the wrong way
round, `ohs rig winding movement.hsproj "part" "partner"` leaves it out of the train (the walk and the animation then
go another way) and `--in-train` puts it back. The choice is kept when the rig is run again.

`ohs rig solve movement.hsproj` closes a broken train. From the escape wheel and the beat rate it works out how many
times an hour each wheel joined to the escape wheel turns, so it can name the wheels that turn the seconds hand (60
turns an hour), the minute hand (1) and the hour hand (1/12). If the train stops short, it looks for the part that
drives the gear at the loose end: a candidate must sit in the same layer, at the centre distance that the gear's
module gives, and have a tip radius that fits. A part whose teeth were not found (a smooth-looking barrel drum) has
them worked out from that distance, and the result says so. `--apply` declares the best candidate's mesh and stores
the worked-out tooth count. The Rig tab's *Solve the train* button does the same after asking.

`rig check --strict` fails on warnings; use it in scripts. Setting a role re-infers mobiles and
meshes, and your choice is recorded with confidence 1 and source `user`.

When two toothed parts mesh but the geometry does not show it, declare the mesh:

```
ohs rig mesh movement.hsproj "barrel gear" "centre pinion"
```

Both parts need a role that rotates and a tooth count. A declared mesh is kept when the rig is
re-inferred, and is marked `manual` in the project file.

`rig check` ends with the train: the chain of mobiles from the barrel to the escape wheel, or, when no
mesh joins them, the two pieces with the gap marked `[no mesh]`, and a warning that the train is broken.
The animation and the simulations need it connected.

## Rig again, and find what is left

```
ohs rig auto movement.hsproj           # classify and rig every part again
ohs rig auto movement.hsproj --reset   # ... and forget the roles you set and the meshes you declared
ohs rig suggest movement.hsproj        # pairs that probably mesh but failed the strict test
ohs rig suggest movement.hsproj --apply --min-confidence 0.3
ohs rig walk movement.hsproj           # the train mesh by mesh from the barrel, and what is missing
```

`rig walk` numbers the meshes from the barrel to the escape wheel (which part drives which, the tooth
counts and the speed ratio) and ends either with "the train is complete" or at the step that is missing,
with the `ohs rig mesh` commands for the pairs that would fill it. It is the same walk as the Rig tab of
the desktop app.

`rig auto` runs the classification and the mesh inference again from the part features stored in the
project, without re-importing the CAD file. Roles you set (`user`) or a preset gave, and meshes you
declared, are kept; every other role is decided afresh, so a part that is newly recognised joins its mobile
and the meshes it makes. It lists the roles that changed. `--coaxial-tolerance` (mm) loosens the test for
parts of one mobile.

`rig suggest` lists toothed parts on different mobiles that overlap in height and whose tip circles are
close to those of a meshing pair for the module their centre distance gives, but that the strict test
rejected, with a likelihood of at most 0.5. Only pairs in which a part meshes with nothing yet are offered.
`--apply` declares them (like `rig mesh`); confirm them by eye first, or in the desktop app's Rig tab,
where they are drawn over the movement.

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
