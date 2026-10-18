# Open Horology Studio

Open Horology Studio (OHS, command `ohs`) turns a CAD model of a watch movement into something you
can reason about. You keep modelling in Fusion 360, SolidWorks, FreeCAD or any other tool, export
STEP, and OHS recognises the wheels, pinions and escapement, then:

- **animates and explains** the going train ([import and rig](import-and-rig.md)),
- **simulates timekeeping**: torque chain, amplitude, rate, isochronism, positional error and a
  virtual timegrapher, with what-if studies ([timekeeping](timekeeping.md)),
- **checks strength**: finite elements for teeth and parts under shock, modal analysis,
  verified against hand calculations ([structural](structural.md)),
- **checks manufacturability** for *your* tools: tolerance stack-up, DFM rules, collisions, and
  generates wheels as STEP and dimensioned drawings ([manufacturing](manufacturing.md)),
- **teaches** as you go: guided lessons on your own movement ([lessons](lessons.md)).

The same engine is available from the command line, from Python ([Python API](python.md)) and,
for rules and checks you write yourself, through a stable [plugin ABI](plugins.md).

## Who it is for

An amateur watchmaker with a lathe, a mill, a wheel-cutting engine and a timegrapher, who wants
both ease of use and real engineering depth. Every number the tool prints comes with the relation
behind it, and `ohs lesson` walks you through the five modifications most people attempt first.

## A first look

```
ohs dev gen-movement -o train.step           # a procedurally generated going train, no CAD file needed
ohs import train.step -o train.hsproj --beat-rate 25200
ohs kinematics train.hsproj                  # 25,200 vph, one revolution per minute at the fourth wheel
ohs timekeeping train.hsproj                 # amplitude, rate, positions, power reserve
ohs lesson show hairspring                   # the lesson, with the numbers of this movement
```

[Quickstart](quickstart.md) walks through it; [the OM10 tutorial](om10.md) does the same with the
open-source OM10 movement.

## What to trust

OHS is built to be honest about what it knows. The solvers are verified against analytical
answers on your own machine (`ohs verify physics --full`), reduced and detailed timekeeping models
are cross-checked against each other (`ohs verify levels`), and every report marks each result as
pass, warning or fail against stated criteria. The default calibre parameters, tolerances and
material values are typical figures, not measurements of your movement: replace them with yours
(`ohs calibre init`, `--tolerances`, `--materials`) and calibrate against a timegrapher
(`ohs bench calibrate`). [Limitations](limitations.md) lists what is not covered.

## Licence

Apache-2.0. The OM10 plans are CC BY-SA 3.0 and are never part of this repository: you download
them yourself (`ohs thirdparty om10 fetch`), and anything derived from them carries their notice.
