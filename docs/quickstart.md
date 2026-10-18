# Quickstart

This tour needs no CAD file: OHS can generate a small going train of its own. Everything shown
works the same on a movement you import from your CAD tool.

## 1. Make and import a movement

```
ohs dev gen-movement -o train.step
ohs import train.step -o train.hsproj --beat-rate 25200
ohs rig check train.hsproj
```

`import` reads the STEP assembly, finds the toothed wheels, groups parts that turn together into
mobiles, infers which wheels mesh, and writes a project file (`.hsproj`, plain TOML). `rig check`
lists every inference with a confidence and what you should confirm.

## 2. Read the train

```
ohs kinematics train.hsproj --expect M4=1
ohs explain train.hsproj --train
```

The train gives 25,200 beats per hour, the escape wheel turns 840 times an hour, and one mobile
turns once a minute: the seconds wheel. `explain` shows the ratio walkthrough one mesh at a time.

## 3. Timekeeping

```
ohs torque train.hsproj                       # torque from the barrel to the escape wheel
ohs timekeeping train.hsproj                  # amplitude, rate, positions, power reserve
ohs timegrapher train.hsproj --duration 30s   # the trace a timegrapher would draw
```

The defaults describe a typical lever calibre. To use your own, write the numbers down once:

```
ohs calibre init -o mine.toml                 # edit it, then add --calibre mine.toml to any command
```

## 4. Ask what if

```
cat > longer.toml <<'EOF'
[study]
name = "longer hairspring"

[[change]]
parameter = "hairspring_length_mm"
scale = 1.002
EOF
ohs whatif train.hsproj longer.toml
```

You get the rate, amplitude, isochronism, positional spread and power reserve of both designs side
by side, and a sentence for each change with the relation behind it (here: frequency goes as the
square root of stiffness over inertia).

## 5. Strength and manufacturing

```
ohs fea train.hsproj --part third_wheel                  # tooth bending, with the Lewis hand calculation
ohs tolerance train.hsproj --samples 10000               # backlash, engagement, end and side shake
ohs dfm train.hsproj --process wheel-cutting
ohs report train.hsproj -o train-report                  # HTML and PDF with pass / warn / fail
```

## 6. Make a part

```
ohs gear --teeth 24 --module 0.3 --bore 1.0 --thickness 0.4 --name wheel24 \
         --step-out wheel24.step --dxf wheel24.dxf --pdf wheel24.pdf
```

The PDF is a dimensioned drawing at a standard scale; `--profile-only` writes just the outline for
a cutting set-up. See [manufacturing](manufacturing.md).

## 7. Learn

```
ohs lesson list
ohs lesson show hairspring
```

Each lesson ends with an exercise you can check. See [lessons](lessons.md).
