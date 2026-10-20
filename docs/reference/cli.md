# Command-line reference

Generated from `ohs --help` by `scripts/gen_cli_reference.py`. Every command accepts the global `--json` flag; exit code 0 is success, 1 a failed check, 2 bad input.

## `ohs`

```text
Open Horology Studio: watch movement design and analysis
Usage: ohs [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit
  --version                   Display program version information and exit
  --json                      Machine-readable output
  -v,--verbose                Debug logging

Subcommands:
  import                      Import a STEP, IGES or DXF file and auto-rig it
  rig                         Inspect a rig
  animate                     Export the running train as an animated binary glTF (.glb) or OpenUSD text file (.usda)
  explain                     Explain a part or walk through the gear train with this movement's numbers
  kinematics                  Rigid-body ratios and speeds of the going train
  serve                       Run commands for `ohs-client` from a resident process (Unix sockets): no start-up, warm geometry
  dev                         Developer tools
  thirdparty                  Third-party data (never committed to the repo)
  torque                      Torque chain from the barrel to the escape wheel
  simulate                    Simulate timekeeping (reduced event-driven or detailed contact model)
  timegrapher                 Virtual timegrapher: rate, amplitude, beat error and trace
  timekeeping                 Amplitude, rate, isochronism, positions, power reserve and energy budget
  whatif                      Compare the baseline with a modified design from a study file
  verify                      Cross-checks of the simulation
  bench                       Compare or calibrate against timegrapher readings from a real movement
  calibre                     Calibre parameter files
  materials                   List the material catalogue
  thermal                     Rate change per kelvin of a hairspring and balance pair
  fea                         Finite-element check of a part: tooth bending or shock and modes
  gear                        Generate an involute or cycloidal wheel as STEP, DXF, SVG or PDF drawing
  draw                        Dimensioned drawing of one part of a project (SVG, DXF or PDF)
  tolerance                   Monte Carlo tolerance stack-up: backlash, engagement, end and side shake
  dfm                         Design-for-manufacturing rules for your processes
  collisions                  Sweep the moving train for overlaps and close approaches
  report                      Write an HTML and PDF report of the analyses
  lesson                      Guided lessons on a movement (OM10 or the built-in demo)
```

### `ohs import`

```text
Import a STEP, IGES or DXF file and auto-rig it
Usage: ohs import [OPTIONS] step

Positionals:
  step TEXT:FILE REQUIRED     CAD file: .step, .stp, .iges, .igs or .dxf (a DXF profile is extruded by its own thickness, else 1 mm)

Options:
  -h,--help                   Print this help message and exit
  -o,--output TEXT REQUIRED   Project (rig) file to write
  --auto-rig,--no-auto-rig{false}
                              Infer roles, mobiles and meshes (default on)
  --preset TEXT:FILE          Role preset TOML (name regex -> role)
  --up TEXT                   CAD axis normal to the dial: x, y, z or auto (default: auto, the axis the parts are thinnest along)
  --beat-rate FLOAT           Beat rate in vph recorded in the project
  --min-teeth INT [6]         Fewest teeth to call a part toothed
```

### `ohs rig`

```text
Inspect a rig
Usage: ohs rig [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  check                       Validate a project's rig
  set                         Override a part's role and re-infer mobiles and meshes
  label                       Give a part a label to show in place of its name (none clears it)
  beat-rate                   Set the beat rate of the project, in vibrations per hour
  solve                       Find what drives the loose end of the train, and which wheels turn the hands
  unmesh                      Take away a mesh that was declared by hand
  winding                     Leave a mesh out of the going train (the winding work), or put it back
  mesh                        Declare that two toothed parts mesh although the geometry does not show it
  auto                        Run the auto-rig again on an imported project: roles, mobiles and meshes
  suggest                     List toothed parts that probably mesh but were not accepted, and optionally declare them
  walk                        Step through the train from the barrel to the escape wheel, mesh by mesh, and show what is missing
```

#### `ohs rig check`

```text
Validate a project's rig
Usage: ohs rig check [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --strict                    Treat warnings as failures
```

#### `ohs rig set`

```text
Override a part's role and re-infer mobiles and meshes
Usage: ohs rig set [OPTIONS] project [parts...]

Positionals:
  project TEXT:FILE REQUIRED  Project file
  parts TEXT ...              Parts to give the --role (any number)

Options:
  -h,--help                   Print this help message and exit
  --role TEXT                 plate, bridge, barrel, arbor, wheel, pinion, escape_wheel, ...
  --assign TEXT ...           PART=ROLE, repeatable: several different roles in one edit
  --min-teeth INT [6]         Fewest teeth to call a part toothed
```

#### `ohs rig label`

```text
Give a part a label to show in place of its name (none clears it)
Usage: ohs rig label [OPTIONS] project [part] [label]

Positionals:
  project TEXT:FILE REQUIRED  Project file
  part TEXT                   Part name
  label TEXT                  The label; leave out to remove it

Options:
  -h,--help                   Print this help message and exit
  --set TEXT ...              PART=LABEL, repeatable: several labels in one edit
```

#### `ohs rig beat-rate`

```text
Set the beat rate of the project, in vibrations per hour
Usage: ohs rig beat-rate [OPTIONS] project rate

Positionals:
  project TEXT:FILE REQUIRED  Project file
  rate FLOAT REQUIRED         Vibrations per hour: 18000, 21600, 25200, 28800 ...

Options:
  -h,--help                   Print this help message and exit
```

#### `ohs rig solve`

```text
Find what drives the loose end of the train, and which wheels turn the hands
Usage: ohs rig solve [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --beat-rate FLOAT           Beat rate in vph (the project's when left out)
  --min-teeth INT [6]         Fewest teeth to call a part toothed
  --apply                     Declare the mesh of the best candidate
  --pick INT [0]              With --apply: the candidate to declare, as numbered in the list
```

#### `ohs rig unmesh`

```text
Take away a mesh that was declared by hand
Usage: ohs rig unmesh [OPTIONS] project part partner

Positionals:
  project TEXT:FILE REQUIRED  Project file
  part TEXT REQUIRED          One of the two parts
  partner TEXT REQUIRED       The other

Options:
  -h,--help                   Print this help message and exit
```

#### `ohs rig winding`

```text
Leave a mesh out of the going train (the winding work), or put it back
Usage: ohs rig winding [OPTIONS] project part partner

Positionals:
  project TEXT:FILE REQUIRED  Project file
  part TEXT REQUIRED          One of the two meshed parts
  partner TEXT REQUIRED       The other

Options:
  -h,--help                   Print this help message and exit
  --in-train                  Put the mesh back in the train
```

#### `ohs rig mesh`

```text
Declare that two toothed parts mesh although the geometry does not show it
Usage: ohs rig mesh [OPTIONS] project [part] [part_b]

Positionals:
  project TEXT:FILE REQUIRED  Project file
  part TEXT                   First part
  part_b TEXT                 Second part

Options:
  -h,--help                   Print this help message and exit
  --pair TEXT ...             A=B, repeatable: several meshes in one edit
```

#### `ohs rig auto`

```text
Run the auto-rig again on an imported project: roles, mobiles and meshes
Usage: ohs rig auto [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --reset                     Also forget the roles you set and the meshes you declared
  --min-teeth INT [6]         Fewest teeth to call a part toothed
  --coaxial-tolerance FLOAT [0.05] 
                              Largest centre offset in mm for parts of one mobile
```

#### `ohs rig suggest`

```text
List toothed parts that probably mesh but were not accepted, and optionally declare them
Usage: ohs rig suggest [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --apply                     Declare the suggested meshes (as 'ohs rig mesh' does)
  --min-confidence FLOAT [0]  Leave out suggestions below this confidence (0 to 0.5)
  --min-teeth INT [6]         Fewest teeth to call a part toothed
```

#### `ohs rig walk`

```text
Step through the train from the barrel to the escape wheel, mesh by mesh, and show what is missing
Usage: ohs rig walk [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --min-teeth INT [6]         Fewest teeth to call a part toothed
```

### `ohs animate`

```text
Export the running train as an animated binary glTF (.glb) or OpenUSD text file (.usda)
Usage: ohs animate [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  -o,--output TEXT REQUIRED   File to write: .glb (glTF 2.0) or .usda (OpenUSD)
  --step TEXT                 STEP file (default: the project's source next to it)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --duration FLOAT [10]       Animation length in seconds
  --fps FLOAT [30]            Minimum keyframes per second
  --time-scale FLOAT [600]    Movement seconds per animation second
  --stepped                   Advance the train one escapement step per beat
```

### `ohs explain`

```text
Explain a part or walk through the gear train with this movement's numbers
Usage: ohs explain [OPTIONS] project [part]

Positionals:
  project TEXT:FILE REQUIRED  Project file
  part TEXT                   Part name

Options:
  -h,--help                   Print this help message and exit
  --train                     Walk through the going train step by step
  --beat-rate FLOAT           Beat rate in vph (default: from project)
```

### `ohs kinematics`

```text
Rigid-body ratios and speeds of the going train
Usage: ohs kinematics [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --root TEXT                 Root mobile name (default: the barrel's mobile)
  --expect TEXT ...           Check a mobile's speed, NAME=RPM (repeatable)
```

### `ohs serve`

```text
Run commands for `ohs-client` from a resident process (Unix sockets): no start-up, warm geometry
Usage: ohs serve [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --socket TEXT               Socket path (default: $OHS_SOCKET, else $XDG_RUNTIME_DIR/ohs.sock)
  --warm TEXT ...             A project whose geometry is read now and kept for every request; repeatable
```

### `ohs dev`

```text
Developer tools
Usage: ohs dev [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  gen-movement                Write the procedural test movement as STEP
  crash                       Crash the process on purpose (tests that `ohs serve` survives a crashing command)
  sleep                       Wait, then finish (tests cancelling a long command)
```

#### `ohs dev gen-movement`

```text
Write the procedural test movement as STEP
Usage: ohs dev gen-movement [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -o,--output TEXT REQUIRED   STEP file to write
```

#### `ohs dev crash`

```text
Crash the process on purpose (tests that `ohs serve` survives a crashing command)
Usage: ohs dev crash [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
```

#### `ohs dev sleep`

```text
Wait, then finish (tests cancelling a long command)
Usage: ohs dev sleep [OPTIONS] [seconds]

Positionals:
  seconds FLOAT [1]           How long

Options:
  -h,--help                   Print this help message and exit
```

### `ohs thirdparty`

```text
Third-party data (never committed to the repo)
Usage: ohs thirdparty [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  om10                        openmovement OM10
```

#### `ohs thirdparty om10`

```text
openmovement OM10
Usage: ohs thirdparty om10 [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  fetch                       Install and import a user-supplied OM10 STEP file. Options after the command go to scripts/thirdparty/fetch_om10.py: --file PATH, --accept, --no-import, --ohs PATH
```

##### `ohs thirdparty om10 fetch`

```text
Install and import a user-supplied OM10 STEP file. Options after the command go to scripts/thirdparty/fetch_om10.py: --file PATH, --accept, --no-import, --ohs PATH
Usage: ohs thirdparty om10 fetch [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
```

### `ohs torque`

```text
Torque chain from the barrel to the escape wheel
Usage: ohs torque [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --wind FLOAT [1]            State of wind, 1 = full, 0 = run down
```

### `ohs simulate`

```text
Simulate timekeeping (reduced event-driven or detailed contact model)
Usage: ohs simulate [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --level TEXT [reduced]      reduced or detailed
  --engine TEXT [builtin]     Engine for --level detailed: builtin or chrono (needs a Chrono build)
  --duration TEXT [60s]       Simulated time: 90s, 10m, 48h (over 10 minutes follows the run-down)
  --position TEXT [dial_up]   dial_up, dial_down, crown_up, crown_down, crown_left, crown_right
  --wind FLOAT [1]            State of wind for runs up to 10 minutes
  -o,--out TEXT               Write the result as JSON
  --events TEXT               Write every escapement event as CSV (short runs)
```

### `ohs timegrapher`

```text
Virtual timegrapher: rate, amplitude, beat error and trace
Usage: ohs timegrapher [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --duration TEXT [60s]       Listening time
  --position TEXT [dial_up]   Orientation
  --wind FLOAT [1]            State of wind
  --jitter-us FLOAT [0]       Timing noise in microseconds rms
  --seed UINT [1]             Noise seed
  --csv TEXT                  Write the beats as CSV
  --json-out TEXT             Also write the readout as JSON to this file (one run for the text and the data)
  --no-trace                  Print only the readout
```

### `ohs timekeeping`

```text
Amplitude, rate, isochronism, positions, power reserve and energy budget
Usage: ohs timekeeping [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --json-out TEXT             Also write the result as JSON to this file (one run for the text and the data)
```

### `ohs whatif`

```text
Compare the baseline with a modified design from a study file
Usage: ohs whatif [OPTIONS] project study

Positionals:
  project TEXT:FILE REQUIRED  Project file
  study TEXT:FILE REQUIRED    Study TOML

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --verify                    Also check both designs against the detailed model
```

### `ohs verify`

```text
Cross-checks of the simulation
Usage: ohs verify [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  levels                      Detailed contact model against the reduced event-driven model
  physics                     Re-run the solvers against analytical answers on this machine
```

#### `ohs verify levels`

```text
Detailed contact model against the reduced event-driven model
Usage: ohs verify levels [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --wind FLOAT [1]            State of wind
  --engine TEXT [builtin]     Engine for the detailed model: builtin or chrono (needs a Chrono build)
  --position TEXT [dial_up]   Orientation
  --amplitude-tolerance FLOAT [2] 
                              Allowed amplitude difference in percent
  --rate-tolerance FLOAT [1]  Allowed rate difference in s/day
```

#### `ohs verify physics`

```text
Re-run the solvers against analytical answers on this machine
Usage: ohs verify physics [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --full                      Finer meshes, benchmarks and convergence rates (slower)
```

### `ohs bench`

```text
Compare or calibrate against timegrapher readings from a real movement
Usage: ohs bench [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  compare                     Simulate each reading and compare
  calibrate                   Fit pivot friction (and air damping) to measured amplitudes
```

#### `ohs bench compare`

```text
Simulate each reading and compare
Usage: ohs bench compare [OPTIONS] project readings

Positionals:
  project TEXT:FILE REQUIRED  Project file
  readings TEXT:FILE REQUIRED CSV: hours,position,amplitude_deg,rate_s_per_day,beat_error_ms

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
```

#### `ohs bench calibrate`

```text
Fit pivot friction (and air damping) to measured amplitudes
Usage: ohs bench calibrate [OPTIONS] project readings

Positionals:
  project TEXT:FILE REQUIRED  Project file
  readings TEXT:FILE REQUIRED Readings CSV

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Calibre parameters (TOML); default: a reference lever calibre
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --fit-damping               Also fit the air damping
  -o,--output TEXT            Write the calibrated calibre as TOML
```

### `ohs calibre`

```text
Calibre parameter files
Usage: ohs calibre [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  init                        Write the default calibre parameters to edit
  hairspring                  Fit a calibre's hairspring from a beam model of a flat spiral: stiffness and the cubic term
```

#### `ohs calibre init`

```text
Write the default calibre parameters to edit
Usage: ohs calibre init [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -o,--output TEXT            File to write (default: stdout)
  --beat-rate FLOAT [0]       Beat rate in vph
```

#### `ohs calibre hairspring`

```text
Fit a calibre's hairspring from a beam model of a flat spiral: stiffness and the cubic term
Usage: ohs calibre hairspring [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --calibre TEXT:FILE         Start from this calibre (default: the reference one)
  --youngs FLOAT [210]        Young's modulus in GPa
  --width FLOAT [0.14]        Strip height along the staff in mm
  --thickness FLOAT [0.025]   Strip radial thickness in mm
  --inner-radius FLOAT [0.6]  Radius of the inner end, mm
  --pitch FLOAT [0.12]        Distance between coils, mm
  --turns FLOAT [12]          Number of coils
  --amplitude FLOAT [300]     Amplitude to fit over, degrees
  --beat-rate FLOAT [0]       Beat rate in vph for the rest of the calibre
  -o,--output TEXT            Write the calibre with the fitted spring as TOML
```

### `ohs materials`

```text
List the material catalogue
Usage: ohs materials [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --materials TEXT:FILE       Project materials TOML (added to the catalogue)
```

### `ohs thermal`

```text
Rate change per kelvin of a hairspring and balance pair
Usage: ohs thermal [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --hairspring TEXT [nife_hairspring] 
                              Hairspring material
  --balance TEXT [brass_leaded] 
                              Balance material
  --materials TEXT:FILE       Project materials TOML
```

### `ohs fea`

```text
Finite-element check of a part: tooth bending or shock and modes
Usage: ohs fea [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --case TEXT [tooth-bending] 
                              tooth-bending or shock
  --part TEXT REQUIRED        Part name
  --material TEXT             Material name (default: brass, steel for pinions)
  --materials TEXT:FILE       Project materials TOML
  --load-n FLOAT              Tangential tooth load in N (default: from the torque chain)
  --wind FLOAT [1]            State of wind for the torque chain
  --size-factor FLOAT [0.14]  Root element size in modules
  --size FLOAT [0.2]          Element size in mm for the shock case
  --g FLOAT [5000]            Shock acceleration in g
  --in-plane                  Shock along the dial instead of end shake
  --modes INT                 Also compute this many natural frequencies
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --mesher TEXT [section]     Shock case mesher: section (mid-height section, extruded) or netgen (the whole B-rep; needs 'pip install netgen-mesher')
  --solver TEXT [direct]      Shock case linear solver: direct (OHS's own) or mfem (MFEM's conjugate gradients; one material, needs a build with MFEM)
  --vtu TEXT                  Write the result for ParaView
  --vtu-compressed TEXT       Write the result as a binary, zlib-compressed VTU written by VTK (needs a build with VTK)
  --hdf5 TEXT                 Write mesh and fields to an HDF5 file (needs a build with libhdf5)
  --crosscheck                Compare with CalculiX if it is installed
  --mfem                      Compare with MFEM, an independent finite element code, if this build has it (shock case)
  --beat-rate FLOAT           Beat rate in vph (default: from project)
```

### `ohs gear`

```text
Generate an involute or cycloidal wheel as STEP, DXF, SVG or PDF drawing
Usage: ohs gear [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --teeth INT REQUIRED        Number of teeth
  --module FLOAT REQUIRED     Module in mm
  --profile TEXT [involute]   Tooth profile: involute or cycloidal
  --pressure-angle FLOAT [20] 
                              Pressure angle in degrees (involute)
  --rolling-radius FLOAT [3]  Rolling circle radius in modules (cycloidal; wheels that mesh share it)
  --addendum FLOAT [1]        Addendum in modules
  --dedendum FLOAT [1.25]     Dedendum in modules
  --fillet FLOAT [0.38]       Root fillet radius in modules
  --bore FLOAT [0]            Bore radius in mm
  --thickness FLOAT [0.5]     Wheel thickness in mm
  --material TEXT [brass_leaded] 
                              Material note for the drawing
  --name TEXT [gear]          Part name
  --step-out TEXT             Write a STEP solid
  --dxf TEXT                  Write a DXF drawing
  --svg TEXT                  Write an SVG drawing
  --pdf TEXT                  Write a PDF drawing at a standard scale
  --profile-only              DXF/SVG/PDF carry only the outline, without views and dimensions (for cutting set-up)
```

### `ohs draw`

```text
Dimensioned drawing of one part of a project (SVG, DXF or PDF)
Usage: ohs draw [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --part TEXT REQUIRED        Part name
  -o,--output TEXT            File ending in .svg, .dxf or .pdf
  --material TEXT [brass_leaded] 
                              Material note
```

### `ohs tolerance`

```text
Monte Carlo tolerance stack-up: backlash, engagement, end and side shake
Usage: ohs tolerance [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --samples INT [10000]       Number of virtual builds
  --seed UINT [42]            Random seed
  --pmi                       Take pivot, hole, arbor-length and bridge-gap tolerances from the STEP file's PMI (AP242 dimensional tolerances), on top of the table
  --tolerances TEXT:FILE      Tolerance table TOML ([tolerance])
  --fail-above FLOAT [0.02]   Exit 1 when more than this fraction of builds has a problem
```

### `ohs dfm`

```text
Design-for-manufacturing rules for your processes
Usage: ohs dfm [OPTIONS] [project]

Positionals:
  project TEXT:FILE           Project file

Options:
  -h,--help                   Print this help message and exit
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --process TEXT [mill,wheel-cutting] 
                              Comma-separated processes (ohs dfm --list)
  --rules TEXT:FILE           Your own rule sets (TOML, same format as data/dfm/rules.toml)
  --list                      List the processes and exit
  --plugin TEXT:FILE ...      A DFM plugin (shared library, see docs/plugins.md); repeatable
```

### `ohs collisions`

```text
Sweep the moving train for overlaps and close approaches
Usage: ohs collisions [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  --beat-rate FLOAT           Beat rate in vph (default: from project)
```

### `ohs report`

```text
Write an HTML and PDF report of the analyses
Usage: ohs report [OPTIONS] project

Positionals:
  project TEXT:FILE REQUIRED  Project file

Options:
  -h,--help                   Print this help message and exit
  --step TEXT                 STEP file (default: next to the project)
  --up TEXT                   CAD axis normal to the dial (default: the one found at import)
  -o,--output TEXT            Output path without extension (default: next to the project)
  --format TEXT [html,pdf]    html, pdf or html,pdf
  --process TEXT [mill,wheel-cutting] 
                              Processes for the DFM section
  --rules TEXT:FILE           Your own DFM rule sets
  --plugin TEXT:FILE ...      A DFM plugin library; repeatable
  --tolerances TEXT:FILE      Tolerance table TOML
  --calibre TEXT:FILE         Calibre parameters TOML
  --samples INT [10000]       Tolerance samples
  --seed UINT [42]            Tolerance seed
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --no-fea                    Skip the structural section
  --no-timekeeping            Skip the timekeeping section
```

### `ohs lesson`

```text
Guided lessons on a movement (OM10 or the built-in demo)
Usage: ohs lesson [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  list                        The lessons
  show                        Read a lesson with your movement's numbers
  check                       Check your exercise
```

#### `ohs lesson list`

```text
The lessons
Usage: ohs lesson list [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
```

#### `ohs lesson show`

```text
Read a lesson with your movement's numbers
Usage: ohs lesson show [OPTIONS] id

Positionals:
  id TEXT REQUIRED            Lesson id (ohs lesson list)

Options:
  -h,--help                   Print this help message and exit
  --project TEXT:FILE         Project file (default: a built-in demo movement)
  --calibre TEXT:FILE         Calibre parameters TOML
  --beat-rate FLOAT           Beat rate in vph (default: from project)
```

#### `ohs lesson check`

```text
Check your exercise
Usage: ohs lesson check [OPTIONS] id

Positionals:
  id TEXT REQUIRED            Lesson id (ohs lesson list)

Options:
  -h,--help                   Print this help message and exit
  --project TEXT:FILE         Project file (default: a built-in demo movement)
  --calibre TEXT:FILE         Calibre parameters TOML
  --beat-rate FLOAT           Beat rate in vph (default: from project)
  --answer FLOAT              Your numeric answer
  --study TEXT:FILE           Your what-if study (TOML)
  --cutout TEXT               Window WIDTHxHEIGHT in mm
  --solution                  Print a working answer instead of checking yours
```
