# Desktop application

`ohs-gui` is a Qt 6 window over the same engine as the command line. Every action it takes is an
`ohs` command run in the same process, so anything you do in the window you can also do in a
script, and the window can save what you did as one.

Build it with `-DOHS_BUILD_GUI=ON` (needs Qt 6.4 or newer with Widgets; Ubuntu 24.04's `qt6-base-dev`, 6.4.2, is enough). See [Install](install.md).

```
ohs-gui movement.hsproj
```

## What is in the window

- **Viewport.** Drag with the left button to orbit, with the right or middle button to pan, scroll
  to zoom, click a part to select it. The View menu has top, isometric and side views.
- **Model dock.** Mobiles and their parts, with each part's role. Untick a part to hide it. A part shows its
  label when it has one (press F2 to give it one, or use *Label* under *Selected part*); hover over it for the part name.
- **Explain dock.** Select a part (in the viewport, the tree or the rig list) and it shows what the
  part does, with the numbers of your movement.
- **Import and Rig.** Open a project, import a STEP file, or open the built-in demonstration
  movement. The Rig tab is for finishing the rig, so that the animation and the simulations work: they
  need meshes joining the barrel to the escape wheel.
  - **Walk the train** steps through that chain one mesh at a time, from the barrel. The viewport shows
    only the step: the parts walked so far as they are, the two gears of the step in green with an arrow
    from the one that drives to the one driven, and everything else faded back. The card says what the
    mesh is (teeth, speed ratio, module, confidence). *Previous* and *Next* (Alt+Left, Alt+Right) move
    along the train, and the row of chips above it jumps to any step.
  - **Where the chain stops** (a gear that meshes with nothing yet), the walk ends in an amber step.
    Its card lists the pairs that probably mesh and would join the gap, and clicking one shows it in the
    viewport; *Declare the mesh* adds it. *Pick what it drives...* lets you click the partner in the
    viewport instead, and *Change its role* selects the part so that its role can be corrected. If there
    is no barrel or no escape wheel, the first or last step asks you to click it. After each change the
    walk carries on from the same place, until the card says the train is complete.
  - **Re-run auto-rig** (also under Edit) classifies and rigs the parts again, so newly recognised parts
    join their mobiles and meshes; the roles you set and the meshes you declared are kept (*Re-rig from
    scratch* in the Edit menu forgets those too).
  - Below, **Parts** lists every part's role, how sure OHS is and what decided it, least sure first;
    **To do** lists what else to check (parts that could not be classified or are unsure, toothed parts
    that mesh with nothing), and clicking an entry selects the part; **Suggested meshes** lists every
    pair that probably meshes, with *Declare selected* and *Declare all*.
  - **Show the whole rig instead** draws everything at once: each mobile in its own colour, a line for
    every mesh (green where the barrel feeds it, blue-grey where it does not reach, dashed violet where
    you declared it, amber where the confidence is low), dotted amber lines with a `?` for suggested
    meshes, and a ring round the parts that need a look. The walk is shown only on the Rig tab.
- **Animate.** Play or scrub the going train over 60 seconds of movement time, at any speed, smooth
  or one escapement step per beat. An exploded view and a cut-away plane help you see inside.
- **Simulate, Analyse, Manufacture, Report.** One button per analysis: torque chain, timekeeping,
  virtual timegrapher, cross-check of the models, tooth bending and shock of the selected part,
  design for manufacture, collisions, tolerance stack-up, and the HTML and PDF report. Each runs as
  a job in a worker process (the `ohs` program), so the window stays responsive and a crashing
  solver cannot take it down; jobs run one at a time and the Jobs dock lists them. Double-click a
  queued job to cancel it, or the running one to kill its worker. The queue is kept in a journal
  beside the project (`<project>.jobs.json`): close the window mid-run and the unfinished jobs run
  again when you open the project, and a finished job's output shows again when you select it. Results
  show in the Results dock, with a plot of rate against amplitude after Timekeeping, and the commands
  and their messages in the Log dock.
- **Edit, Undo and Redo** step through every change to the project made in the window: *Set role of
  the selected part*, *Declare that the selected part meshes with this one*, *Declare all* suggested
  meshes and *Re-run auto-rig*. Undo puts the project
  file back exactly as it was (the history is snapshots of the file), and the Edit history dock lists
  the steps; click one to return to that state. Opening or importing another project starts a new
  history. Analyses, views and the timeline change no project data and are not part of it.
- **Field plot.** *Shock and modes of the selected part* also writes its result, and the part is then
  drawn in the viewport as the finite element surface coloured by von Mises stress (or, from the
  *Plot on the part* list, by displacement), with a colour scale. *Clear the field plot* brings the CAD
  surface back. Only the shock case places its mesh in the movement, so the tooth case has no plot.
  From the command line: `ohs-gui project.hsproj --snapshot out.png --field result.vtu,part,von_mises`.
- **Tooth force arrows and motion trails** (Animate workspace, or `--forces` and `--trails SECONDS`
  with `--snapshot` or `--video`). An arrow at each mesh on the path from the barrel to the escape
  wheel shows the tangential tooth force at full wind, computed from the torque chain and the pitch
  radius, with its value in mN. A trail follows a marker on the rim of every turning mobile over the
  chosen movement time, at most one turn, so the length of a trail shows the speed of the wheel.
- **File, Export video** renders the animation to an MP4 (or WebM or GIF) with ffmpeg, which must be
  installed (or set `OHS_FFMPEG`): the movement time shown, the length and the frame rate are the
  options of `ohs-gui project.hsproj --video out.mp4 --movement-seconds 60 --video-seconds 10 --fps 30
  --size 1280x720` (add `--view`, `--explode`), which renders and exits without opening a window.
- **File, Save session as script** writes every command you ran as a shell script that repeats it.

## Not in the window

The viewport draws with Qt's own painter, so it needs no GPU and runs anywhere, but it does not
have the OpenCASCADE OpenGL viewer or PBR materials. The flat-shaded triangles, the field plot and the
overlays are the whole of it. A job that was running when the window closed starts again from the
beginning rather than carrying on (the solvers do not checkpoint). Views cannot be undone, only
edits of the project. Use `ohs animate` for glTF or USD, and `ohs report` for the report. See
[Limitations](limitations.md).
