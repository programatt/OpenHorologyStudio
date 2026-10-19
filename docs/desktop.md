# Desktop application

`ohs-gui` is a Qt 6 window over the same engine as the command line. Every action it takes is an
`ohs` command run in the same process, so anything you do in the window you can also do in a
script, and the window can save what you did as one.

Build it with `-DOHS_BUILD_GUI=ON` (needs Qt 6.5 or newer with Widgets). See [Install](install.md).

```
ohs-gui movement.hsproj
```

## What is in the window

- **Viewport.** Drag with the left button to orbit, with the right or middle button to pan, scroll
  to zoom, click a part to select it. The View menu has top, isometric and side views.
- **Model dock.** Mobiles and their parts, with each part's role. Untick a part to hide it.
- **Explain dock.** Select a part (in the viewport, the tree or the rig list) and it shows what the
  part does, with the numbers of your movement.
- **Import and Rig.** Open a project, import a STEP file, or open the built-in demonstration
  movement. The rig list shows every part's role, how sure OHS is and what decided it, least sure
  first.
- **Animate.** Play or scrub the going train over 60 seconds of movement time, at any speed, smooth
  or one escapement step per beat. An exploded view and a cut-away plane help you see inside.
- **Simulate, Analyse, Manufacture, Report.** One button per analysis: torque chain, timekeeping,
  virtual timegrapher, cross-check of the models, tooth bending and shock of the selected part,
  design for manufacture, collisions, tolerance stack-up, and the HTML and PDF report. Each runs as
  a job on a worker thread, so the window stays responsive; jobs run one at a time and the Jobs dock
  lists them (double-click a queued job to cancel it). Results show in the Results dock, with a plot
  of rate against amplitude after Timekeeping, and the commands and their messages in the Log dock.
- **Edit, Undo and Redo** step through every change to the project made in the window: *Set role of
  the selected part* and *Declare that the selected part meshes with this one*. Undo puts the project
  file back exactly as it was (the history is snapshots of the file), and the Edit history dock lists
  the steps; click one to return to that state. Opening or importing another project starts a new
  history. Analyses, views and the timeline change no project data and are not part of it.
- **File, Save session as script** writes every command you ran as a shell script that repeats it.

## Not in the window

The viewport draws with Qt's own painter, so it needs no GPU and runs anywhere, but it does not
have the OpenCASCADE OpenGL viewer, PBR materials, motion trails or force arrows, and there is no
MP4 export. Jobs run on a thread of the window's process, not in separate worker processes, so a
running job cannot be cancelled and a crashing solver would take the window with it; the job list
does not survive a restart. The only plot is rate against amplitude, and only role changes can be
undone. Use `ohs animate` for glTF, and `ohs report` for the report. See [Limitations](limitations.md).
