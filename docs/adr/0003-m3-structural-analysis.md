# ADR 3: M3 structural analysis without MFEM, Netgen and VTK

The plan names MFEM, Netgen and VTK. None of them is available in this environment, so M3 builds
the pieces in-house, small and tested against analytical answers. The interfaces (a tetrahedral
mesh, a model of material, constraints and loads, results as fields) are the ones an MFEM or Netgen
adapter would sit behind later.

- **Units.** mm, N, MPa (N/mm²), tonne, s, K. With density in t/mm³ and E in MPa, frequencies
  come out in Hz and a mass in tonnes; `1 g` is 9810 mm/s².
- **Meshing is for 2.5D parts.** Watch parts are plates, bridges and wheels, so the mesher works
  on the horizontal section of a part (`geometry::section_region`) and extrudes it. The 2D step is
  a conforming Delaunay triangulation refined by Ruppert's algorithm to a radius-edge bound of
  1.5 (minimum angle about 19.5°) and to a size field, smoothed lightly. Each prism becomes three
  tetrahedra with the "lower index on top" diagonal rule so neighbours always agree. General 3D
  solids (an undercut, a varying section) need a real volume mesher and are not covered; the
  section approach loses features that vary through the thickness.
- **Elements.** 4-node and 10-node tetrahedra, isoparametric, with 1, 4 and 14 point rules.
  Quadratic tetrahedra are the default: linear ones lock in bending (the verification suite shows
  it). Boundaries are straight-edged, so curved outlines are as accurate as the polyline sampling.
- **Solvers.** Direct sparse LDLᵀ (Eigen) on the free dofs. Modal analysis is block inverse
  iteration with Rayleigh-Ritz on a factorisation of K + sM, which also handles free-free bodies
  (the six rigid modes are found and dropped). Transient is Newmark average acceleration with
  Rayleigh damping and an optional support acceleration. The nonlinear solver is total-Lagrangian
  Newton with a St. Venant-Kirchhoff material: large rotations, small strains; its tangent is
  tested against finite differences of the internal force. Frictionless contact is a penalty
  against a rigid plane with active-set iteration; press fits are penalty ties with a gap. All of
  these refuse to run on meshes beyond a dof limit rather than exhaust memory.
- **Thermal.** Thermal strain loads and thermoelastic stresses, plus the closed-form rate change
  of a hairspring/balance pair, `86400 (e/2 + 3 α_spring/2 − α_balance)` s/day per kelvin.
- **Materials** live in `data/materials/materials.toml`, embedded at configure time, with a source
  note per entry. The values are typical handbook figures, not supplier data; the file says so.
- **Tooth bending** is analysed on a sector of three teeth clamped at the rim, loaded at the tip of
  the middle tooth, and reported next to the Lewis hand calculation. The Lewis stress ignores the
  root fillet, so the FE peak is 1.5–2.2 times higher for a 0.38 module fillet; the generated
  tooth is an involute with a circular fillet, not the cycloidal or hob-cut form of a real
  calibre. The load comes from the M2 torque chain.
- **CalculiX** is GPL and stays out of process. The adapter writes a deck and parses the `.frd`
  displacements; `--crosscheck` runs it when `ccx` is on PATH (or `$OHS_CCX`). It has not been
  run against a real CalculiX here because none is installed; the writer and the fixed-width
  parser are unit-tested.
- **Results** are VTU files for ParaView plus JSON on stdout. HDF5 output and VTK in the UI wait
  for the Qt front end.
- **Verification** (`ohs verify physics [--full]`): cantilever deflection against Timoshenko,
  cantilever frequency, restrained thermal stress, Lamé cylinder, hole stress concentration,
  large-deflection elastica, the Lewis ratio and convergence rates of linear (≥ first order) and
  quadratic (third order) tetrahedra on a manufactured solution. The unit tests add Hertz line
  contact, the shrink-fit pressure, the transient step response and the patch tests.
