# Structural analysis

OHS ships its own small finite-element stack (see ADR 3 for why): a quality mesher for flat parts,
4- and 10-node tetrahedra, and static, modal, transient, geometrically nonlinear and contact
solvers. Quadratic tetrahedra are the default because linear ones lock in bending.

## Tooth bending

```
ohs fea movement.hsproj --part "third wheel"
ohs fea movement.hsproj --part "third wheel" --load-n 0.05 --material steel_hardened --vtu tooth.vtu
```

The load is the torque of the part's mobile (from the [torque chain](timekeeping.md)) over the
pitch radius, applied at the tip of one tooth. The model is a sector of three teeth clamped at the
rim. The output puts the FE root stress next to the **Lewis hand calculation**; the FE value is
higher by the stress concentration of the root fillet (about 1.5 to 2.2 for a 0.38 module fillet).
The tooth is an idealised involute (or cycloid, for a wheel fitted as cycloidal) with a circular fillet,
not the form of your cutter. The Lewis hand calculation uses the involute form factor, so for a
cycloidal wheel it is only a rough guide.

## Shock and vibration of a part

```
ohs fea movement.hsproj --part "train bridge" --case shock --g 5000 --size 0.2 --modes 4 --vtu bridge.vtu
```

The part is cut at mid-height, meshed, and extruded; every hole (pivots, screws) is clamped; the
shock is a body force (`--in-plane` for a lateral shock). Output: mass, largest deflection, peak
von Mises stress, safety factor against the material's strength, and the lowest natural
frequencies. `--vtu` writes a file for ParaView; `--hdf5` writes the mesh and every field to an HDF5 file
for Python (`h5py`) or MATLAB (needs a build with libhdf5; the Linux packages have it).

By default the mesh comes from the mid-height section, extruded, which is exact for a prismatic part
and fast. `--mesher netgen` meshes the whole B-rep with Netgen instead, so counterbores, steps, bosses
and chamfers through the thickness are meshed where they are (it uses far more elements: expect 10
times the dofs, and set `--size` accordingly). Netgen runs in a Python process (`pip install
netgen-mesher` into the `python3` on your PATH, or point `OHS_NETGEN_PYTHON` at the interpreter that
has it); OHS links nothing from it. The holes that are clamped are still the round holes found in the
mid-height section.

`--mfem` re-solves the shock case with MFEM, an independent finite element code, built from the same
mesh and loads but with its own elements, assembly and conjugate-gradient solver, and prints the
largest displacement difference. The two codes use the same discretisation, so they agree to solver
tolerance (about 1e-8 on the test blocks); a larger difference means a bug in one of them. It needs a
build with libmfem (`libmfem-dev`), and `--crosscheck` does the same against CalculiX.

Meshes are limited to 250,000 degrees of freedom: choose a larger `--size` when you hit it.

## Materials

```
ohs materials                       # the catalogue with its sources
ohs materials --materials mine.toml # add your own
```

The catalogue holds typical handbook values (leaded brass, hardened steel, Ni-Fe hairspring alloy,
Co-Ni mainspring alloy, synthetic ruby, silicon, stainless, titanium). Each entry carries a source
note and a temperature range. They are starting points: confirm them against your supplier's data
sheet and override them with `--materials`.

## Verifying the solvers

```
ohs verify physics            # a few seconds
ohs verify physics --full     # finer meshes, benchmarks and convergence rates
```

The suite checks a cantilever's tip deflection (Timoshenko), its first natural frequency,
restrained thermal stress, Lame's thick cylinder, the stress concentration at a hole, the
large-deflection elastica, the tooth root against Lewis, and the convergence rates of linear and
quadratic tetrahedra on a manufactured solution. Exit code 1 means a solver no longer reproduces a
known answer on this machine.

The unit tests add Hertz line contact, the shrink-fit pressure of a shaft in a hub, the transient
step response (twice the static deflection), patch tests and the tangent stiffness against finite
differences.

## CalculiX cross-check

`--crosscheck` writes the model as a CalculiX input deck, runs `ccx` when it is on your `PATH`
(or at `$OHS_CCX`) and compares displacements. CalculiX is GPL and only ever runs as a separate
program. The deck writer and result parser are tested, and a run against CalculiX 2.21 on a clamped block agrees
within 1e-3 of the peak displacement; running CalculiX needs your install (`sudo apt install calculix-ccx`).
