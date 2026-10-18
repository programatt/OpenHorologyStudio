# ADR 1: M0 scope and tooling

- C++20 with exceptions (`ohs::Error`) for user-facing failures; no `std::expected`, keeping the
  code portable across the compilers listed in the plan.
- Dependencies come from the system package manager for now (Ubuntu 24.04: OCCT 7.6.3). The
  vcpkg superbuild from the plan is deferred until Qt, MFEM and Chrono arrive.
- The dial axis is Z; `--up` rotates other CAD conventions into it.
- Gear module is inferred from the tip radius assuming a one-module addendum. Cycloidal
  horological profiles have a larger addendum, so the 5 % module tolerance may need loosening
  or the fit replaced by a flank fit (plan step 4) once real movements are available.
- Tooth counting uses a Fourier fundamental of the mid-height outline. A wheel and pinion fused
  into one solid are measured at mid-height only; split them in the CAD export or add a preset.
