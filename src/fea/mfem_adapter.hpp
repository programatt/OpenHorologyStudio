// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "fea/fea.hpp"

namespace ohs::fea {

// MFEM (BSD-3) is linked in-process but only as an independent check: the same linear static problem is
// built again from OHS's mesh with MFEM's own mesh, finite element space, elasticity integrator and
// conjugate gradient solver. The two codes share nothing but the mesh and the load vector, so agreement
// tests OHS's element formulation and assembly, not MFEM's.

/// True when this build links MFEM.
bool mfem_available();

/// Solves the linear static problem with MFEM (same restrictions as the cross-check below). Returns false, with the
/// reason in `note`, when MFEM is not built in, the model is not supported or the solver does not converge.
bool mfem_solve(const Model& model, const Vec& load, Vec& u, int& iterations, std::string& note);

struct MfemCheck {
    bool available{false};
    double max_displacement_diff{};  ///< largest nodal difference, relative to the largest displacement
    double ohs_max_displacement{};
    double mfem_max_displacement{};
    int cg_iterations{};
    std::string note;
};

/// Solves `model` under `load` with MFEM and compares the displacements with `ohs_result`. Handles
/// one-material models of linear or quadratic tetrahedra with fixed displacements; ties and several
/// materials are not supported (available = false, with the reason in `note`).
MfemCheck run_mfem_crosscheck(const Model& model, const Vec& load, const StaticResult& ohs_result);

}  // namespace ohs::fea
