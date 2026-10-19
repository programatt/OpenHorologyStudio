// SPDX-License-Identifier: Apache-2.0
#pragma once

// Finite elements for the parts of a movement. Units: mm, N, MPa (N/mm^2), tonne, s, K.
// With these a density of 7.85e-9 t/mm^3 and E in MPa give frequencies in Hz directly.

#include <array>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "fea/material.hpp"
#include "mesh/tetmesh.hpp"

namespace ohs::fea {

using Vec = Eigen::VectorXd;
using Sparse = Eigen::SparseMatrix<double>;
using Stress = std::array<double, 6>;  ///< xx yy zz xy yz zx

/// One prescribed displacement component of a node.
struct Fixed {
    int node{};
    int comp{};        ///< 0, 1, 2 for x, y, z
    double value{0};
};

/// Penalty constraint (u_a - u_b) . dir = gap between two coincident nodes of different bodies;
/// the interference of a press fit is a tie with a negative gap on the radial direction.
struct Tie {
    int a{};
    int b{};
    mesh::V3 dir;
    double gap{0};
};

struct Model {
    mesh::TetMesh mesh;
    Material material;
    /// Optional second and further materials: element e uses materials[element_material[e]];
    /// when element_material is empty every element uses `material`.
    std::vector<Material> materials;
    std::vector<int> element_material;
    std::vector<Fixed> fixed;
    std::vector<Tie> ties;
    double tie_stiffness{0};  ///< N/mm; 0 picks 1e4 * E * mean edge length

    std::size_t dofs() const { return mesh.dofs(); }
    const Material& material_of(std::size_t element) const;
    double effective_tie_stiffness() const;
};

/// Sparsity pattern of the stiffness (explicit zeros), including the tie couplings.
Sparse stiffness_pattern(const Model& model);
Sparse assemble_stiffness(const Model& model);
/// Consistent mass, or the row-sum lumped mass (positive and exact in total) when `lumped`.
Sparse assemble_mass(const Model& model, bool lumped = false);

// ---- load vectors (all add into `f`, sized model.dofs()) ----
void add_point_load(Vec& f, int node, const mesh::V3& force);
/// Force per unit area `traction` (N/mm^2, any direction) over boundary faces.
void add_traction(const Model& model, Vec& f, const std::vector<mesh::BoundaryFace>& faces, const mesh::V3& traction);
/// Pressure p (N/mm^2) pushing into the body on the given faces.
void add_pressure(const Model& model, Vec& f, const std::vector<mesh::BoundaryFace>& faces, double pressure);
/// Body force from an acceleration in mm/s^2 (9810 mm/s^2 is one g).
void add_gravity(const Model& model, Vec& f, const mesh::V3& acceleration);
/// Equivalent nodal forces of a uniform temperature rise dT (K) that the body may not expand into.
void add_thermal(const Model& model, Vec& f, double dT);
/// Body force per unit volume (N/mm^3) given as a function of position, integrated by quadrature.
void add_body_force(const Model& model, Vec& f, const std::function<mesh::V3(const mesh::V3&)>& force);
/// Root-mean-square of u_h - exact over the volume (L2 norm / sqrt(volume)), by quadrature.
double l2_error(const Model& model, const Vec& u, const std::function<mesh::V3(const mesh::V3&)>& exact);

double von_mises(const Stress& s);
/// Principal stresses, largest first.
std::array<double, 3> principal(const Stress& s);

struct StaticResult {
    Vec u;                         ///< displacements (3 per node)
    Vec reaction;                  ///< K u - f: the force the constraints supply at every dof
    double strain_energy{};        ///< N·mm
    std::vector<Stress> stress;    ///< averaged at the nodes
    std::vector<double> von_mises; ///< at the nodes
    double max_von_mises{};
    double max_principal{};        ///< largest tensile principal stress
    int max_principal_node{-1};
    int max_von_mises_node{-1};
};

enum class Solver {
    Direct,  ///< OHS's own sparse Cholesky
    Mfem,    ///< MFEM's conjugate gradient solver on the same mesh (one material, no ties; needs a build with MFEM)
};

struct SolveOptions {
    double thermal_dT{0};  ///< also subtract this thermal strain when recovering stresses
    Solver solver{Solver::Direct};
};

StaticResult solve_static(const Model& model, const Vec& load, const SolveOptions& options = {});

/// Stress at the nodes: element stresses evaluated at the element's own nodes and averaged.
std::vector<Stress> nodal_stress(const Model& model, const Vec& u, double thermal_dT = 0);

/// Solves K x = b on the dofs not listed in `fixed`, where those take `prescribed`; `b` may be
/// modified by the prescribed values. Used by all analyses.
Vec solve_reduced(const Sparse& K, const Vec& b, const std::vector<char>& is_fixed, const Vec& prescribed);

/// Dofs that are constrained, as a mask, and the values they take.
void constraint_vectors(const Model& model, std::vector<char>& is_fixed, Vec& prescribed);

}  // namespace ohs::fea
