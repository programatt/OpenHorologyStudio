// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <vector>

#include "fea/fea.hpp"

namespace ohs::fea {

// ---- modal ----------------------------------------------------------------------------------

struct ModalResult {
    std::vector<double> frequencies_hz;  ///< ascending; rigid-body modes are dropped
    std::vector<Vec> modes;              ///< full-size shapes, mass-normalised (phi^T M phi = 1)
    int iterations{};
    int rigid_modes_dropped{};
};

/// The lowest `count` natural frequencies of K x = lambda M x on the unconstrained dofs, by
/// subspace (block inverse) iteration with Rayleigh-Ritz on a sparse factorisation of K + s M.
/// Free-free bodies are fine: the zero-frequency rigid-body modes are found and dropped.
ModalResult solve_modal(const Model& model, int count);

// ---- transient --------------------------------------------------------------------------------

struct TransientOptions {
    double dt{1e-5};
    double duration{1e-3};
    double beta{0.25};   ///< Newmark average acceleration by default: unconditionally stable
    double gamma{0.5};
    double alpha_damping{0};  ///< Rayleigh damping C = alpha M + beta_damping K
    double beta_damping{0};
};

struct TransientResult {
    std::vector<double> time;
    std::vector<std::vector<double>> probes;  ///< probes[k][step]: displacement history of probe dof k
    double peak_von_mises{};                  ///< over all recorded steps (checked every `stress_every` steps)
};

/// Newmark time integration from rest. `load(t)` returns the external force vector; `base_accel(t)`
/// (optional) a uniform support acceleration in mm/s^2 applied as an inertial load -M a.
/// Constrained dofs must be fixed at zero.
TransientResult solve_transient(const Model& model, const TransientOptions& options,
                                const std::function<Vec(double)>& load,
                                const std::function<mesh::V3(double)>& base_accel,
                                const std::vector<int>& probe_dofs, int stress_every = 0);

// ---- geometric nonlinearity -----------------------------------------------------------------------

struct NonlinearOptions {
    int steps{10};
    int max_iterations{30};
    double tolerance{1e-8};  ///< relative residual
};

struct NonlinearResult {
    Vec u;
    bool converged{false};
    int total_iterations{};
    std::vector<int> iterations_per_step;
    double final_residual{};
};

/// Total-Lagrangian Newton solution with a St. Venant-Kirchhoff material (large rotations and
/// displacements, small strains). The load and the prescribed displacements are scaled together
/// from zero to full in `steps` increments. Loads are dead loads (they do not follow the body).
NonlinearResult solve_nonlinear(const Model& model, const Vec& load, const NonlinearOptions& options = {});

/// Internal force and tangent stiffness at displacement `u` (exposed for tests).
Vec nonlinear_internal_force(const Model& model, const Vec& u);
Sparse nonlinear_tangent(const Model& model, const Vec& u);

// ---- contact with a rigid plane ----------------------------------------------------------------------

struct RigidPlane {
    mesh::V3 point;
    mesh::V3 normal;  ///< unit, pointing from the rigid body into the space where the deformable body lives
};

struct ContactResult {
    StaticResult solution;
    std::vector<int> active_nodes;
    std::vector<double> pressure;    ///< per active node, N/mm^2 (nodal force over lumped area)
    double total_force{};            ///< N, sum of the contact forces along the plane normal
    int iterations{};
};

/// Frictionless penalty contact of the given boundary faces against the plane, by active-set
/// iteration. `stiffness` is the penalty per unit area in N/mm^3; 0 picks 100 E / mean edge length.
ContactResult solve_contact(const Model& model, const Vec& load, const RigidPlane& plane,
                            const std::vector<mesh::BoundaryFace>& faces, double stiffness = 0);

// ---- assembling bodies -------------------------------------------------------------------------------------

/// Appends `src` to `dst` (node and element indices shifted); returns the node offset.
int append_mesh(mesh::TetMesh& dst, const mesh::TetMesh& src);

/// Ties each node of `a_nodes` to the node of `b_nodes` at the same position (within `tol`), with
/// direction `dir(position)` and gap `gap`. Throws when some node of `a_nodes` has no partner.
std::vector<Tie> tie_coincident(const mesh::TetMesh& mesh, const std::vector<int>& a_nodes, const std::vector<int>& b_nodes,
                                const std::function<mesh::V3(const mesh::V3&)>& dir, double gap, double tol = 1e-6);

// ---- thermal -------------------------------------------------------------------------------------------------------

/// Daily rate change per kelvin of a balance and hairspring pair, in s/day/K. The frequency is
/// sqrt(k/I) with k ~ E L^3 (the spring's section and length all grow with temperature) and
/// I ~ r^2, so d(rate)/dT = 86400 * (e/2 + 3 a_spring/2 - a_balance).
double temperature_rate_coefficient(const Material& hairspring, const Material& balance);

}  // namespace ohs::fea
