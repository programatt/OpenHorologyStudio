// SPDX-License-Identifier: Apache-2.0
#pragma once

// Isoparametric 4- and 10-node tetrahedra. Internal to ohs_fea.

#include <array>
#include <vector>

#include <Eigen/Dense>

#include "mesh/tetmesh.hpp"

namespace ohs::fea {

using Bary = std::array<double, 4>;

struct Quadrature {
    std::vector<Bary> points;
    std::vector<double> weights;  ///< sum to 1/6 (the volume of the reference tetrahedron)
};

/// Exact for polynomials up to `degree` (1, 2 or 5 are available; others round up).
const Quadrature& tet_rule(int degree);

/// Shape functions and their physical gradients at a point of one element.
struct Shape {
    Eigen::VectorXd N;      ///< npe
    Eigen::MatrixXd dNdx;   ///< npe x 3
    double detJ{};
};

Shape evaluate_shape(const mesh::TetMesh& mesh, int element, const Bary& L);

/// Natural coordinates of local node `i` (0..npe-1).
Bary node_bary(int i);

/// Isotropic elasticity in Voigt order xx yy zz xy yz zx (engineering shear), N/mm^2.
Eigen::Matrix<double, 6, 6> elasticity(double youngs_mpa, double poisson);

/// Small-strain B (6 x 3 npe) from shape gradients.
Eigen::MatrixXd strain_matrix(const Eigen::MatrixXd& dNdx);

/// Surface shape functions on a boundary face (3 or 6 nodes) at area coordinates (l0, l1, l2).
Eigen::VectorXd face_shape(int nodes, double l0, double l1, double l2);

}  // namespace ohs::fea
