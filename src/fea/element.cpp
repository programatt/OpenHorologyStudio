// SPDX-License-Identifier: Apache-2.0
#include "fea/element.hpp"

#include <cmath>

namespace ohs::fea {

namespace {

void add_permutations_3_1(Quadrature& q, double a, double b, double w) {
    // three coordinates a and one coordinate b
    for (int k = 0; k < 4; ++k) {
        Bary p{a, a, a, a};
        p[k] = b;
        q.points.push_back(p);
        q.weights.push_back(w);
    }
}

Quadrature make_rule(int degree) {
    Quadrature q;
    if (degree <= 1) {
        q.points.push_back({0.25, 0.25, 0.25, 0.25});
        q.weights.push_back(1.0 / 6.0);
    } else if (degree == 2) {
        add_permutations_3_1(q, 0.1381966011250105, 0.5854101966249685, 1.0 / 24.0);
    } else {
        // 14-point rule of degree 5 with positive weights (Walkington / Keast).
        const double a1 = 0.0927352503108912264, b1 = 1.0 - 3.0 * a1, w1 = 0.0122488405193936582;
        const double a2 = 0.3108859192633006098, b2 = 1.0 - 3.0 * a2, w2 = 0.0187813209530026417;
        const double a3 = 0.0455037041256496039, b3 = 0.5 - a3, w3 = 0.0070910034628469110;
        add_permutations_3_1(q, a1, b1, w1);
        add_permutations_3_1(q, a2, b2, w2);
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                Bary p{b3, b3, b3, b3};
                p[i] = a3;
                p[j] = a3;
                q.points.push_back(p);
                q.weights.push_back(w3);
            }
        }
    }
    return q;
}

}  // namespace

const Quadrature& tet_rule(int degree) {
    static const Quadrature r1 = make_rule(1), r2 = make_rule(2), r5 = make_rule(5);
    if (degree <= 1) return r1;
    if (degree == 2) return r2;
    return r5;
}

Bary node_bary(int i) {
    static const int pairs[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    Bary b{0, 0, 0, 0};
    if (i < 4) {
        b[i] = 1.0;
    } else {
        b[pairs[i - 4][0]] = 0.5;
        b[pairs[i - 4][1]] = 0.5;
    }
    return b;
}

Shape evaluate_shape(const mesh::TetMesh& m, int element, const Bary& L) {
    const int npe = m.nodes_per_element();
    const auto& conn = m.elems[element];
    Shape s;
    s.N.resize(npe);
    Eigen::MatrixXd dL(npe, 4);  // derivatives with respect to the four barycentric coordinates
    dL.setZero();
    if (npe == 4) {
        for (int i = 0; i < 4; ++i) {
            s.N[i] = L[i];
            dL(i, i) = 1.0;
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            s.N[i] = L[i] * (2 * L[i] - 1);
            dL(i, i) = 4 * L[i] - 1;
        }
        static const int pairs[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
        for (int k = 0; k < 6; ++k) {
            const int a = pairs[k][0], b = pairs[k][1];
            s.N[4 + k] = 4 * L[a] * L[b];
            dL(4 + k, a) = 4 * L[b];
            dL(4 + k, b) = 4 * L[a];
        }
    }
    Eigen::MatrixXd dxi(npe, 3);
    for (int j = 0; j < 3; ++j) dxi.col(j) = dL.col(j + 1) - dL.col(0);
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    for (int i = 0; i < npe; ++i) {
        const mesh::V3& p = m.nodes[conn[i]];
        for (int j = 0; j < 3; ++j) {
            J(0, j) += p.x * dxi(i, j);
            J(1, j) += p.y * dxi(i, j);
            J(2, j) += p.z * dxi(i, j);
        }
    }
    s.detJ = J.determinant();
    const Eigen::Matrix3d Jinv = J.inverse();
    s.dNdx = dxi * Jinv;  // (npe x 3) * (3 x 3): row i = dN_i/dxi^T * J^-1 = (J^-T dN_i/dxi)^T
    return s;
}

Eigen::Matrix<double, 6, 6> elasticity(double E, double nu) {
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    const double c = E / ((1 + nu) * (1 - 2 * nu));
    const double a = c * (1 - nu), b = c * nu, g = E / (2 * (1 + nu));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) D(i, j) = (i == j) ? a : b;
        D(3 + i, 3 + i) = g;
    }
    return D;
}

Eigen::MatrixXd strain_matrix(const Eigen::MatrixXd& g) {
    const int npe = static_cast<int>(g.rows());
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3 * npe);
    for (int i = 0; i < npe; ++i) {
        const double dx = g(i, 0), dy = g(i, 1), dz = g(i, 2);
        B(0, 3 * i) = dx;
        B(1, 3 * i + 1) = dy;
        B(2, 3 * i + 2) = dz;
        B(3, 3 * i) = dy;
        B(3, 3 * i + 1) = dx;
        B(4, 3 * i + 1) = dz;
        B(4, 3 * i + 2) = dy;
        B(5, 3 * i) = dz;
        B(5, 3 * i + 2) = dx;
    }
    return B;
}

Eigen::VectorXd face_shape(int nodes, double l0, double l1, double l2) {
    Eigen::VectorXd N(nodes);
    if (nodes == 3) {
        N << l0, l1, l2;
    } else {
        N << l0 * (2 * l0 - 1), l1 * (2 * l1 - 1), l2 * (2 * l2 - 1), 4 * l0 * l1, 4 * l1 * l2, 4 * l2 * l0;
    }
    return N;
}

}  // namespace ohs::fea
