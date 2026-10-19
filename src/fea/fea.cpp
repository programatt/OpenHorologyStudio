// SPDX-License-Identifier: Apache-2.0
#include "fea/fea.hpp"
#include "fea/mfem_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "core/error.hpp"
#include "fea/element.hpp"

namespace ohs::fea {

namespace {

using mesh::V3;

Sparse make_pattern(const Model& m) {
    const std::size_t nn = m.mesh.nodes.size();
    std::vector<std::vector<int>> adj(nn);
    const int npe = m.mesh.nodes_per_element();
    for (const auto& e : m.mesh.elems) {
        for (int i = 0; i < npe; ++i) {
            for (int j = 0; j < npe; ++j) adj[e[i]].push_back(e[j]);
        }
    }
    for (const Tie& t : m.ties) {
        for (int x : {t.a, t.b}) {
            for (int y : {t.a, t.b}) adj[x].push_back(y);
        }
    }
    Eigen::VectorXi counts(3 * nn);
    for (std::size_t i = 0; i < nn; ++i) {
        std::sort(adj[i].begin(), adj[i].end());
        adj[i].erase(std::unique(adj[i].begin(), adj[i].end()), adj[i].end());
        for (int c = 0; c < 3; ++c) counts[3 * i + c] = static_cast<int>(3 * adj[i].size());
    }
    Sparse A(3 * nn, 3 * nn);
    A.reserve(counts);
    for (std::size_t i = 0; i < nn; ++i) {
        for (int c = 0; c < 3; ++c) {
            for (int j : adj[i]) {
                for (int d = 0; d < 3; ++d) A.insert(3 * j + d, 3 * i + c) = 0.0;
            }
        }
    }
    A.makeCompressed();
    return A;
}

void scatter(Sparse& A, const std::array<int, 10>& conn, int npe, const Eigen::MatrixXd& Ke) {
    for (int a = 0; a < npe; ++a) {
        for (int b = 0; b < npe; ++b) {
            for (int c = 0; c < 3; ++c) {
                for (int d = 0; d < 3; ++d) A.coeffRef(3 * conn[a] + c, 3 * conn[b] + d) += Ke(3 * a + c, 3 * b + d);
            }
        }
    }
}

Eigen::Matrix<double, 6, 6> elasticity_of(const Model& model, std::size_t element) {
    const Material& mat = model.material_of(element);
    return elasticity(mat.youngs_mpa(), mat.poisson);
}

double mean_edge(const mesh::TetMesh& m) {
    if (m.elems.empty()) return 1.0;
    double s = 0;
    int n = 0;
    for (const auto& e : m.elems) {
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                const V3& p = m.nodes[e[i]];
                const V3& q = m.nodes[e[j]];
                s += std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z));
                ++n;
            }
        }
    }
    return s / n;
}

}  // namespace

const Material& Model::material_of(std::size_t element) const {
    if (element_material.empty()) return material;
    return materials.at(static_cast<std::size_t>(element_material[element]));
}

double Model::effective_tie_stiffness() const {
    double e = material.youngs_mpa();
    for (const Material& m : materials) e = std::max(e, m.youngs_mpa());
    return tie_stiffness > 0 ? tie_stiffness : 1e4 * e * mean_edge(mesh);
}

Sparse stiffness_pattern(const Model& model) { return make_pattern(model); }

Sparse assemble_stiffness(const Model& model) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 1 : 2);
    Sparse K = make_pattern(model);
    Eigen::MatrixXd Ke(3 * npe, 3 * npe);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        Ke.setZero();
        const Eigen::Matrix<double, 6, 6> D = elasticity_of(model, ei);
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            const Eigen::MatrixXd B = strain_matrix(s.dNdx);
            Ke += B.transpose() * D * B * (rule.weights[g] * std::abs(s.detJ));
        }
        scatter(K, m.elems[ei], npe, Ke);
    }
    const double kp = model.effective_tie_stiffness();
    for (const Tie& t : model.ties) {
        const double d[3] = {t.dir.x, t.dir.y, t.dir.z};
        for (int p = 0; p < 2; ++p) {
            for (int q = 0; q < 2; ++q) {
                const int na = p == 0 ? t.a : t.b, nb = q == 0 ? t.a : t.b;
                const double sign = (p == q) ? 1.0 : -1.0;
                for (int c = 0; c < 3; ++c) {
                    for (int e = 0; e < 3; ++e) K.coeffRef(3 * na + c, 3 * nb + e) += kp * sign * d[c] * d[e];
                }
            }
        }
    }
    return K;
}

Sparse assemble_mass(const Model& model, bool lumped) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 2 : 5);
    Sparse M = make_pattern(model);
    Eigen::MatrixXd Me(3 * npe, 3 * npe);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        Me.setZero();
        const double rho = model.material_of(ei).density_t_mm3();
        double vol = 0;
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            const double w = rule.weights[g] * std::abs(s.detJ);
            vol += w;
            for (int a = 0; a < npe; ++a) {
                for (int b = 0; b < npe; ++b) {
                    for (int c = 0; c < 3; ++c) Me(3 * a + c, 3 * b + c) += rho * s.N[a] * s.N[b] * w;
                }
            }
        }
        if (lumped) {
            // HRZ lumping: keep the diagonal of the consistent matrix, scaled to conserve the mass.
            double diag = 0;
            for (int a = 0; a < npe; ++a) diag += Me(3 * a, 3 * a);
            const double scale = rho * vol / diag;
            Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3 * npe, 3 * npe);
            for (int i = 0; i < 3 * npe; ++i) L(i, i) = Me(i, i) * scale;
            Me = L;
        }
        scatter(M, m.elems[ei], npe, Me);
    }
    return M;
}

void add_point_load(Vec& f, int node, const V3& force) {
    f[3 * node] += force.x;
    f[3 * node + 1] += force.y;
    f[3 * node + 2] += force.z;
}

void add_traction(const Model& model, Vec& f, const std::vector<mesh::BoundaryFace>& faces, const V3& t) {
    static const double gp[3][3] = {{2.0 / 3, 1.0 / 6, 1.0 / 6}, {1.0 / 6, 2.0 / 3, 1.0 / 6}, {1.0 / 6, 1.0 / 6, 2.0 / 3}};
    const int fn = model.mesh.order == 1 ? 3 : 6;
    for (const mesh::BoundaryFace& face : faces) {
        const double area = mesh::face_area(model.mesh, face);
        for (const auto& g : gp) {
            const Eigen::VectorXd N = face_shape(fn, g[0], g[1], g[2]);
            for (int i = 0; i < fn; ++i) {
                const double w = area / 3.0 * N[i];
                f[3 * face.nodes[i]] += w * t.x;
                f[3 * face.nodes[i] + 1] += w * t.y;
                f[3 * face.nodes[i] + 2] += w * t.z;
            }
        }
    }
}

void add_pressure(const Model& model, Vec& f, const std::vector<mesh::BoundaryFace>& faces, double pressure) {
    const int fn = model.mesh.order == 1 ? 3 : 6;
    static const double gp[3][3] = {{2.0 / 3, 1.0 / 6, 1.0 / 6}, {1.0 / 6, 2.0 / 3, 1.0 / 6}, {1.0 / 6, 1.0 / 6, 2.0 / 3}};
    for (const mesh::BoundaryFace& face : faces) {
        const double area = mesh::face_area(model.mesh, face);
        const V3 n = mesh::face_normal(model.mesh, face);
        for (const auto& g : gp) {
            const Eigen::VectorXd N = face_shape(fn, g[0], g[1], g[2]);
            for (int i = 0; i < fn; ++i) {
                const double w = -pressure * area / 3.0 * N[i];
                f[3 * face.nodes[i]] += w * n.x;
                f[3 * face.nodes[i] + 1] += w * n.y;
                f[3 * face.nodes[i] + 2] += w * n.z;
            }
        }
    }
}

void add_gravity(const Model& model, Vec& f, const V3& a) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 1 : 2);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        const double rho = model.material_of(ei).density_t_mm3();
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            const double w = rule.weights[g] * std::abs(s.detJ) * rho;
            for (int i = 0; i < npe; ++i) {
                const int n = m.elems[ei][i];
                f[3 * n] += w * s.N[i] * a.x;
                f[3 * n + 1] += w * s.N[i] * a.y;
                f[3 * n + 2] += w * s.N[i] * a.z;
            }
        }
    }
}

void add_thermal(const Model& model, Vec& f, double dT) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 1 : 2);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        const auto D = elasticity_of(model, ei);
        Eigen::Matrix<double, 6, 1> eth;
        eth << 1, 1, 1, 0, 0, 0;
        eth *= model.material_of(ei).cte * dT;
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            const Eigen::VectorXd fe = strain_matrix(s.dNdx).transpose() * D * eth * (rule.weights[g] * std::abs(s.detJ));
            for (int i = 0; i < npe; ++i) {
                for (int c = 0; c < 3; ++c) f[3 * m.elems[ei][i] + c] += fe[3 * i + c];
            }
        }
    }
}

void add_body_force(const Model& model, Vec& f, const std::function<V3(const V3&)>& force) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(5);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            V3 x{};
            for (int i = 0; i < npe; ++i) {
                const V3& p = m.nodes[m.elems[ei][i]];
                x.x += s.N[i] * p.x;
                x.y += s.N[i] * p.y;
                x.z += s.N[i] * p.z;
            }
            const V3 b = force(x);
            const double w = rule.weights[g] * std::abs(s.detJ);
            for (int i = 0; i < npe; ++i) {
                const int n = m.elems[ei][i];
                f[3 * n] += w * s.N[i] * b.x;
                f[3 * n + 1] += w * s.N[i] * b.y;
                f[3 * n + 2] += w * s.N[i] * b.z;
            }
        }
    }
}

double l2_error(const Model& model, const Vec& u, const std::function<V3(const V3&)>& exact) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(5);
    double sum = 0, vol = 0;
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), rule.points[g]);
            V3 x{};
            Eigen::Vector3d uh = Eigen::Vector3d::Zero();
            for (int i = 0; i < npe; ++i) {
                const int n = m.elems[ei][i];
                x.x += s.N[i] * m.nodes[n].x;
                x.y += s.N[i] * m.nodes[n].y;
                x.z += s.N[i] * m.nodes[n].z;
                uh += s.N[i] * u.segment<3>(3 * n);
            }
            const V3 ue = exact(x);
            const double w = rule.weights[g] * std::abs(s.detJ);
            sum += w * ((uh[0] - ue.x) * (uh[0] - ue.x) + (uh[1] - ue.y) * (uh[1] - ue.y) + (uh[2] - ue.z) * (uh[2] - ue.z));
            vol += w;
        }
    }
    return std::sqrt(sum / vol);
}

double von_mises(const Stress& s) {
    return std::sqrt(0.5 * ((s[0] - s[1]) * (s[0] - s[1]) + (s[1] - s[2]) * (s[1] - s[2]) + (s[2] - s[0]) * (s[2] - s[0])) +
                     3 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
}

std::array<double, 3> principal(const Stress& s) {
    Eigen::Matrix3d T;
    T << s[0], s[3], s[5], s[3], s[1], s[4], s[5], s[4], s[2];
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(T, Eigen::EigenvaluesOnly);
    const auto& v = es.eigenvalues();  // ascending
    return {v[2], v[1], v[0]};
}

void constraint_vectors(const Model& model, std::vector<char>& is_fixed, Vec& prescribed) {
    is_fixed.assign(model.dofs(), 0);
    prescribed = Vec::Zero(static_cast<Eigen::Index>(model.dofs()));
    for (const Fixed& c : model.fixed) {
        if (c.node < 0 || static_cast<std::size_t>(c.node) >= model.mesh.nodes.size() || c.comp < 0 || c.comp > 2) {
            throw Error("a displacement constraint refers to a node or component that does not exist");
        }
        is_fixed[3 * c.node + c.comp] = 1;
        prescribed[3 * c.node + c.comp] = c.value;
    }
}

Vec solve_reduced(const Sparse& K, const Vec& b, const std::vector<char>& is_fixed, const Vec& prescribed) {
    const Eigen::Index n = K.rows();
    std::vector<int> map(static_cast<std::size_t>(n), -1);
    int nf = 0;
    for (Eigen::Index i = 0; i < n; ++i) {
        if (!is_fixed[i]) map[i] = nf++;
    }
    Vec x = prescribed;
    if (nf == 0) return x;
    Vec rhs(nf);
    for (Eigen::Index i = 0; i < n; ++i) {
        if (map[i] >= 0) rhs[map[i]] = b[i];
    }
    std::vector<Eigen::Triplet<double>> trip;
    trip.reserve(static_cast<std::size_t>(K.nonZeros()));
    for (int c = 0; c < K.outerSize(); ++c) {
        for (Sparse::InnerIterator it(K, c); it; ++it) {
            const int r = static_cast<int>(it.row());
            if (map[r] < 0) continue;
            if (map[c] >= 0) trip.emplace_back(map[r], map[c], it.value());
            else rhs[map[r]] -= it.value() * prescribed[c];
        }
    }
    Sparse Kff(nf, nf);
    Kff.setFromTriplets(trip.begin(), trip.end());
    Eigen::SimplicialLDLT<Sparse> ldlt(Kff);
    if (ldlt.info() != Eigen::Success) {
        throw Error("the stiffness matrix could not be factorised: the body is probably not fully constrained");
    }
    const Vec sol = ldlt.solve(rhs);
    if (!sol.allFinite()) throw Error("the solution is not finite: the body is probably not fully constrained");
    for (Eigen::Index i = 0; i < n; ++i) {
        if (map[i] >= 0) x[i] = sol[map[i]];
    }
    return x;
}

std::vector<Stress> nodal_stress(const Model& model, const Vec& u, double dT) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    std::vector<Stress> sum(m.nodes.size(), Stress{});
    std::vector<int> count(m.nodes.size(), 0);
    Eigen::VectorXd ue(3 * npe);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        const auto& conn = m.elems[ei];
        const auto D = elasticity_of(model, ei);
        Eigen::Matrix<double, 6, 1> eth;
        eth << 1, 1, 1, 0, 0, 0;
        eth *= model.material_of(ei).cte * dT;
        for (int i = 0; i < npe; ++i) ue.segment<3>(3 * i) = u.segment<3>(3 * conn[i]);
        for (int i = 0; i < npe; ++i) {
            const Shape s = evaluate_shape(m, static_cast<int>(ei), node_bary(i));
            const Eigen::Matrix<double, 6, 1> sig = D * (strain_matrix(s.dNdx) * ue - eth);
            for (int k = 0; k < 6; ++k) sum[conn[i]][k] += sig[k];
            ++count[conn[i]];
        }
    }
    for (std::size_t i = 0; i < sum.size(); ++i) {
        if (count[i] > 0) {
            for (double& v : sum[i]) v /= count[i];
        }
    }
    return sum;
}

StaticResult solve_static(const Model& model, const Vec& load, const SolveOptions& opt) {
    if (static_cast<std::size_t>(load.size()) != model.dofs()) throw Error("the load vector does not match the mesh");
    std::vector<char> is_fixed;
    Vec prescribed;
    constraint_vectors(model, is_fixed, prescribed);

    Sparse K = assemble_stiffness(model);
    Vec f = load;
    const double kp = model.effective_tie_stiffness();
    for (const Tie& t : model.ties) {
        const double d[3] = {t.dir.x, t.dir.y, t.dir.z};
        for (int c = 0; c < 3; ++c) {
            f[3 * t.a + c] += kp * t.gap * d[c];
            f[3 * t.b + c] -= kp * t.gap * d[c];
        }
    }

    StaticResult r;
    if (opt.solver == Solver::Mfem) {
        std::string note;
        int iterations = 0;
        if (!mfem_solve(model, load, r.u, iterations, note)) throw Error(note);
    } else {
        r.u = solve_reduced(K, f, is_fixed, prescribed);
    }
    r.reaction = K * r.u - f;
    for (std::size_t i = 0; i < is_fixed.size(); ++i) {
        if (!is_fixed[i]) r.reaction[i] = 0.0;
    }
    r.strain_energy = 0.5 * r.u.dot(K * r.u);
    r.stress = nodal_stress(model, r.u, opt.thermal_dT);
    r.von_mises.resize(r.stress.size());
    r.max_principal = -1e300;
    for (std::size_t i = 0; i < r.stress.size(); ++i) {
        r.von_mises[i] = fea::von_mises(r.stress[i]);
        if (r.von_mises[i] > r.max_von_mises) {
            r.max_von_mises = r.von_mises[i];
            r.max_von_mises_node = static_cast<int>(i);
        }
        const double p1 = principal(r.stress[i])[0];
        if (p1 > r.max_principal) {
            r.max_principal = p1;
            r.max_principal_node = static_cast<int>(i);
        }
    }
    return r;
}

}  // namespace ohs::fea
