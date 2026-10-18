// SPDX-License-Identifier: Apache-2.0
#include "fea/analysis.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <tuple>

#include "core/error.hpp"
#include "fea/element.hpp"

namespace ohs::fea {

namespace {

using mesh::V3;

/// The free dofs (not constrained) and conversions between full and reduced vectors.
struct Reduced {
    std::vector<int> map;   // full -> reduced, -1 when constrained
    std::vector<int> free;  // reduced -> full

    explicit Reduced(const std::vector<char>& is_fixed) : map(is_fixed.size(), -1) {
        for (std::size_t i = 0; i < is_fixed.size(); ++i) {
            if (!is_fixed[i]) {
                map[i] = static_cast<int>(free.size());
                free.push_back(static_cast<int>(i));
            }
        }
    }
    int n() const { return static_cast<int>(free.size()); }

    Sparse reduce(const Sparse& A) const {
        std::vector<Eigen::Triplet<double>> t;
        t.reserve(static_cast<std::size_t>(A.nonZeros()));
        for (int c = 0; c < A.outerSize(); ++c) {
            if (map[c] < 0) continue;
            for (Sparse::InnerIterator it(A, c); it; ++it) {
                if (map[it.row()] >= 0) t.emplace_back(map[it.row()], map[c], it.value());
            }
        }
        Sparse B(n(), n());
        B.setFromTriplets(t.begin(), t.end());
        return B;
    }
    Vec reduce(const Vec& v) const {
        Vec r(n());
        for (int i = 0; i < n(); ++i) r[i] = v[free[i]];
        return r;
    }
    Vec expand(const Vec& r, Eigen::Index full) const {
        Vec v = Vec::Zero(full);
        for (int i = 0; i < n(); ++i) v[free[i]] = r[i];
        return v;
    }
};

Reduced reduced_for(const Model& model, Vec* prescribed = nullptr) {
    std::vector<char> is_fixed;
    Vec pres;
    constraint_vectors(model, is_fixed, pres);
    if (prescribed) *prescribed = pres;
    return Reduced(is_fixed);
}

double bbox_diagonal(const mesh::TetMesh& m) {
    if (m.nodes.empty()) return 1.0;
    V3 lo = m.nodes[0], hi = m.nodes[0];
    for (const V3& p : m.nodes) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    return std::sqrt((hi.x - lo.x) * (hi.x - lo.x) + (hi.y - lo.y) * (hi.y - lo.y) + (hi.z - lo.z) * (hi.z - lo.z));
}

}  // namespace

// ---- modal ---------------------------------------------------------------------------------------------

ModalResult solve_modal(const Model& model, int count) {
    if (count < 1) throw Error("ask for at least one mode");
    const Reduced R = reduced_for(model);
    const Sparse K = R.reduce(assemble_stiffness(model));
    const Sparse M = R.reduce(assemble_mass(model, false));
    const int nf = R.n();
    const int target = std::min(count + 6, nf);
    const int q = std::min(nf, target + std::max(target, 8));
    if (target < 1) throw Error("nothing is free to vibrate");

    const double L = bbox_diagonal(model.mesh);
    const double lambda_scale = model.material.youngs_mpa() / (model.material.density_t_mm3() * L * L);
    const double shift = 1e-6 * lambda_scale;
    Sparse A = K + shift * M;
    Eigen::SimplicialLDLT<Sparse> ldlt(A);
    if (ldlt.info() != Eigen::Success) throw Error("modal analysis: the stiffness matrix could not be factorised");

    // Deterministic starting block.
    Eigen::MatrixXd X(nf, q);
    unsigned long long state = 88172645463325252ULL;
    for (int j = 0; j < q; ++j) {
        for (int i = 0; i < nf; ++i) {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            X(i, j) = static_cast<double>(state % 2000003ULL) / 1000001.5 - 1.0;
        }
    }
    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(q);
    ModalResult out;
    for (int it = 0; it < 80; ++it) {
        Eigen::MatrixXd Y = ldlt.solve(Eigen::MatrixXd(M * X));
        const Eigen::MatrixXd Kr = Y.transpose() * (K * Y);
        const Eigen::MatrixXd Mr = Y.transpose() * (M * Y);
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es(Kr, Mr);
        if (es.info() != Eigen::Success) throw Error("modal analysis: the projected eigenproblem failed");
        const Eigen::VectorXd next = es.eigenvalues();
        X = Y * es.eigenvectors();
        out.iterations = it + 1;
        double change = 0;
        for (int k = 0; k < target; ++k) {
            change = std::max(change, std::abs(next[k] - lambda[k]) / std::max(std::abs(next[k]), 1e-9 * lambda_scale));
        }
        lambda = next;
        if (it > 0 && change < 1e-11) break;
    }

    const int full = static_cast<int>(model.dofs());
    for (int k = 0; k < target && static_cast<int>(out.frequencies_hz.size()) < count; ++k) {
        if (lambda[k] < 1e-8 * lambda_scale) {
            ++out.rigid_modes_dropped;
            continue;
        }
        out.frequencies_hz.push_back(std::sqrt(lambda[k]) / (2.0 * std::numbers::pi));
        out.modes.push_back(R.expand(X.col(k), full));
    }
    return out;
}

// ---- transient ------------------------------------------------------------------------------------------

TransientResult solve_transient(const Model& model, const TransientOptions& o, const std::function<Vec(double)>& load,
                                const std::function<V3(double)>& base_accel, const std::vector<int>& probes,
                                int stress_every) {
    if (o.dt <= 0 || o.duration <= 0) throw Error("transient: dt and duration must be positive");
    for (const Fixed& c : model.fixed) {
        if (c.value != 0.0) throw Error("transient: prescribed displacements must be zero");
    }
    const Reduced R = reduced_for(model);
    const Sparse Kfull = assemble_stiffness(model);
    const Sparse Mfull = assemble_mass(model, false);
    const Sparse K = R.reduce(Kfull);
    const Sparse M = R.reduce(Mfull);
    const Sparse C = o.alpha_damping * M + o.beta_damping * K;

    const double a0 = 1.0 / (o.beta * o.dt * o.dt), a1 = o.gamma / (o.beta * o.dt), a2 = 1.0 / (o.beta * o.dt);
    const double a3 = 1.0 / (2 * o.beta) - 1.0, a4 = o.gamma / o.beta - 1.0, a5 = o.dt * (o.gamma / (2 * o.beta) - 1.0);
    Sparse Keff = K + a0 * M + a1 * C;
    Eigen::SimplicialLDLT<Sparse> keff(Keff);
    Eigen::SimplicialLDLT<Sparse> mass(M);
    if (keff.info() != Eigen::Success || mass.info() != Eigen::Success) {
        throw Error("transient: the system could not be factorised: is the body constrained?");
    }

    // Mass times the unit field in each direction, for a base acceleration.
    Vec Mdir[3];
    for (int c = 0; c < 3; ++c) {
        Vec e = Vec::Zero(model.dofs());
        for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) e[3 * n + c] = 1.0;
        Mdir[c] = Mfull * e;
    }
    auto external = [&](double t) {
        Vec f = load ? load(t) : Vec::Zero(model.dofs());
        if (base_accel) {
            const V3 a = base_accel(t);
            f -= Mdir[0] * a.x + Mdir[1] * a.y + Mdir[2] * a.z;
        }
        return R.reduce(f);
    };

    const int nf = R.n();
    Vec u = Vec::Zero(nf), v = Vec::Zero(nf), a = mass.solve(external(0.0));
    TransientResult out;
    out.probes.assign(probes.size(), {});
    auto record = [&](double t) {
        out.time.push_back(t);
        for (std::size_t k = 0; k < probes.size(); ++k) {
            const int r = R.map[probes[k]];
            out.probes[k].push_back(r >= 0 ? u[r] : 0.0);
        }
    };
    record(0.0);
    const long steps = static_cast<long>(std::llround(o.duration / o.dt));
    for (long s = 1; s <= steps; ++s) {
        const double t = s * o.dt;
        const Vec rhs = external(t) + M * (a0 * u + a2 * v + a3 * a) + C * (a1 * u + a4 * v + a5 * a);
        const Vec un = keff.solve(rhs);
        const Vec an = a0 * (un - u) - a2 * v - a3 * a;
        v += o.dt * ((1 - o.gamma) * a + o.gamma * an);
        a = an;
        u = un;
        record(t);
        if (stress_every > 0 && s % stress_every == 0) {
            for (const Stress& st : nodal_stress(model, R.expand(u, static_cast<Eigen::Index>(model.dofs())))) {
                out.peak_von_mises = std::max(out.peak_von_mises, von_mises(st));
            }
        }
    }
    return out;
}

// ---- geometric nonlinearity ------------------------------------------------------------------------------------

namespace {

struct TlPoint {
    Eigen::Matrix3d F;
    Eigen::Matrix3d S;
    Eigen::MatrixXd B;  // 6 x 3npe
    Eigen::MatrixXd dN;
    double w;
};

void tl_point(const Model& model, std::size_t ei, const Vec& u, const Bary& L, double weight, TlPoint& p) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Shape s = evaluate_shape(m, static_cast<int>(ei), L);
    p.dN = s.dNdx;
    p.w = weight * std::abs(s.detJ);
    Eigen::MatrixXd Ue(3, npe);
    for (int a = 0; a < npe; ++a) Ue.col(a) = u.segment<3>(3 * m.elems[ei][a]);
    p.F = Eigen::Matrix3d::Identity() + Ue * s.dNdx;
    const Eigen::Matrix3d E = 0.5 * (p.F.transpose() * p.F - Eigen::Matrix3d::Identity());
    const Material& mat = model.material_of(ei);
    const double lambda = mat.youngs_mpa() * mat.poisson / ((1 + mat.poisson) * (1 - 2 * mat.poisson));
    const double mu = mat.shear_mpa();
    p.S = lambda * E.trace() * Eigen::Matrix3d::Identity() + 2 * mu * E;
    p.B = Eigen::MatrixXd::Zero(6, 3 * npe);
    for (int a = 0; a < npe; ++a) {
        for (int i = 0; i < 3; ++i) {
            const int col = 3 * a + i;
            p.B(0, col) = p.F(i, 0) * p.dN(a, 0);
            p.B(1, col) = p.F(i, 1) * p.dN(a, 1);
            p.B(2, col) = p.F(i, 2) * p.dN(a, 2);
            p.B(3, col) = p.F(i, 0) * p.dN(a, 1) + p.F(i, 1) * p.dN(a, 0);
            p.B(4, col) = p.F(i, 1) * p.dN(a, 2) + p.F(i, 2) * p.dN(a, 1);
            p.B(5, col) = p.F(i, 2) * p.dN(a, 0) + p.F(i, 0) * p.dN(a, 2);
        }
    }
}

}  // namespace

Vec nonlinear_internal_force(const Model& model, const Vec& u) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 1 : 5);
    Vec f = Vec::Zero(model.dofs());
    TlPoint p;
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            tl_point(model, ei, u, rule.points[g], rule.weights[g], p);
            Eigen::Matrix<double, 6, 1> sv;
            sv << p.S(0, 0), p.S(1, 1), p.S(2, 2), p.S(0, 1), p.S(1, 2), p.S(2, 0);
            const Eigen::VectorXd fe = p.B.transpose() * sv * p.w;
            for (int a = 0; a < npe; ++a) f.segment<3>(3 * m.elems[ei][a]) += fe.segment<3>(3 * a);
        }
    }
    return f;
}

Sparse nonlinear_tangent(const Model& model, const Vec& u) {
    const auto& m = model.mesh;
    const int npe = m.nodes_per_element();
    const Quadrature& rule = tet_rule(npe == 4 ? 1 : 5);
    Sparse K = stiffness_pattern(model);
    TlPoint p;
    Eigen::MatrixXd Ke(3 * npe, 3 * npe);
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        Ke.setZero();
        const Material& mat = model.material_of(ei);
        const auto D = elasticity(mat.youngs_mpa(), mat.poisson);
        for (std::size_t g = 0; g < rule.points.size(); ++g) {
            tl_point(model, ei, u, rule.points[g], rule.weights[g], p);
            Ke += p.B.transpose() * D * p.B * p.w;
            const Eigen::MatrixXd G = p.dN * p.S * p.dN.transpose() * p.w;  // npe x npe
            for (int a = 0; a < npe; ++a) {
                for (int b = 0; b < npe; ++b) {
                    for (int i = 0; i < 3; ++i) Ke(3 * a + i, 3 * b + i) += G(a, b);
                }
            }
        }
        for (int a = 0; a < npe; ++a) {
            for (int b = 0; b < npe; ++b) {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) K.coeffRef(3 * m.elems[ei][a] + i, 3 * m.elems[ei][b] + j) += Ke(3 * a + i, 3 * b + j);
                }
            }
        }
    }
    return K;
}

NonlinearResult solve_nonlinear(const Model& model, const Vec& load, const NonlinearOptions& o) {
    if (!model.ties.empty()) throw Error("nonlinear analysis does not support ties");
    std::vector<char> is_fixed;
    Vec prescribed;
    constraint_vectors(model, is_fixed, prescribed);
    const Reduced R(is_fixed);
    NonlinearResult out;
    out.u = Vec::Zero(model.dofs());
    const Vec zero = Vec::Zero(model.dofs());
    for (int step = 1; step <= o.steps; ++step) {
        const double lambda = static_cast<double>(step) / o.steps;
        for (std::size_t i = 0; i < is_fixed.size(); ++i) {
            if (is_fixed[i]) out.u[i] = lambda * prescribed[i];
        }
        int iter = 0;
        bool ok = false;
        for (; iter < o.max_iterations; ++iter) {
            const Vec fint = nonlinear_internal_force(model, out.u);
            const Vec residual = fint - lambda * load;
            const Vec rf = R.reduce(residual);
            const double ref = (lambda * R.reduce(load)).norm() + R.reduce(fint).norm() + 1e-300;
            out.final_residual = rf.norm() / ref;
            if (out.final_residual < o.tolerance) {
                ok = true;
                break;
            }
            const Sparse K = nonlinear_tangent(model, out.u);
            const Vec du = solve_reduced(K, -residual, is_fixed, zero);
            out.u += du;
        }
        out.iterations_per_step.push_back(iter);
        out.total_iterations += iter;
        if (!ok) {
            out.converged = false;
            return out;
        }
    }
    out.converged = true;
    return out;
}

// ---- contact -------------------------------------------------------------------------------------------------------

ContactResult solve_contact(const Model& model, const Vec& load, const RigidPlane& plane,
                            const std::vector<mesh::BoundaryFace>& faces, double stiffness) {
    const auto& m = model.mesh;
    // Lumped (HRZ) face weights: corners 6/114, mid-sides 32/114 of the area for 6-node faces.
    std::map<int, double> weight;
    for (const mesh::BoundaryFace& f : faces) {
        const double area = mesh::face_area(m, f);
        if (m.order == 1) {
            for (int k = 0; k < 3; ++k) weight[f.nodes[k]] += area / 3.0;
        } else {
            for (int k = 0; k < 3; ++k) weight[f.nodes[k]] += area * 6.0 / 114.0;
            for (int k = 3; k < 6; ++k) weight[f.nodes[k]] += area * 32.0 / 114.0;
        }
    }
    double edge = 0;
    int ne = 0;
    for (const auto& e : m.elems) {
        const V3& p = m.nodes[e[0]];
        const V3& q = m.nodes[e[1]];
        edge += std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z));
        ++ne;
    }
    edge /= std::max(ne, 1);
    const double k_area = stiffness > 0 ? stiffness : 100.0 * model.material.youngs_mpa() / edge;
    const double nx = plane.normal.x, ny = plane.normal.y, nz = plane.normal.z;
    const double n[3] = {nx, ny, nz};

    std::vector<char> is_fixed;
    Vec prescribed;
    constraint_vectors(model, is_fixed, prescribed);
    const Sparse K0 = assemble_stiffness(model);

    auto initial_gap = [&](int node) {
        const V3& p = m.nodes[node];
        return (p.x - plane.point.x) * nx + (p.y - plane.point.y) * ny + (p.z - plane.point.z) * nz;
    };

    ContactResult out;
    std::map<int, bool> active;
    // Start with the nodes closest to the plane in contact, so the first system is supported.
    double closest = 1e300;
    for (const auto& [node, w] : weight) closest = std::min(closest, initial_gap(node));
    for (const auto& [node, w] : weight) active[node] = initial_gap(node) <= closest + 1e-9;
    for (int it = 0; it < 80; ++it) {
        Sparse K = K0;
        Vec f = load;
        for (const auto& [node, on] : active) {
            if (!on) continue;
            const double kp = k_area * weight[node];
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) K.coeffRef(3 * node + a, 3 * node + b) += kp * n[a] * n[b];
                f[3 * node + a] -= kp * initial_gap(node) * n[a];
            }
        }
        const Vec u = solve_reduced(K, f, is_fixed, prescribed);
        bool changed = false;
        for (const auto& [node, w] : weight) {
            const double g = initial_gap(node) + u[3 * node] * nx + u[3 * node + 1] * ny + u[3 * node + 2] * nz;
            const bool now = g < 0;
            if (now != active[node]) {
                active[node] = now;
                changed = true;
            }
        }
        out.iterations = it + 1;
        if (!changed) {
            out.solution.u = u;
            // Contact forces as reactions.
            out.solution.reaction = Vec::Zero(model.dofs());
            for (const auto& [node, on] : active) {
                if (!on) continue;
                const double g = initial_gap(node) + u[3 * node] * nx + u[3 * node + 1] * ny + u[3 * node + 2] * nz;
                const double force = -k_area * weight[node] * g;
                out.active_nodes.push_back(node);
                out.pressure.push_back(force / weight[node]);
                out.total_force += force;
                for (int a = 0; a < 3; ++a) out.solution.reaction[3 * node + a] = force * n[a];
            }
            out.solution.strain_energy = 0.5 * u.dot(K0 * u);
            out.solution.stress = nodal_stress(model, u);
            out.solution.von_mises.resize(out.solution.stress.size());
            for (std::size_t i = 0; i < out.solution.stress.size(); ++i) {
                out.solution.von_mises[i] = von_mises(out.solution.stress[i]);
                out.solution.max_von_mises = std::max(out.solution.max_von_mises, out.solution.von_mises[i]);
            }
            return out;
        }
    }
    throw Error("contact: the active set did not settle in 80 iterations");
}

// ---- assembling bodies -----------------------------------------------------------------------------------------------

int append_mesh(mesh::TetMesh& dst, const mesh::TetMesh& src) {
    if (!dst.nodes.empty() && dst.order != src.order) throw Error("cannot join meshes of different element order");
    const int offset = static_cast<int>(dst.nodes.size());
    dst.order = src.order;
    dst.nodes.insert(dst.nodes.end(), src.nodes.begin(), src.nodes.end());
    for (auto e : src.elems) {
        for (int& v : e) {
            if (v >= 0) v += offset;
        }
        dst.elems.push_back(e);
    }
    return offset;
}

std::vector<Tie> tie_coincident(const mesh::TetMesh& m, const std::vector<int>& a_nodes, const std::vector<int>& b_nodes,
                                const std::function<V3(const V3&)>& dir, double gap, double tol) {
    const double cell = std::max(10 * tol, 1e-9);
    using Key = std::tuple<long long, long long, long long>;
    auto key_of = [&](const V3& p) {
        return Key{static_cast<long long>(std::floor(p.x / cell)), static_cast<long long>(std::floor(p.y / cell)),
                   static_cast<long long>(std::floor(p.z / cell))};
    };
    std::map<Key, std::vector<int>> grid;
    for (int b : b_nodes) grid[key_of(m.nodes[b])].push_back(b);
    std::vector<Tie> ties;
    for (int a : a_nodes) {
        const V3& p = m.nodes[a];
        const auto [kx, ky, kz] = key_of(p);
        int partner = -1;
        double best = tol;
        for (long long dx = -1; dx <= 1; ++dx) {
            for (long long dy = -1; dy <= 1; ++dy) {
                for (long long dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find(Key{kx + dx, ky + dy, kz + dz});
                    if (it == grid.end()) continue;
                    for (int b : it->second) {
                        const V3& q = m.nodes[b];
                        const double d = std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z));
                        if (d <= best) {
                            best = d;
                            partner = b;
                        }
                    }
                }
            }
        }
        if (partner < 0) throw Error("tie: a node of the first body has no partner on the second body");
        ties.push_back({a, partner, dir(p), gap});
    }
    return ties;
}

double temperature_rate_coefficient(const Material& hairspring, const Material& balance) {
    return 86400.0 * (0.5 * hairspring.thermoelastic + 1.5 * hairspring.cte - balance.cte);
}

}  // namespace ohs::fea
