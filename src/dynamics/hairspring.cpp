// SPDX-License-Identifier: Apache-2.0
#include "dynamics/hairspring.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

#include "core/error.hpp"

namespace ohs::dynamics {

namespace {

constexpr double kPi = std::numbers::pi;

struct Node {
    double x, y;   ///< reference position, mm
    double psi;    ///< reference tangent direction, rad
};

std::vector<Node> spiral_nodes(const HairspringSpec& s) {
    if (s.turns <= 0 || s.elements_per_turn < 4 || s.pitch_mm <= 0 || s.inner_radius_mm <= 0) throw Error("hairspring: turns, pitch and inner radius must be positive");
    const int elements = static_cast<int>(std::ceil(s.turns * s.elements_per_turn));
    const double dr = s.pitch_mm / (2.0 * kPi);
    std::vector<Node> nodes;
    for (int i = 0; i <= elements; ++i) {
        const double phi = 2.0 * kPi * s.turns * i / elements;
        const double r = s.inner_radius_mm + dr * phi;
        nodes.push_back({r * std::cos(phi), r * std::sin(phi), phi + kPi / 2.0 + std::atan2(dr, r)});
    }
    return nodes;
}

double wrap(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

/// A co-rotational Euler-Bernoulli beam between two nodes. Unknowns per node: position (x, y) and the rotation r
/// of the cross-section away from its reference tangent. The strip is stress free in the reference spiral.
struct Beam {
    double L0, alpha0, EA, EI, psi1, psi2;

    // Internal forces on [x1 y1 r1 x2 y2 r2] (the same ordering as q), N and N·mm.
    std::array<double, 6> force(const std::array<double, 6>& q) const {
        const double dx = q[3] - q[0], dy = q[4] - q[1];
        const double Ln = std::hypot(dx, dy);
        const double c = dx / Ln, s = dy / Ln;
        // The chord has turned by about what its end sections have (they follow it closely), which may exceed pi on
        // the inner turns: unwrap relative to that, not to zero.
        const double guess = 0.5 * (q[2] + q[5]);
        const double db = wrap(std::atan2(dy, dx) - alpha0 - guess) + guess;
        const double ul = Ln - L0, phi1 = q[2] - db, phi2 = q[5] - db;
        const double N = EA / L0 * ul;
        const double M1 = 2.0 * EI / L0 * (2.0 * phi1 + phi2), M2 = 2.0 * EI / L0 * (phi1 + 2.0 * phi2);
        const double shear = (M1 + M2) / Ln;
        return {-N * c - shear * s, -N * s + shear * c, M1, N * c + shear * s, N * s - shear * c, M2};
    }
};

}  // namespace

double HairspringSpec::length_mm() const {
    const double dr = pitch_mm / (2.0 * kPi), phi = 2.0 * kPi * turns;
    // Arc length of r = r0 + dr phi: the integral of sqrt(r^2 + dr^2) dphi, closed form.
    auto F = [&](double ph) {
        const double r = inner_radius_mm + dr * ph;
        return 0.5 / dr * (r * std::sqrt(r * r + dr * dr) + dr * dr * std::asinh(r / dr));
    };
    return F(phi) - F(0.0);
}

double HairspringSpec::euler_stiffness() const {
    const double E = youngs_gpa * 1e9, b = width_mm * 1e-3, h = thickness_mm * 1e-3;
    return E * b * h * h * h / 12.0 / (length_mm() * 1e-3);
}

std::vector<HairspringPoint> hairspring_curve(const HairspringSpec& spec, const std::vector<double>& angles) {
    const std::vector<Node> ref = spiral_nodes(spec);
    const int nn = static_cast<int>(ref.size());
    const double E = spec.youngs_gpa * 1e3;  // N/mm^2
    const double A = spec.width_mm * spec.thickness_mm, I = spec.width_mm * std::pow(spec.thickness_mm, 3) / 12.0;
    std::vector<Beam> beams;
    for (int i = 0; i + 1 < nn; ++i) {
        const double dx = ref[i + 1].x - ref[i].x, dy = ref[i + 1].y - ref[i].y;
        beams.push_back({std::hypot(dx, dy), std::atan2(dy, dx), E * A, E * I, ref[i].psi, ref[i + 1].psi});
    }

    // Unknown vector: x y r of every node; the first node follows the collet and the last is the clamped stud.
    const int ndof = 3 * nn;
    Eigen::VectorXd q(ndof);
    for (int i = 0; i < nn; ++i) q.segment<3>(3 * i) << ref[i].x, ref[i].y, 0.0;
    auto assemble = [&](const Eigen::VectorXd& u, Eigen::VectorXd& f, Eigen::MatrixXd* K) {
        f = Eigen::VectorXd::Zero(ndof);
        if (K) *K = Eigen::MatrixXd::Zero(ndof, ndof);
        for (int e = 0; e + 1 < nn; ++e) {
            std::array<double, 6> qe{};
            for (int k = 0; k < 6; ++k) qe[static_cast<std::size_t>(k)] = u(3 * e + k);
            const auto fe = beams[static_cast<std::size_t>(e)].force(qe);
            for (int k = 0; k < 6; ++k) f(3 * e + k) += fe[static_cast<std::size_t>(k)];
            if (!K) continue;
            for (int k = 0; k < 6; ++k) {
                const double h = 1e-7;
                auto qp = qe, qm = qe;
                qp[static_cast<std::size_t>(k)] += h;
                qm[static_cast<std::size_t>(k)] -= h;
                const auto fp = beams[static_cast<std::size_t>(e)].force(qp), fm = beams[static_cast<std::size_t>(e)].force(qm);
                for (int j = 0; j < 6; ++j) (*K)(3 * e + j, 3 * e + k) += (fp[static_cast<std::size_t>(j)] - fm[static_cast<std::size_t>(j)]) / (2 * h);
            }
        }
    };

    std::vector<HairspringPoint> out;
    for (double target : angles) {
        // Load stepping from the reference configuration: restart each angle from zero so results do not depend on order.
        for (int i = 0; i < nn; ++i) q.segment<3>(3 * i) << ref[i].x, ref[i].y, 0.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(target) / 0.15)));
        Eigen::VectorXd f;
        Eigen::MatrixXd K;
        for (int step = 1; step <= steps; ++step) {
            const double a = target * step / steps;
            q(0) = ref[0].x * std::cos(a) - ref[0].y * std::sin(a);
            q(1) = ref[0].x * std::sin(a) + ref[0].y * std::cos(a);
            q(2) = a;
            bool converged = false;
            for (int it = 0; it < 60 && !converged; ++it) {
                assemble(q, f, &K);
                const int free_lo = 3, free_n = ndof - 6;  // every node but the collet and the stud
                Eigen::VectorXd rhs = -f.segment(free_lo, free_n);
                Eigen::VectorXd du = K.block(free_lo, free_lo, free_n, free_n).partialPivLu().solve(rhs);
                q.segment(free_lo, free_n) += du;
                converged = du.cwiseAbs().maxCoeff() < 1e-9;  // mm and rad
            }
            if (!converged) throw Error("hairspring: the equilibrium iteration did not converge at a collet rotation of " + std::to_string(a) + " rad");
        }
        assemble(q, f, nullptr);
        // Force and moment the staff applies to the collet node to hold the spring there (N, N·mm).
        const double fx = f(0), fy = f(1), mz = f(2);
        const double torque_nmm = q(0) * fy - q(1) * fx + mz;
        out.push_back({target, torque_nmm * 1e-3, std::hypot(fx, fy)});
    }
    return out;
}

HairspringFit fit_hairspring(const HairspringSpec& spec, double amplitude) {
    if (amplitude <= 0) throw Error("hairspring: the amplitude must be positive");
    const int n = 6;
    std::vector<double> angles;
    for (int i = 1; i <= n; ++i) {
        angles.push_back(amplitude * i / n);
        angles.push_back(-amplitude * i / n);
    }
    const auto curve = hairspring_curve(spec, angles);
    // The odd part of the torque, tau_o(a) = (tau(a) - tau(-a)) / 2, fitted by k a + c a^3.
    double s_aa = 0, s_a4 = 0, s_a6 = 0, s_ta = 0, s_ta3 = 0, odd_max = 0, even_max = 0, side = 0;
    for (int i = 0; i < n; ++i) {
        const auto& p = curve[static_cast<std::size_t>(2 * i)];
        const auto& m = curve[static_cast<std::size_t>(2 * i + 1)];
        const double a = p.angle_rad, odd = 0.5 * (p.torque_nm - m.torque_nm), even = 0.5 * (p.torque_nm + m.torque_nm);
        s_aa += a * a;
        s_a4 += std::pow(a, 4);
        s_a6 += std::pow(a, 6);
        s_ta += odd * a;
        s_ta3 += odd * a * a * a;
        odd_max = std::max(odd_max, std::abs(odd));
        even_max = std::max(even_max, std::abs(even));
        side = std::max({side, p.side_force_n, m.side_force_n});
    }
    const double det = s_aa * s_a6 - s_a4 * s_a4;
    HairspringFit fit;
    fit.stiffness_nm_rad = (s_ta * s_a6 - s_ta3 * s_a4) / det;
    fit.cubic_nm_rad3 = (s_aa * s_ta3 - s_a4 * s_ta) / det;
    fit.asymmetry = odd_max > 0 ? even_max / odd_max : 0.0;
    fit.euler_stiffness = spec.euler_stiffness();
    fit.max_side_force_n = side;
    return fit;
}

}  // namespace ohs::dynamics
