// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <numbers>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

namespace ohs::cases {

namespace {

using mesh::V3;
constexpr double kPi = std::numbers::pi;

fea::Material steel(double nu = 0.3) {
    fea::Material m;
    m.name = "steel";
    m.youngs_gpa = 210;
    m.poisson = nu;
    m.density = 7850;
    m.cte = 12e-6;
    return m;
}

mesh::TetMesh box(double L, double w, double h, int nx, int ny, int nz, int order) {
    return mesh::extrude(mesh::structured_rect(0, 0, L, w, nx, ny), 0, h, nz, order);
}

Check relative(std::string name, std::string category, double expected, double actual, double tol, std::string note = "") {
    Check c{std::move(name), std::move(category), expected, actual, tol, false, std::move(note)};
    const double scale = std::abs(expected) > 0 ? std::abs(expected) : 1.0;
    c.passed = std::abs(actual - expected) <= tol * scale;
    return c;
}

// Clamped end at x = 0, tip load downwards: returns the mean tip deflection.
double cantilever_tip(int nx, int ny, int nz, int order, double L, double w, double h, double P, double nu) {
    fea::Model model;
    model.material = steel(nu);
    model.mesh = box(L, w, h, nx, ny, nz, order);
    for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
        for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
    }
    const auto faces = mesh::boundary_faces(model.mesh);
    fea::Vec f = fea::Vec::Zero(model.dofs());
    fea::add_traction(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3&) { return c.x > L - 1e-9; }),
                      {0, 0, -P / (w * h)});
    const fea::StaticResult r = fea::solve_static(model, f);
    double sum = 0;
    int count = 0;
    for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
        sum += r.u[3 * n + 2];
        ++count;
    }
    return -sum / count;
}

double lame_hoop(double size, int ring_points) {
    const double a = 5, b = 10, p = 10.0;
    mesh::Region region;
    mesh::Ring outer, inner;
    for (int i = 0; i < ring_points; ++i) {
        const double t = 2 * kPi * i / ring_points;
        outer.push_back({b * std::cos(t), b * std::sin(t)});
        inner.push_back({a * std::cos(t), a * std::sin(t)});
    }
    region.rings = {outer, inner};
    mesh::Mesh2dOptions o;
    o.size = size;
    fea::Model model;
    model.material = steel();
    model.mesh = mesh::extrude(mesh::triangulate(region, o), 0, size, 1, 2);
    const double eps = 1e-9;
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) model.fixed.push_back({static_cast<int>(n), 2});
    const int east = mesh::nodes_where(model.mesh, [&](const V3& q) { return std::abs(q.x - b) < eps && std::abs(q.y) < eps && q.z < eps; })[0];
    const int west = mesh::nodes_where(model.mesh, [&](const V3& q) { return std::abs(q.x + b) < eps && std::abs(q.y) < eps && q.z < eps; })[0];
    model.fixed.push_back({east, 1});
    model.fixed.push_back({west, 1});
    model.fixed.push_back({west, 0});
    const auto faces = mesh::boundary_faces(model.mesh);
    fea::Vec f = fea::Vec::Zero(model.dofs());
    fea::add_pressure(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) {
                          return std::abs(n.z) < 0.5 && std::hypot(c.x, c.y) < 0.5 * (a + b);
                      }),
                      p);
    const fea::StaticResult r = fea::solve_static(model, f);
    double hoop = 0;
    int count = 0;
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& q = model.mesh.nodes[n];
        if (std::abs(std::hypot(q.x, q.y) - a) < 0.02 * a) {
            const double t = std::atan2(q.y, q.x);
            const fea::Stress& s = r.stress[n];
            hoop += s[0] * std::sin(t) * std::sin(t) + s[1] * std::cos(t) * std::cos(t) - 2 * s[3] * std::sin(t) * std::cos(t);
            ++count;
        }
    }
    return hoop / count;
}

double kirsch_peak() {
    const double r = 1, half_w = 10, half_l = 30, sigma = 100.0;
    mesh::Ring ring{{r, 0}, {half_w, 0}, {half_w, half_l}, {0, half_l}, {0, r}};
    for (int k = 1; k < 60; ++k) {
        const double t = kPi / 2 - (kPi / 2) * k / 60;
        ring.push_back({r * std::cos(t), r * std::sin(t)});
    }
    mesh::Mesh2dOptions mo;
    mo.size = 3.0;
    mo.size_at = [&](mesh::P2 p) { return std::min(3.0, 0.1 + 0.2 * std::max(0.0, std::hypot(p.x, p.y) - r)); };
    fea::Model model;
    model.material = steel();
    model.mesh = mesh::extrude(mesh::triangulate({{ring}}, mo), 0, 0.2, 1, 2);
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& p = model.mesh.nodes[n];
        model.fixed.push_back({static_cast<int>(n), 2});
        if (p.x < 1e-9) model.fixed.push_back({static_cast<int>(n), 0});
        if (p.y < 1e-9) model.fixed.push_back({static_cast<int>(n), 1});
    }
    const auto faces = mesh::boundary_faces(model.mesh);
    fea::Vec f = fea::Vec::Zero(model.dofs());
    fea::add_traction(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) { return c.y > half_l - 1e-9 && n.y > 0.5; }),
                      {0, sigma, 0});
    const fea::StaticResult res = fea::solve_static(model, f);
    double peak = 0;
    int count = 0;
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& p = model.mesh.nodes[n];
        if (std::abs(p.x - r) < 1e-9 && std::abs(p.y) < 1e-9) {
            peak += res.stress[n][1];
            ++count;
        }
    }
    return peak / count / sigma;
}

// Manufactured solution u_x = a sin(pi x / L) on a bar held at both ends: returns the L2 error.
double sine_bar_error(int k, int order) {
    const double L = 4, a = 1e-3;
    fea::Model model;
    model.material = steel(0.0);
    model.mesh = box(L, 1, 1, 4 * k, k, k, order);
    for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x < 1e-9 || p.x > L - 1e-9; })) model.fixed.push_back({n, 0});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 1});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 2});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y > 1 - 1e-9 && p.z < 1e-9; })[0], 2});
    const double E = model.material.youngs_mpa();
    fea::Vec f = fea::Vec::Zero(model.dofs());
    fea::add_body_force(model, f, [&](const V3& p) { return V3{E * a * (kPi / L) * (kPi / L) * std::sin(kPi * p.x / L), 0, 0}; });
    const fea::StaticResult r = fea::solve_static(model, f);
    return fea::l2_error(model, r.u, [&](const V3& p) { return V3{a * std::sin(kPi * p.x / L), 0, 0}; });
}

}  // namespace

std::vector<Check> run_physics_checks(bool full) {
    std::vector<Check> out;

    // ---- analytical ------------------------------------------------------------------------------------------
    {
        const double L = 20, w = 2, h = 1, P = 1.0;
        const double E = steel(0.0).youngs_mpa(), G = E / 2, I = w * h * h * h / 12, A = w * h;
        const double timoshenko = P * L * L * L / (3 * E * I) + P * L / (5.0 / 6.0 * G * A);
        out.push_back(relative("cantilever tip deflection (tet10, Timoshenko)", "analytical", timoshenko,
                               cantilever_tip(20, 2, 2, 2, L, w, h, P, 0.0), 0.01, "L/h = 20, one element through the depth"));
    }
    {
        fea::Model model;
        model.material = steel();
        const double L = 20, h = 1;
        model.mesh = box(L, 2, h, 20, 2, 2, 2);
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
            for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
        }
        const double E = model.material.youngs_mpa(), rho = model.material.density_t_mm3();
        const double f1 = 1.8751 * 1.8751 / (2 * kPi) * std::sqrt(E * h * h / (12 * rho * L * L * L * L));
        out.push_back(relative("cantilever first natural frequency (Euler-Bernoulli)", "analytical", f1,
                               fea::solve_modal(model, 1).frequencies_hz[0], 0.015));
    }
    {
        fea::Model model;
        model.material = steel();
        model.mesh = box(10, 2, 1, 5, 2, 1, 2);
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) model.fixed.push_back({n, 0});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 1});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 2});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y > 2 - 1e-9 && p.z < 1e-9; })[0], 2});
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x > 10 - 1e-9; })) model.fixed.push_back({n, 0});
        fea::Vec f = fea::Vec::Zero(model.dofs());
        fea::add_thermal(model, f, 30.0);
        fea::SolveOptions opt;
        opt.thermal_dT = 30.0;
        const fea::StaticResult r = fea::solve_static(model, f, opt);
        out.push_back(relative("thermal stress of a fully restrained bar, -E alpha dT", "analytical",
                               -model.material.youngs_mpa() * model.material.cte * 30.0, r.stress[0][0], 1e-6));
    }
    out.push_back(relative("thick cylinder bore hoop stress (Lame)", "analytical", 10.0 * (100 + 25) / (100 - 25), lame_hoop(2.0, 48), 0.01,
                           "plane strain, p = 10 MPa"));

    if (!full) return out;

    // ---- benchmarks -----------------------------------------------------------------------------------------------
    out.push_back(relative("stress concentration at a circular hole, d/W = 0.1 (Peterson)", "benchmark", 3.02, kirsch_peak(), 0.04));

    {
        const double L = 20, w = 2, h = 1;
        fea::Model model;
        model.material = steel(0.0);
        model.mesh = box(L, w, h, 20, 2, 2, 2);
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
            for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
        }
        const double EI = model.material.youngs_mpa() * w * h * h * h / 12;
        const auto faces = mesh::boundary_faces(model.mesh);
        fea::Vec f = fea::Vec::Zero(model.dofs());
        fea::add_traction(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3&) { return c.x > L - 1e-9; }),
                          {0, 0, -EI / (L * L) / (w * h)});
        fea::NonlinearOptions o;
        o.steps = 8;
        const fea::NonlinearResult r = fea::solve_nonlinear(model, f, o);
        double dz = 0;
        int n = 0;
        for (int node : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
            dz += r.u[3 * node + 2];
            ++n;
        }
        out.push_back(relative("large-deflection cantilever, alpha = P L^2 / EI = 1 (elastica)", "benchmark", 0.30172, -dz / n / L, 0.02,
                               r.converged ? "" : "Newton did not converge"));
    }
    {
        ToothCaseInput in;
        in.spec.teeth = 24;
        in.face_width = 2.0;
        in.tangential_load_n = 100.0;
        in.material = steel();
        const ToothCaseResult r = run_tooth_case(in);
        Check c = relative("tooth root stress over the Lewis stress", "benchmark", 1.6, r.concentration, 0.375,
                           "Lewis ignores the fillet: expect 1.0 to 2.2");
        out.push_back(c);
    }

    // ---- convergence -------------------------------------------------------------------------------------------------
    for (int order : {1, 2}) {
        const double e1 = sine_bar_error(1, order), e2 = sine_bar_error(2, order), e3 = sine_bar_error(4, order);
        const double rate = 0.5 * (std::log2(e1 / e2) + std::log2(e2 / e3));
        // Quadratic tetrahedra reach their O(h^3) L2 rate; linear ones on these Kuhn meshes show about first order
        // before the asymptotic range, so only first order is demanded of them.
        const double expected = order == 1 ? 1.0 : 3.0;
        Check c;
        c.name = std::string("L2 convergence rate of tet") + (order == 1 ? "4" : "10") + " on a manufactured solution";
        c.category = "convergence";
        c.expected = expected;
        c.actual = rate;
        c.tolerance = 0.1;
        c.passed = rate >= expected * (1 - c.tolerance);
        c.note = "cell sizes L/4, L/8, L/16 in all directions";
        out.push_back(c);
    }
    return out;
}

}  // namespace ohs::cases
