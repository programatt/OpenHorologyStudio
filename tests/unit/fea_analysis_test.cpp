// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "fea/analysis.hpp"
#include "fea/fea.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

using namespace ohs;
using namespace ohs::fea;
using mesh::V3;
using Catch::Approx;

namespace {

constexpr double kPi = std::numbers::pi;

Material make_material(double E_gpa, double nu, double rho = 7850, double cte = 12e-6) {
    Material m;
    m.name = "test";
    m.youngs_gpa = E_gpa;
    m.poisson = nu;
    m.density = rho;
    m.cte = cte;
    return m;
}

mesh::TetMesh box(double L, double w, double h, int nx, int ny, int nz, int order) {
    return mesh::extrude(mesh::structured_rect(0, 0, L, w, nx, ny), 0, h, nz, order);
}

void clamp_end(Model& model) {
    for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
        for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
    }
}

// Isostatic supports for a bar along x: x on the x=0 face, plus three single components.
void isostatic_bar(Model& model, double w) {
    for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) model.fixed.push_back({n, 0});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 1});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 2});
    model.fixed.push_back({mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x < 1e-9 && p.y > w - 1e-9 && p.z < 1e-9; })[0], 2});
}

}  // namespace

TEST_CASE("verification: cantilever natural frequencies against Euler-Bernoulli") {
    // 20 x 2 x 1 mm steel: bending about the thin direction first (z), then the wide one (y).
    const double L = 20, w = 2, h = 1;
    Model model;
    model.material = make_material(210, 0.3);
    model.mesh = box(L, w, h, 20, 2, 2, 2);
    clamp_end(model);
    const double E = model.material.youngs_mpa(), rho = model.material.density_t_mm3();
    auto beam = [&](double beta_l, double thickness) {
        return beta_l * beta_l / (2 * kPi) * std::sqrt(E * thickness * thickness / (12 * rho * L * L * L * L));
    };
    const ModalResult r = solve_modal(model, 4);
    REQUIRE(r.frequencies_hz.size() == 4);
    CHECK(r.rigid_modes_dropped == 0);
    CHECK(r.frequencies_hz[0] == Approx(beam(1.8751, h)).epsilon(0.015));
    CHECK(r.frequencies_hz[1] == Approx(beam(1.8751, w)).epsilon(0.015));
    for (std::size_t i = 1; i < r.frequencies_hz.size(); ++i) CHECK(r.frequencies_hz[i] >= r.frequencies_hz[i - 1]);

    // Mass normalisation: phi^T M phi = 1.
    const Sparse M = assemble_mass(model);
    CHECK(r.modes[0].dot(M * r.modes[0]) == Approx(1.0).epsilon(1e-8));
    CHECK(r.modes[0].dot(M * r.modes[1]) == Approx(0.0).margin(1e-8));
}

TEST_CASE("verification: free-free bar drops six rigid-body modes and keeps the beam spectrum") {
    const double L = 20, w = 1, h = 1;
    Model model;
    model.material = make_material(210, 0.3);
    model.mesh = box(L, w, h, 16, 1, 1, 2);
    const double E = model.material.youngs_mpa(), rho = model.material.density_t_mm3();
    const ModalResult r = solve_modal(model, 2);
    CHECK(r.rigid_modes_dropped == 6);
    const double f1 = 4.7300 * 4.7300 / (2 * kPi) * std::sqrt(E * h * h / (12 * rho * L * L * L * L));
    REQUIRE(r.frequencies_hz.size() == 2);
    CHECK(r.frequencies_hz[0] == Approx(f1).epsilon(0.03));
    CHECK(r.frequencies_hz[1] == Approx(f1).epsilon(0.03));  // square section: the two bending planes are degenerate
}

TEST_CASE("verification: a step load doubles the static deflection and rings at the first frequency") {
    const double L = 20, w = 2, h = 1;
    Model model;
    model.material = make_material(210, 0.3);
    model.mesh = box(L, w, h, 20, 2, 2, 2);
    clamp_end(model);
    const auto faces = mesh::boundary_faces(model.mesh);
    const auto tip_faces = mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3&) { return c.x > L - 1e-9; });
    Vec f = Vec::Zero(model.dofs());
    add_traction(model, f, tip_faces, {0, 0, -0.5});  // 1 N over the 2 mm2 tip

    const StaticResult stat = solve_static(model, f);
    const int tip = mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9 && p.y < 1e-9 && p.z > h - 1e-9; })[0];
    const double static_tip = stat.u[3 * tip + 2];

    const ModalResult modal = solve_modal(model, 1);
    const double period = 1.0 / modal.frequencies_hz[0];
    TransientOptions o;
    o.dt = period / 200;
    o.duration = 2.2 * period;
    const TransientResult tr = solve_transient(model, o, [&](double) { return f; }, nullptr, {3 * tip + 2});
    const auto& hist = tr.probes[0];
    const double peak = *std::min_element(hist.begin(), hist.end());  // downward
    CHECK(peak / static_tip == Approx(2.0).epsilon(0.05));
    // First maximum of the deflection occurs half a period after the load is applied.
    std::size_t first = 1;
    while (first + 1 < hist.size() && hist[first + 1] <= hist[first]) ++first;
    const double t_peak = tr.time[first];
    CHECK(t_peak == Approx(0.5 * period).epsilon(0.03));
    CHECK(hist.back() < 0.0);  // average-acceleration Newmark does not damp: still swinging
}

TEST_CASE("verification: a support acceleration loads the beam like gravity does") {
    const double L = 20, w = 2, h = 1;
    Model model;
    model.material = make_material(210, 0.3);
    model.mesh = box(L, w, h, 12, 2, 2, 2);
    clamp_end(model);
    const double a = 9810.0 * 50;  // 50 g, in mm/s^2
    Vec g = Vec::Zero(model.dofs());
    add_gravity(model, g, {0, 0, -a});
    const StaticResult stat = solve_static(model, g);
    const int tip = mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9 && p.y < 1e-9 && p.z > h - 1e-9; })[0];

    const ModalResult modal = solve_modal(model, 1);
    const double period = 1.0 / modal.frequencies_hz[0];
    TransientOptions o;
    o.dt = period / 200;
    o.duration = 1.2 * period;
    // The base moves up with +a, so relative to it the beam sees -a inertia.
    const TransientResult tr = solve_transient(model, o, nullptr, [&](double) { return V3{0, 0, a}; }, {3 * tip + 2});
    const double peak = *std::min_element(tr.probes[0].begin(), tr.probes[0].end());
    CHECK(peak / stat.u[3 * tip + 2] == Approx(2.0).epsilon(0.05));
}

TEST_CASE("verification: thermal expansion, free and fully restrained") {
    const double L = 10, w = 2, h = 1, dT = 30;
    Model model;
    model.material = make_material(210, 0.3, 7850, 12e-6);
    model.mesh = box(L, w, h, 5, 2, 1, 2);
    isostatic_bar(model, w);
    const double E = model.material.youngs_mpa(), alpha = model.material.cte;

    // Free to grow: no stress, the far end moves alpha dT L.
    Vec f = Vec::Zero(model.dofs());
    add_thermal(model, f, dT);
    SolveOptions opt;
    opt.thermal_dT = dT;
    StaticResult r = solve_static(model, f, opt);
    for (const Stress& s : r.stress) CHECK(std::abs(s[0]) < 1e-6 * E);
    for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
        CHECK(r.u[3 * n] == Approx(alpha * dT * L).epsilon(1e-8));
    }

    // Held at both ends: sigma = -E alpha dT.
    for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) model.fixed.push_back({n, 0});
    r = solve_static(model, f, opt);
    for (const Stress& s : r.stress) {
        CHECK(s[0] == Approx(-E * alpha * dT).epsilon(1e-6));
        CHECK(std::abs(s[1]) < 1e-3);
    }
}

TEST_CASE("temperature coefficient of the rate follows e/2 + 3 a_spring/2 - a_balance") {
    const Material& elinvar = find_material("nife_hairspring");
    const Material& steel = find_material("steel_hardened");
    const Material& brass = find_material("brass_leaded");
    CHECK(temperature_rate_coefficient(elinvar, brass) ==
          Approx(86400.0 * (0.5 * elinvar.thermoelastic + 1.5 * elinvar.cte - brass.cte)));
    // An uncompensated steel spring on a steel balance loses about 10 s/day per kelvin as it warms;
    // the Ni-Fe alloy brings that down by an order of magnitude.
    CHECK(temperature_rate_coefficient(steel, steel) < -8.0);
    CHECK(temperature_rate_coefficient(steel, steel) > -13.0);
    CHECK(std::abs(temperature_rate_coefficient(elinvar, steel)) < 2.0);
}

TEST_CASE("bodies of different materials in series share the load like springs") {
    Model model;
    model.material = make_material(210, 0.0);
    model.materials = {make_material(210, 0.0), make_material(97, 0.0)};
    model.mesh = box(10, 2, 1, 4, 2, 1, 2);
    for (std::size_t e = 0; e < model.mesh.elems.size(); ++e) {
        double cx = 0;
        for (int k = 0; k < 4; ++k) cx += model.mesh.nodes[model.mesh.elems[e][k]].x / 4;
        model.element_material.push_back(cx < 5 ? 0 : 1);
    }
    isostatic_bar(model, 2);
    const auto faces = mesh::boundary_faces(model.mesh);
    Vec f = Vec::Zero(model.dofs());
    add_traction(model, f, mesh::faces_where(model.mesh, faces, [](const V3& c, const V3&) { return c.x > 10 - 1e-9; }), {50, 0, 0});
    const StaticResult r = solve_static(model, f);
    const int far = mesh::nodes_where(model.mesh, [](const V3& p) { return p.x > 10 - 1e-9; })[0];
    CHECK(r.u[3 * far] == Approx(50.0 * (5 / 210000.0 + 5 / 97000.0)).epsilon(1e-8));
    // Stress is the same in both halves; strain is not.
    for (const Stress& s : r.stress) CHECK(s[0] == Approx(50.0).epsilon(1e-6));
}

TEST_CASE("verification: the nonlinear tangent is the derivative of the internal force") {
    for (int order : {1, 2}) {
        Model model;
        model.material = make_material(210, 0.3);
        model.mesh = box(2, 1, 1, 2, 1, 1, order);
        std::mt19937 rng(11);
        std::uniform_real_distribution<double> dist(-0.05, 0.05);
        Vec u(model.dofs());
        for (Eigen::Index i = 0; i < u.size(); ++i) u[i] = dist(rng);
        const Eigen::MatrixXd K = Eigen::MatrixXd(nonlinear_tangent(model, u));
        const double h = 1e-6;
        Eigen::MatrixXd fd(u.size(), u.size());
        for (Eigen::Index j = 0; j < u.size(); ++j) {
            Vec up = u, um = u;
            up[j] += h;
            um[j] -= h;
            fd.col(j) = (nonlinear_internal_force(model, up) - nonlinear_internal_force(model, um)) / (2 * h);
        }
        CHECK((K - fd).norm() / K.norm() < 1e-6);
        CHECK((K - K.transpose()).norm() / K.norm() < 1e-12);
        // And at u = 0 it is the small-strain stiffness.
        const Eigen::MatrixXd K0 = Eigen::MatrixXd(nonlinear_tangent(model, Vec::Zero(model.dofs())));
        const Eigen::MatrixXd Klin = Eigen::MatrixXd(assemble_stiffness(model));
        CHECK((K0 - Klin).norm() / Klin.norm() < 1e-10);
    }
}

TEST_CASE("verification: large-deflection cantilever against the elastica") {
    // Tip load P with alpha = P L^2 / (E I). Mattiasson's solutions: alpha = 1 gives w/L = 0.30172
    // and u/L = 0.05612; alpha = 2 gives 0.49346 and 0.16064.
    const double L = 20, w = 2, h = 1;
    Model model;
    model.material = make_material(210, 0.0);
    model.mesh = box(L, w, h, 20, 2, 2, 2);
    clamp_end(model);
    const double EI = model.material.youngs_mpa() * w * h * h * h / 12;
    const auto faces = mesh::boundary_faces(model.mesh);
    const auto tip_faces = mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3&) { return c.x > L - 1e-9; });
    const int tip = mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9 && p.y < 1e-9 && p.z < h / 2 + 1e-9 && p.z > h / 2 - 1e-9; })[0];
    (void)tip;

    for (auto [alpha, wl, ul] : {std::tuple{1.0, 0.30172, 0.05612}, std::tuple{2.0, 0.49346, 0.16064}}) {
        const double P = alpha * EI / (L * L);
        Vec f = Vec::Zero(model.dofs());
        add_traction(model, f, tip_faces, {0, 0, -P / (w * h)});
        NonlinearOptions o;
        o.steps = 8;
        const NonlinearResult r = solve_nonlinear(model, f, o);
        REQUIRE(r.converged);
        // Average over the tip face nodes (the neutral axis moves with the face centroid).
        double dz = 0, dx = 0;
        int n = 0;
        for (int node : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
            dz += r.u[3 * node + 2];
            dx += r.u[3 * node];
            ++n;
        }
        INFO("alpha " << alpha);
        CHECK(-dz / n / L == Approx(wl).epsilon(0.02));
        CHECK(-dx / n / L == Approx(ul).epsilon(0.04));
        CHECK(r.total_iterations < 8 * 12);
    }
    // The linear solution overestimates the deflection at alpha = 2, as it must.
    const double P = 2 * EI / (L * L);
    Vec f = Vec::Zero(model.dofs());
    add_traction(model, f, tip_faces, {0, 0, -P / (w * h)});
    const StaticResult lin = solve_static(model, f);
    double dz = 0;
    int n = 0;
    for (int node : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
        dz += lin.u[3 * node + 2];
        ++n;
    }
    CHECK(-dz / n / L > 0.6);
}

TEST_CASE("verification: Hertz line contact of a cylinder on a rigid plane") {
    const double R = 10, thickness = 0.4, line_load = 100.0;  // N/mm over the full width
    const double E = 210000, nu = 0.3;
    const double b_hertz = std::sqrt(4 * line_load * R * (1 - nu * nu) / (kPi * E));
    const double p0_hertz = 2 * line_load / (kPi * b_hertz);

    // Half the disc (x >= 0) centred at (0, R) touching y = 0, graded towards the contact point.
    const double h0 = 0.012;
    auto size_at = [&](mesh::P2 p) { return std::min(1.2, h0 + 0.12 * std::hypot(p.x, p.y)); };
    mesh::Ring ring;
    double theta = 0;
    while (theta < kPi) {
        const mesh::P2 p{R * std::sin(theta), R - R * std::cos(theta)};
        ring.push_back(p);
        theta += std::min(0.3, 0.8 * size_at(p) / R);
    }
    ring.push_back({0, 2 * R});
    mesh::Mesh2dOptions mo;
    mo.size = 1.2;
    mo.size_at = size_at;
    Model model;
    model.material = make_material(E / 1000.0, nu);
    model.mesh = mesh::extrude(mesh::triangulate({{ring}}, mo), 0, thickness, 1, 2);

    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        model.fixed.push_back({static_cast<int>(n), 2});
        if (model.mesh.nodes[n].x < 1e-9) model.fixed.push_back({static_cast<int>(n), 0});
    }
    const auto faces = mesh::boundary_faces(model.mesh);
    const auto top = mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) { return c.y > 2 * R - 0.4 && n.y > 0.3; });
    double top_area = 0;
    for (const auto& f : top) top_area += mesh::face_area(model.mesh, f);
    const double half_force = 0.5 * line_load * thickness;
    Vec load = Vec::Zero(model.dofs());
    add_traction(model, load, top, {0, -half_force / top_area, 0});

    const auto contact_faces = mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) { return c.y < 0.6 && n.y < -0.3; });
    const ContactResult cr = solve_contact(model, load, {{0, 0, 0}, {0, 1, 0}}, contact_faces);

    // The contact force balances the applied force exactly.
    CHECK(cr.total_force == Approx(half_force).epsilon(1e-6));
    // Contact half-width: the outermost active node, within a node spacing of the Hertz value.
    double reach = 0, peak = 0;
    for (std::size_t i = 0; i < cr.active_nodes.size(); ++i) {
        reach = std::max(reach, model.mesh.nodes[cr.active_nodes[i]].x);
        peak = std::max(peak, cr.pressure[i]);
    }
    CHECK(reach == Approx(b_hertz).epsilon(0.12));
    // Pressure peak near the centre: within 25 % of Hertz (nodal pressures are noisy).
    CHECK(peak == Approx(p0_hertz).epsilon(0.25));
    CHECK(cr.active_nodes.size() > 10);
}

TEST_CASE("verification: shrink fit of a shaft in a hub (Lame, plane strain)") {
    const double r = 3, ro = 6, delta = 0.01;
    const Material hub_mat = make_material(97, 0.34, 8440), shaft_mat = make_material(210, 0.30, 7850);
    auto disc = [&](double inner, double outer) {
        mesh::Region region;
        mesh::Ring out;
        for (int i = 0; i < 96; ++i) out.push_back({outer * std::cos(2 * kPi * i / 96), outer * std::sin(2 * kPi * i / 96)});
        region.rings.push_back(out);
        if (inner > 0) {
            mesh::Ring in;
            for (int i = 0; i < 96; ++i) in.push_back({inner * std::cos(2 * kPi * i / 96), inner * std::sin(2 * kPi * i / 96)});
            region.rings.push_back(in);
        }
        return region;
    };
    mesh::Mesh2dOptions mo;
    mo.size = 0.3;
    const auto hub = mesh::extrude(mesh::triangulate(disc(r, ro), mo), 0, 0.3, 1, 2);
    const auto shaft = mesh::extrude(mesh::triangulate(disc(0, r), mo), 0, 0.3, 1, 2);

    Model model;
    model.material = hub_mat;
    model.materials = {hub_mat, shaft_mat};
    model.mesh = hub;
    const int hub_nodes = static_cast<int>(hub.nodes.size());
    const int offset = append_mesh(model.mesh, shaft);
    for (std::size_t e = 0; e < model.mesh.elems.size(); ++e) model.element_material.push_back(model.mesh.elems[e][0] >= offset ? 1 : 0);

    std::vector<int> a_nodes, b_nodes;
    for (int n = 0; n < static_cast<int>(model.mesh.nodes.size()); ++n) {
        const V3& p = model.mesh.nodes[n];
        if (std::abs(std::hypot(p.x, p.y) - r) < 1e-7) (n < hub_nodes ? a_nodes : b_nodes).push_back(n);
    }
    // (u_hub - u_shaft) . r_hat = delta: the bore is smaller than the shaft by delta.
    model.ties = tie_coincident(model.mesh, a_nodes, b_nodes, [](const V3& p) { return V3{p.x / std::hypot(p.x, p.y), p.y / std::hypot(p.x, p.y), 0}; }, delta);
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) model.fixed.push_back({static_cast<int>(n), 2});
    auto find = [&](double x, double y, bool in_hub) {
        for (int n = in_hub ? 0 : hub_nodes; n < (in_hub ? hub_nodes : static_cast<int>(model.mesh.nodes.size())); ++n) {
            const V3& p = model.mesh.nodes[n];
            if (std::abs(p.x - x) < 1e-7 && std::abs(p.y - y) < 1e-7 && p.z < 1e-9) return n;
        }
        throw Error("test: node not found");
    };
    model.fixed.push_back({find(ro, 0, true), 1});
    model.fixed.push_back({find(-ro, 0, true), 1});
    model.fixed.push_back({find(-ro, 0, true), 0});
    model.fixed.push_back({find(r, 0, false), 1});  // the shaft may not spin

    const StaticResult res = solve_static(model, Vec::Zero(model.dofs()));

    auto plane_strain = [](const Material& m) { return std::pair{m.youngs_mpa() / (1 - m.poisson * m.poisson), m.poisson / (1 - m.poisson)}; };
    const auto [Eh, nuh] = plane_strain(hub_mat);
    const auto [Es, nus] = plane_strain(shaft_mat);
    const double p = delta / (r * ((1 / Eh) * ((ro * ro + r * r) / (ro * ro - r * r) + nuh) + (1 / Es) * (1 - nus)));

    // Interface pressure from the force the ties carry over the bore area, and the hub bore hoop
    // stress, which Lame ties to it by sigma_theta = p (ro^2 + r^2) / (ro^2 - r^2).
    double tie_force = 0, hoop = 0;
    for (const Tie& t : model.ties) {
        const double jump = (res.u[3 * t.a] - res.u[3 * t.b]) * t.dir.x + (res.u[3 * t.a + 1] - res.u[3 * t.b + 1]) * t.dir.y;
        tie_force += model.effective_tie_stiffness() * (t.gap - jump);
    }
    for (int n : a_nodes) {
        const V3& q = model.mesh.nodes[n];
        const double c = q.x / std::hypot(q.x, q.y), s = q.y / std::hypot(q.x, q.y);
        const Stress& st = res.stress[n];
        hoop += st[0] * s * s + st[1] * c * c - 2 * st[3] * c * s;
    }
    CHECK(tie_force / (2 * kPi * r * 0.3) == Approx(p).epsilon(0.04));
    CHECK(hoop / static_cast<double>(a_nodes.size()) == Approx(p * (ro * ro + r * r) / (ro * ro - r * r)).epsilon(0.04));
    // The tie is honoured: the radial displacement jump is the interference.
    for (std::size_t i = 0; i < model.ties.size(); i += 7) {
        const Tie& t = model.ties[i];
        const double jump = (res.u[3 * t.a] - res.u[3 * t.b]) * t.dir.x + (res.u[3 * t.a + 1] - res.u[3 * t.b + 1]) * t.dir.y;
        CHECK(jump == Approx(delta).epsilon(1e-3));
    }
    // A node without a partner is an error.
    CHECK_THROWS_AS(tie_coincident(model.mesh, a_nodes, {b_nodes[0]}, [](const V3&) { return V3{1, 0, 0}; }, 0.0), Error);
}

TEST_CASE("verification: stress concentration at a hole in a plate under tension") {
    // Quarter of a plate of width W = 20 with a hole of radius 1 (d/W = 0.1): peak stress 3.02 sigma.
    const double r = 1, half_w = 10, half_l = 30, sigma = 100.0;
    auto size_at = [&](mesh::P2 p) { return std::min(3.0, 0.1 + 0.2 * std::max(0.0, std::hypot(p.x, p.y) - r)); };
    mesh::Ring ring;
    ring.push_back({r, 0});
    ring.push_back({half_w, 0});
    ring.push_back({half_w, half_l});
    ring.push_back({0, half_l});
    ring.push_back({0, r});
    for (int k = 1; k < 60; ++k) {
        const double t = kPi / 2 - (kPi / 2) * k / 60;
        ring.push_back({r * std::cos(t), r * std::sin(t)});
    }
    mesh::Mesh2dOptions mo;
    mo.size = 3.0;
    mo.size_at = size_at;
    Model model;
    model.material = make_material(210, 0.3);
    model.mesh = mesh::extrude(mesh::triangulate({{ring}}, mo), 0, 0.2, 1, 2);
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& p = model.mesh.nodes[n];
        model.fixed.push_back({static_cast<int>(n), 2});
        if (p.x < 1e-9) model.fixed.push_back({static_cast<int>(n), 0});
        if (p.y < 1e-9) model.fixed.push_back({static_cast<int>(n), 1});
    }
    const auto faces = mesh::boundary_faces(model.mesh);
    Vec f = Vec::Zero(model.dofs());
    add_traction(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) { return c.y > half_l - 1e-9 && n.y > 0.5; }),
                 {0, sigma, 0});
    const StaticResult res = solve_static(model, f);

    // The peak is at the side of the hole, where it meets the x axis (perpendicular to the load).
    double peak = 0;
    int count = 0;
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& p = model.mesh.nodes[n];
        if (std::abs(p.x - r) < 1e-9 && std::abs(p.y) < 1e-9) {
            peak += res.stress[n][1];
            ++count;
        }
    }
    REQUIRE(count >= 2);
    CHECK(peak / count / sigma == Approx(3.02).epsilon(0.04));
    // Far from the hole the stress is the applied one.
    for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
        const V3& p = model.mesh.nodes[n];
        if (p.y > 25 && p.x > 8) CHECK(res.stress[n][1] == Approx(sigma).epsilon(0.05));
    }
}
