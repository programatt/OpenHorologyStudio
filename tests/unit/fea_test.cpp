// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "fea/element.hpp"
#include "fea/fea.hpp"
#include "fea/material.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

using namespace ohs;
using namespace ohs::fea;
using mesh::V3;
using Catch::Approx;

namespace {

Material steel(double nu = 0.3) {
    Material m;
    m.name = "test";
    m.youngs_gpa = 210;
    m.poisson = nu;
    m.density = 7850;
    m.cte = 12e-6;
    return m;
}

// A box [0,L] x [0,w] x [0,h], structured, in the requested element order.
mesh::TetMesh box(double L, double w, double h, int nx, int ny, int nz, int order) {
    return mesh::extrude(mesh::structured_rect(0, 0, L, w, nx, ny), 0, h, nz, order);
}

double factorial(int n) { return n <= 1 ? 1.0 : n * factorial(n - 1); }

}  // namespace

TEST_CASE("quadrature rules integrate monomials exactly up to their degree") {
    // Integral of L0^a L1^b L2^c L3^d over the reference tetrahedron: a!b!c!d! 3! / (a+b+c+d+3)! / 6.
    for (auto [degree, expect_count] : {std::pair{1, 1}, std::pair{2, 4}, std::pair{5, 14}}) {
        const Quadrature& q = tet_rule(degree);
        REQUIRE(static_cast<int>(q.points.size()) == expect_count);
        double wsum = 0;
        for (double w : q.weights) wsum += w;
        CHECK(wsum == Approx(1.0 / 6.0));
        for (int a = 0; a <= degree; ++a) {
            for (int b = 0; a + b <= degree; ++b) {
                for (int c = 0; a + b + c <= degree; ++c) {
                    const int d = std::min(degree - a - b - c, 1);
                    double sum = 0;
                    for (std::size_t g = 0; g < q.points.size(); ++g) {
                        sum += q.weights[g] * std::pow(q.points[g][0], a) * std::pow(q.points[g][1], b) *
                               std::pow(q.points[g][2], c) * std::pow(q.points[g][3], d);
                    }
                    const double exact = factorial(a) * factorial(b) * factorial(c) * factorial(d) * 6.0 / factorial(a + b + c + d + 3) / 6.0;
                    INFO("degree " << degree << " exponents " << a << b << c << d);
                    CHECK(sum == Approx(exact).epsilon(1e-10));
                }
            }
        }
    }
}

TEST_CASE("shape functions: partition of unity, nodal interpolation, exact gradients") {
    for (int order : {1, 2}) {
        mesh::TetMesh m;
        m.nodes = {{0.1, 0.0, 0.2}, {1.3, 0.1, 0.0}, {0.2, 1.1, 0.1}, {0.0, 0.2, 0.9}};
        m.elems = {{0, 1, 2, 3, -1, -1, -1, -1, -1, -1}};
        if (order == 2) mesh::promote_to_quadratic(m);
        const int npe = m.nodes_per_element();
        for (int i = 0; i < npe; ++i) {
            const Shape s = evaluate_shape(m, 0, node_bary(i));
            for (int j = 0; j < npe; ++j) CHECK(s.N[j] == Approx(i == j ? 1.0 : 0.0).margin(1e-14));
        }
        const Shape s = evaluate_shape(m, 0, {0.1, 0.2, 0.3, 0.4});
        CHECK(s.N.sum() == Approx(1.0));
        CHECK(s.dNdx.colwise().sum().cwiseAbs().maxCoeff() < 1e-12);
        CHECK(s.detJ == Approx(6.0 * mesh::volume(m)));
        // A linear field f = 2x - y + 3z + 1 is interpolated exactly, with exact gradient.
        double grad[3] = {0, 0, 0};
        for (int i = 0; i < npe; ++i) {
            const V3& p = m.nodes[m.elems[0][i]];
            const double f = 2 * p.x - p.y + 3 * p.z + 1;
            for (int k = 0; k < 3; ++k) grad[k] += f * s.dNdx(i, k);
        }
        CHECK(grad[0] == Approx(2.0));
        CHECK(grad[1] == Approx(-1.0));
        CHECK(grad[2] == Approx(3.0));
    }
}

TEST_CASE("extruded tetrahedra fill the box, agree across faces and stay positive") {
    for (int order : {1, 2}) {
        const auto m = box(4, 2, 1, 4, 2, 2, order);
        CHECK(mesh::volume(m) == Approx(8.0));
        CHECK(mesh::min_quality(m) > 0.2);
        const auto faces = mesh::boundary_faces(m);
        double area = 0, nx = 0, ny = 0, nz = 0;
        for (const auto& f : faces) {
            const V3 n = mesh::face_normal(m, f);
            const double a = mesh::face_area(m, f);
            area += a;
            nx += n.x * a;
            ny += n.y * a;
            nz += n.z * a;
        }
        CHECK(area == Approx(2 * (4 * 2 + 4 * 1 + 2 * 1)));
        CHECK(std::abs(nx) + std::abs(ny) + std::abs(nz) < 1e-12);  // a closed surface
        // Outward: the centroid of every face lies on the box boundary, with the normal pointing away.
        for (const auto& f : faces) {
            const V3 c = mesh::face_centroid(m, f), n = mesh::face_normal(m, f);
            const V3 inside{c.x - 1e-6 * n.x, c.y - 1e-6 * n.y, c.z - 1e-6 * n.z};
            CHECK(inside.x >= -1e-9);
            CHECK(inside.x <= 4 + 1e-9);
            CHECK(inside.y >= -1e-9);
            CHECK(inside.y <= 2 + 1e-9);
            CHECK(inside.z >= -1e-9);
            CHECK(inside.z <= 1 + 1e-9);
        }
        if (order == 2) CHECK(m.nodes.size() > box(4, 2, 1, 4, 2, 2, 1).nodes.size() * 3);
    }
}

TEST_CASE("material catalogue loads, looks up and reports errors") {
    const auto& all = builtin_materials();
    CHECK(all.size() >= 8);
    const Material& brass = find_material("brass_leaded");
    CHECK(brass.youngs_mpa() == Approx(97000));
    CHECK(brass.density_t_mm3() == Approx(8.44e-9));
    CHECK(brass.shear_mpa() == Approx(97000 / (2 * 1.34)));
    CHECK_FALSE(brass.source.empty());
    // The hairspring alloy is the one with a nearly zero thermoelastic coefficient.
    CHECK(std::abs(find_material("nife_hairspring").thermoelastic) < 1e-5);
    CHECK(find_material("steel_hardened").thermoelastic < -1e-4);
    CHECK_THROWS_AS(find_material("unobtainium"), Error);

    const auto custom = materials_from_toml("[[material]]\nname='x'\nyoungs_gpa=100\ndensity=5000\n");
    CHECK(find_material("x", custom).poisson == Approx(0.3));
    CHECK_THROWS_AS(materials_from_toml("[[material]]\nname='x'\nyoungs_gpa=-1\ndensity=5000\n"), Error);
    CHECK_THROWS_AS(materials_from_toml("[[material]]\nyoungs_gpa=100\ndensity=5000\n"), Error);
    CHECK_THROWS_AS(materials_from_toml("nothing = 1"), Error);
}

TEST_CASE("verification: uniaxial tension gives sigma = F/A and u = F L / (A E)") {
    for (int order : {1, 2}) {
        Model model;
        model.material = steel(0.3);
        model.mesh = box(10, 2, 1, 5, 2, 1, order);
        const auto faces = mesh::boundary_faces(model.mesh);
        // Isostatic supports: x fixed on the x=0 face, rigid-body rotations removed by two nodes.
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) model.fixed.push_back({n, 0});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 1});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y < 1e-9 && p.z < 1e-9; })[0], 2});
        model.fixed.push_back({mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9 && p.y > 2 - 1e-9 && p.z < 1e-9; })[0], 2});

        Vec f = Vec::Zero(model.dofs());
        const auto end = mesh::faces_where(model.mesh, faces, [](const V3& c, const V3&) { return c.x > 10 - 1e-9; });
        add_traction(model, f, end, {100.0, 0, 0});  // 100 MPa
        const StaticResult r = solve_static(model, f);

        const double E = model.material.youngs_mpa();
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x > 10 - 1e-9; })) {
            CHECK(r.u[3 * n] == Approx(100.0 * 10 / E).epsilon(1e-9));
        }
        for (const Stress& s : r.stress) {
            CHECK(s[0] == Approx(100.0).epsilon(1e-8));
            CHECK(std::abs(s[1]) < 1e-6);
            CHECK(std::abs(s[3]) < 1e-6);
        }
        // Lateral contraction: -nu * strain * width at the far corner.
        const int corner = mesh::nodes_where(model.mesh, [](const V3& p) { return p.x > 10 - 1e-9 && p.y > 2 - 1e-9 && p.z < 1e-9; })[0];
        CHECK(r.u[3 * corner + 1] == Approx(-0.3 * 100.0 / E * 2).epsilon(1e-8));
        // Equilibrium: the support reactions balance the applied force, 100 N/mm2 * 2 mm2.
        double rx = 0;
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) rx += r.reaction[3 * n];
        CHECK(rx == Approx(-200.0).epsilon(1e-9));
        CHECK(r.strain_energy == Approx(0.5 * 200.0 * 100.0 * 10 / E).epsilon(1e-9));
    }
}

TEST_CASE("verification: a linear displacement field is reproduced exactly (patch test)") {
    for (int order : {1, 2}) {
        Model model;
        model.material = steel(0.25);
        model.mesh = box(3, 2, 1.5, 3, 2, 2, order);
        auto field = [](const V3& p) {
            return V3{1e-3 * p.x + 2e-4 * p.y - 1e-4 * p.z, -3e-4 * p.x + 5e-4 * p.y + 2e-4 * p.z, 1e-4 * p.x - 2e-4 * p.y + 8e-4 * p.z};
        };
        const auto faces = mesh::boundary_faces(model.mesh);
        std::vector<char> on_boundary(model.mesh.nodes.size(), 0);
        for (const auto& f : faces) {
            for (int k = 0; k < (order == 1 ? 3 : 6); ++k) on_boundary[f.nodes[k]] = 1;
        }
        for (std::size_t n = 0; n < on_boundary.size(); ++n) {
            if (!on_boundary[n]) continue;
            const V3 u = field(model.mesh.nodes[n]);
            model.fixed.push_back({static_cast<int>(n), 0, u.x});
            model.fixed.push_back({static_cast<int>(n), 1, u.y});
            model.fixed.push_back({static_cast<int>(n), 2, u.z});
        }
        const StaticResult r = solve_static(model, Vec::Zero(model.dofs()));
        for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
            const V3 u = field(model.mesh.nodes[n]);
            CHECK(r.u[3 * n] == Approx(u.x).margin(1e-12));
            CHECK(r.u[3 * n + 1] == Approx(u.y).margin(1e-12));
            CHECK(r.u[3 * n + 2] == Approx(u.z).margin(1e-12));
        }
        // Stress is uniform: sigma_xx = lambda tr(e) + 2 mu e_xx.
        const double E = model.material.youngs_mpa(), nu = 0.25;
        const double lambda = E * nu / ((1 + nu) * (1 - 2 * nu)), mu = E / (2 * (1 + nu));
        const double tr = 1e-3 + 5e-4 + 8e-4;
        for (const Stress& s : r.stress) {
            CHECK(s[0] == Approx(lambda * tr + 2 * mu * 1e-3).epsilon(1e-8));
            CHECK(s[3] == Approx(mu * (2e-4 - 3e-4)).epsilon(1e-6));
        }
    }
}

TEST_CASE("verification: cantilever tip deflection against Timoshenko beam theory") {
    // L/h = 20, nu = 0 so that there is no anticlastic bending; clamp at x = 0, tip load P in -z.
    const double L = 20, w = 2, h = 1, P = 1.0;
    Material mat = steel(0.0);
    const double E = mat.youngs_mpa(), G = E / 2, I = w * h * h * h / 12, A = w * h;
    const double timoshenko = P * L * L * L / (3 * E * I) + P * L / (5.0 / 6.0 * G * A);

    auto tip = [&](int nx, int ny, int nz, int order) {
        Model model;
        model.material = mat;
        model.mesh = box(L, w, h, nx, ny, nz, order);
        const auto faces = mesh::boundary_faces(model.mesh);
        for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
            for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
        }
        Vec f = Vec::Zero(model.dofs());
        add_traction(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3&) { return c.x > L - 1e-9; }),
                     {0, 0, -P / A});
        const StaticResult r = solve_static(model, f);
        double sum = 0;
        int count = 0;
        for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return p.x > L - 1e-9; })) {
            sum += r.u[3 * n + 2];
            ++count;
        }
        return -sum / count;
    };

    // Quadratic tetrahedra are accurate with one element through the depth.
    CHECK(tip(10, 2, 2, 2) == Approx(timoshenko).epsilon(0.015));
    CHECK(tip(20, 2, 2, 2) == Approx(timoshenko).epsilon(0.01));

    // Linear tetrahedra lock in bending and converge from below as the mesh is refined.
    const double coarse = tip(10, 1, 1, 1), medium = tip(20, 2, 2, 1), fine = tip(40, 4, 4, 1);
    CHECK(coarse < medium);
    CHECK(medium < fine);
    CHECK(fine <= timoshenko * 1.001);
    CHECK(std::abs(fine - timoshenko) < std::abs(medium - timoshenko));
    CHECK(fine > 0.6 * timoshenko);  // linear tetrahedra are slow to converge in bending
}

TEST_CASE("verification: thick cylinder under internal pressure (Lame, plane strain)") {
    // Inner radius a, outer b; hoop stress at r = a is p (b^2 + a^2)/(b^2 - a^2), radial -p.
    const double a = 5, b = 10, p = 10.0;
    auto solve = [&](double size, int order, int ring_points) {
        mesh::Region region;
        mesh::Ring outer, inner;
        for (int i = 0; i < ring_points; ++i) {
            const double t = 2 * std::numbers::pi * i / ring_points;
            outer.push_back({b * std::cos(t), b * std::sin(t)});
            inner.push_back({a * std::cos(t), a * std::sin(t)});
        }
        region.rings = {outer, inner};
        mesh::Mesh2dOptions o;
        o.size = size;
        Model model;
        model.material = steel(0.3);
        model.mesh = mesh::extrude(mesh::triangulate(region, o), 0, size, 1, order);
        const double eps = 1e-9;
        // Plane strain: no motion in z anywhere; rigid-body motion removed at four points.
        for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) model.fixed.push_back({static_cast<int>(n), 2});
        const int east = mesh::nodes_where(model.mesh, [&](const V3& q) { return std::abs(q.x - b) < eps && std::abs(q.y) < eps && q.z < eps; })[0];
        const int west = mesh::nodes_where(model.mesh, [&](const V3& q) { return std::abs(q.x + b) < eps && std::abs(q.y) < eps && q.z < eps; })[0];
        model.fixed.push_back({east, 1});
        model.fixed.push_back({west, 1});
        model.fixed.push_back({west, 0});
        const auto faces = mesh::boundary_faces(model.mesh);
        Vec f = Vec::Zero(model.dofs());
        add_pressure(model, f, mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) {
                         return std::abs(n.z) < 0.5 && std::hypot(c.x, c.y) < 0.5 * (a + b);
                     }),
                     p);
        const StaticResult r = solve_static(model, f);
        // Hoop stress of the nodes on the bore.
        double hoop = 0;
        int count = 0;
        for (std::size_t n = 0; n < model.mesh.nodes.size(); ++n) {
            const V3& q = model.mesh.nodes[n];
            if (std::abs(std::hypot(q.x, q.y) - a) < 0.02 * a) {
                const double t = std::atan2(q.y, q.x);
                const Stress& s = r.stress[n];
                hoop += s[0] * std::sin(t) * std::sin(t) + s[1] * std::cos(t) * std::cos(t) - 2 * s[3] * std::sin(t) * std::cos(t);
                ++count;
            }
        }
        return hoop / count;
    };
    const double exact = p * (b * b + a * a) / (b * b - a * a);
    const double e_coarse = std::abs(solve(2.0, 2, 48) - exact) / exact;
    const double e_fine = std::abs(solve(1.0, 2, 96) - exact) / exact;
    CHECK(e_fine < 0.005);
    CHECK(e_coarse < 0.01);

    // Quadratic elements beat linear ones on the same mesh.
    const double e_linear = std::abs(solve(1.0, 1, 96) - exact) / exact;
    CHECK(e_fine < e_linear);
}
