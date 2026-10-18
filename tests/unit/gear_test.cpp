// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "fea/fea.hpp"
#include "gear/gear.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

using namespace ohs;
using namespace ohs::gear;
using mesh::V3;
using Catch::Approx;

namespace {

constexpr double kPi = std::numbers::pi;

double polygon_area(const mesh::Ring& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.size(); ++i) {
        const auto& p = r[i];
        const auto& q = r[(i + 1) % r.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

}  // namespace

TEST_CASE("involute gear outline has the right radii, area and tooth thickness") {
    GearSpec s;
    s.teeth = 24;
    s.module = 1.0;
    const mesh::Ring ring = outline(s);
    CHECK(polygon_area(ring) > 0);  // counter-clockwise
    CHECK(s.pitch_radius() == Approx(12.0));
    CHECK(s.base_radius() == Approx(12.0 * std::cos(20 * kPi / 180)));
    double rmin = 1e9, rmax = 0;
    for (const auto& p : ring) {
        rmin = std::min(rmin, std::hypot(p.x, p.y));
        rmax = std::max(rmax, std::hypot(p.x, p.y));
    }
    CHECK(rmin == Approx(s.root_radius()).epsilon(1e-6));
    CHECK(rmax == Approx(s.tip_radius()).epsilon(1e-6));
    CHECK(polygon_area(ring) > kPi * s.root_radius() * s.root_radius());
    CHECK(polygon_area(ring) < kPi * s.tip_radius() * s.tip_radius());
    // The tooth is half the circular pitch thick at the pitch circle.
    CHECK(2 * half_tooth_angle(s, s.pitch_radius()) * s.pitch_radius() == Approx(kPi * s.module / 2).epsilon(1e-9));
    // Tooth 0 is centred on +x: the outline is symmetric about the x axis.
    double sym = 0;
    for (const auto& p : ring) {
        double best = 1e9;
        for (const auto& q : ring) best = std::min(best, std::hypot(p.x - q.x, p.y + q.y));
        sym = std::max(sym, best);
    }
    CHECK(sym < 0.02);
    // The outline is a simple polygon the mesher accepts, covering the polygon's area.
    mesh::Mesh2dOptions o;
    o.size = 0.4;
    const mesh::Mesh2d m = mesh::triangulate({{ring}}, o);
    CHECK(mesh::area(m) == Approx(polygon_area(ring)).epsilon(1e-9));
}

TEST_CASE("gear with a bore, sectors and invalid specifications") {
    GearSpec s;
    s.bore_radius = 3;
    const auto region = wheel_region(s);
    REQUIRE(region.rings.size() == 2);
    mesh::Mesh2dOptions o;
    o.size = 0.6;
    const mesh::Mesh2d m = mesh::triangulate(region, o);
    CHECK(mesh::area(m) == Approx(polygon_area(region.rings[0]) - polygon_area(region.rings[1])).epsilon(1e-9));

    const auto sector = sector_region(s, 3, 8.0);
    mesh::Mesh2dOptions so;
    so.size = 0.3;
    const mesh::Mesh2d sm = mesh::triangulate(sector, so);
    CHECK(mesh::area(sm) > 0);
    CHECK(mesh::area(sm) < 0.25 * polygon_area(region.rings[0]));

    GearSpec bad = s;
    bad.teeth = 3;
    CHECK_THROWS_AS(outline(bad), Error);
    bad = s;
    bad.addendum = 3.0;  // pointed teeth
    CHECK_THROWS_AS(outline(bad), Error);
    bad = s;
    bad.bore_radius = 12;
    CHECK_THROWS_AS(outline(bad), Error);
    CHECK_THROWS_AS(sector_region(s, 2, 8.0), Error);
    CHECK_THROWS_AS(sector_region(s, 3, 11.9), Error);
}

TEST_CASE("Lewis form factor follows the standard table") {
    CHECK(lewis_form_factor(20) == Approx(0.322));
    CHECK(lewis_form_factor(24) == Approx(0.337));
    CHECK(lewis_form_factor(25) == Approx(0.3415));
    CHECK(lewis_form_factor(8) == Approx(0.245));
    CHECK(lewis_form_factor(5000) == Approx(0.484));
    GearSpec s;
    s.teeth = 24;
    s.module = 0.5;
    // sigma = Wt / (F m Y)
    CHECK(lewis_stress(s, 10.0, 2.0) == Approx(10.0 / (2.0 * 0.5 * 0.337)));
}

TEST_CASE("verification: FE tooth root stress against the Lewis formula") {
    GearSpec s;
    s.teeth = 24;
    s.module = 1.0;
    const double face = 2.0, wt = 100.0;
    auto size_at = [&](mesh::P2 p) { return std::min(0.7, 0.14 + 0.3 * std::hypot(p.x - s.root_radius(), p.y)); };
    mesh::Mesh2dOptions mo;
    mo.size = 0.7;
    mo.size_at = size_at;
    fea::Model model;
    model.material.youngs_gpa = 210;
    model.material.poisson = 0.3;
    model.mesh = mesh::extrude(mesh::triangulate(sector_region(s, 3, 9.5), mo), 0, face, 1, 2);
    for (int n : mesh::nodes_where(model.mesh, [&](const V3& p) { return std::hypot(p.x, p.y) < 9.5 + 1e-6; })) {
        for (int c = 0; c < 3; ++c) model.fixed.push_back({n, c});
    }
    // Wt perpendicular to the tooth axis, on the flank facing -y near the tip, pushing towards +y.
    const auto faces = mesh::boundary_faces(model.mesh);
    const auto patch = mesh::faces_where(model.mesh, faces, [&](const V3& c, const V3& n) {
        return std::abs(c.y) < 1.2 && std::hypot(c.x, c.y) > s.tip_radius() - 0.45 && c.y < 0 && n.y < -0.3;
    });
    REQUIRE_FALSE(patch.empty());
    double area = 0;
    for (const auto& f : patch) area += mesh::face_area(model.mesh, f);
    fea::Vec load = fea::Vec::Zero(model.dofs());
    fea::add_traction(model, load, patch, {0, wt / area, 0});
    const fea::StaticResult r = fea::solve_static(model, load);

    const double lewis = lewis_stress(s, wt, face);
    const double kf = r.max_principal / lewis;
    INFO("FE " << r.max_principal << " MPa, Lewis " << lewis << " MPa, ratio " << kf);
    // Lewis ignores the stress concentration of the root fillet (Kf of up to 2 for a 0.38 m fillet) and
    // uses a parabola for the tooth, so the FE peak lies above it but within a factor of about two.
    CHECK(kf > 1.0);
    CHECK(kf < 2.2);
    // The peak tensile stress sits at the root, on the loaded side.
    const V3& at = model.mesh.nodes[static_cast<std::size_t>(r.max_principal_node)];
    CHECK(std::hypot(at.x, at.y) < s.pitch_radius() - 0.8 * s.module);
    CHECK(at.y < 0);
}
