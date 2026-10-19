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

namespace {

bool inside(const mesh::Ring& ring, double x, double y) {
    bool in = false;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        if ((ring[i].y > y) != (ring[j].y > y) && x < (ring[j].x - ring[i].x) * (y - ring[i].y) / (ring[j].y - ring[i].y) + ring[i].x) in = !in;
    }
    return in;
}

mesh::Ring moved(const mesh::Ring& ring, double turn, double dx) {
    mesh::Ring out;
    for (const auto& p : ring) out.push_back({dx + p.x * std::cos(turn) - p.y * std::sin(turn), p.x * std::sin(turn) + p.y * std::cos(turn)});
    return out;
}

// Area (mm^2) covered by both wheels, by sampling the strip between the two pitch circles' contact zone.
double overlap_area(const mesh::Ring& a, const mesh::Ring& b, double x0, double x1, double y0, double y1, int n) {
    double count = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const double x = x0 + (x1 - x0) * (i + 0.5) / n, y = y0 + (y1 - y0) * (j + 0.5) / n;
            if (inside(a, x, y) && inside(b, x, y)) ++count;
        }
    }
    return count * (x1 - x0) * (y1 - y0) / (static_cast<double>(n) * n);
}

GearSpec cycloidal(int teeth, double module = 0.2) {
    GearSpec s;
    s.teeth = teeth;
    s.module = module;
    s.profile = Profile::Cycloidal;
    s.root_fillet = 0.2;
    return s;
}

}  // namespace

TEST_CASE("cycloidal tooth: thickness at the pitch circle, monotone flank, radial dedendum at 12 teeth") {
    const GearSpec s = cycloidal(30, 1.0);
    const double R = s.pitch_radius();
    CHECK(half_tooth_angle(s, R) == Approx(kPi / 60).epsilon(1e-9));  // tooth = gap = half the pitch
    double prev = half_tooth_angle(s, s.root_radius());
    for (int i = 1; i <= 100; ++i) {
        const double r = s.root_radius() + (s.tip_radius() - s.root_radius()) * i / 100;
        const double h = half_tooth_angle(s, r);
        CHECK(h < prev);  // the tooth narrows from root to tip
        prev = h;
    }
    CHECK(prev > 0);

    // A rolling circle of half the pitch radius makes the dedendum flank a diameter: constant angle.
    GearSpec twelve = cycloidal(12, 1.0);
    for (double r : {twelve.root_radius() + 1e-3, 4.0, 5.5})  CHECK(half_tooth_angle(twelve, r) == Approx(kPi / 24).epsilon(1e-6));

    const mesh::Ring ring = outline(s);
    CHECK(polygon_area(ring) > 0);
    double rmin = 1e9, rmax = 0;
    for (const auto& p : ring) {
        rmin = std::min(rmin, std::hypot(p.x, p.y));
        rmax = std::max(rmax, std::hypot(p.x, p.y));
    }
    CHECK(rmin == Approx(s.root_radius()).epsilon(1e-6));
    CHECK(rmax == Approx(s.tip_radius()).epsilon(1e-6));
}

TEST_CASE("cycloidal wheel and pinion with one rolling circle mesh without overlap at every phase") {
    const GearSpec wheel = cycloidal(30), pinion = cycloidal(10);
    const double cd = wheel.pitch_radius() + pinion.pitch_radius();
    const mesh::Ring a = outline(wheel);
    const mesh::Ring b0 = outline(pinion);
    // Wheel turns by t, the pinion by the ratio the other way; the pinion starts with a gap on the line of centres.
    const double pitch_a = 2 * kPi / wheel.teeth, pitch_b = 2 * kPi / pinion.teeth;
    const double tooth_area = polygon_area(b0) / pinion.teeth;
    double worst = 0.0;
    for (int k = 0; k < 24; ++k) {
        const double t = pitch_a * k / 24.0;
        const mesh::Ring wa = moved(a, t, 0.0);
        const mesh::Ring pb = moved(b0, kPi + pitch_b / 2 - t * wheel.teeth / pinion.teeth, cd);
        worst = std::max(worst, overlap_area(wa, pb, cd - pinion.tip_radius() - 0.1, wheel.tip_radius() + 0.1, -0.7 * cd / 3, 0.7 * cd / 3, 300));
    }
    CHECK(worst < 0.03 * tooth_area);

    // A pinion turned half a pitch off (tooth into tooth) must overlap, or the test would pass for nothing.
    const mesh::Ring bad = moved(b0, kPi, cd);
    const mesh::Ring wa = moved(a, 0.0, 0.0);
    CHECK(overlap_area(wa, bad, cd - pinion.tip_radius() - 0.1, wheel.tip_radius() + 0.1, -0.7 * cd / 3, 0.7 * cd / 3, 300) > 0.2 * tooth_area);
}

TEST_CASE("cycloidal specifications that cannot be made are refused") {
    GearSpec s = cycloidal(30, 1.0);
    s.rolling_radius = 20.0;  // bigger than the pitch circle
    CHECK_THROWS_AS(outline(s), Error);
    s = cycloidal(30, 1.0);
    s.addendum = 7.0;  // beyond the epicycloid's cusp at 2 rho
    CHECK_THROWS_AS(outline(s), Error);
    s = cycloidal(30, 1.0);
    s.rolling_radius = 14.5;  // the hypocycloid cusp (at 14 mm) lies above the 13.75 mm root
    CHECK_THROWS_AS(outline(s), Error);
}
