// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <gp_Ax2.hxx>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "fea/calculix.hpp"
#include "fea/fea.hpp"
#include "fea/results.hpp"
#include "gear/gear.hpp"
#include "geometry/section.hpp"
#include "manufacturing/gear_export.hpp"
#include "mesh/tetmesh.hpp"

using namespace ohs;
using mesh::V3;
using Catch::Approx;

namespace {

// 10 x 4 x 0.6 mm plate with a 0.5 mm radius hole near each end.
TopoDS_Shape plate_with_holes() {
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(10, 4, 0.6).Shape();
    for (double x : {1.5, 8.5}) {
        const TopoDS_Shape hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, 2, -0.1), gp_Dir(0, 0, 1)), 0.5, 0.8).Shape();
        plate = BRepAlgoAPI_Cut(plate, hole).Shape();
    }
    return plate;
}

fea::Material brass() {
    fea::Material m;
    m.name = "brass";
    m.youngs_gpa = 97;
    m.poisson = 0.34;
    m.density = 8440;
    m.strength_mpa = 350;
    return m;
}

}  // namespace

TEST_CASE("section of a plate with holes gives the outline and the bores") {
    const auto region = geometry::section_region(plate_with_holes(), 0.3, 0.001);
    REQUIRE(region.rings.size() == 3);
    auto area = [](const mesh::Ring& r) {
        double a = 0;
        for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
        return std::abs(0.5 * a);
    };
    CHECK(area(region.rings[0]) == Approx(40.0).epsilon(1e-9));
    CHECK(area(region.rings[1]) == Approx(std::numbers::pi * 0.25).epsilon(0.01));
    CHECK(area(region.rings[2]) == Approx(std::numbers::pi * 0.25).epsilon(0.01));

    // Thinning keeps the shape within the tolerance and drops points of the sampled circles.
    const auto thin = geometry::section_region(plate_with_holes(), 0.3, 0.0005, 0.01);
    REQUIRE(thin.rings.size() == 3);
    CHECK(thin.rings[1].size() < region.rings[1].size() * 3);
    CHECK(area(thin.rings[1]) == Approx(std::numbers::pi * 0.25).epsilon(0.02));
    CHECK_THROWS_AS(geometry::section_region(plate_with_holes(), 5.0), Error);
}

TEST_CASE("section of a part with a pocket is not mistaken for an extrusion") {
    // A 10 x 10 x 1 plate with a 4 x 4 pocket cut 0.5 deep from the top: below the pocket floor the
    // section is the full plate, above it the plate has a hole. The bottom face alone cannot say that.
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(10, 10, 1).Shape();
    const TopoDS_Shape pocket = BRepPrimAPI_MakeBox(gp_Pnt(3, 3, 0.5), 4, 4, 1).Shape();
    plate = BRepAlgoAPI_Cut(plate, pocket).Shape();
    auto area = [](const mesh::Ring& r) {
        double a = 0;
        for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
        return std::abs(0.5 * a);
    };
    const auto low = geometry::section_region(plate, 0.25);
    REQUIRE(low.rings.size() == 1);
    CHECK(area(low.rings[0]) == Approx(100.0));
    const auto high = geometry::section_region(plate, 0.75);
    REQUIRE(high.rings.size() == 2);
    CHECK(area(high.rings[0]) == Approx(100.0));
    CHECK(area(high.rings[1]) == Approx(16.0));
}

TEST_CASE("section of a generated spline wheel uses the extrusion shortcut and agrees with its outline") {
    gear::GearSpec s;
    s.teeth = 30;
    s.module = 0.4;
    s.bore_radius = 1.0;
    const TopoDS_Shape solid = manufacturing::gear_solid(s, 0.5);
    const auto region = geometry::section_region(solid, 0.25, 0.0005);
    REQUIRE(region.rings.size() == 2);
    auto area = [](const mesh::Ring& r) {
        double a = 0;
        for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
        return std::abs(0.5 * a);
    };
    gear::GearSpec fine = s;
    fine.points_per_flank = 48;
    CHECK(area(region.rings[0]) == Approx(area(gear::outline(fine))).epsilon(2e-3));
    CHECK(area(region.rings[1]) == Approx(std::numbers::pi * 1.0).epsilon(1e-3));
    // The same shape cut at a different height gives the same outline.
    const auto other = geometry::section_region(solid, 0.1, 0.0005);
    CHECK(area(other.rings[0]) == Approx(area(region.rings[0])).epsilon(1e-9));
}

TEST_CASE("a part under shock: mass, linearity in g, clamped holes, modes") {
    cases::PartCaseInput in;
    in.shape = plate_with_holes();
    in.element_size = 0.5;
    in.material = brass();
    in.shock_g = 1000;
    in.modes = 3;
    const cases::PartCaseResult r = cases::run_part_case(in);

    const double volume = 10 * 4 * 0.6 - 2 * std::numbers::pi * 0.25 * 0.6;
    CHECK(r.mass_g == Approx(volume * 8.44e-3).epsilon(0.01));  // mm3 * g/mm3
    CHECK(r.max_displacement_mm > 0);
    CHECK(r.max_von_mises_mpa > 0);
    CHECK(r.safety_factor == Approx(350.0 / r.max_von_mises_mpa));
    REQUIRE(r.frequencies_hz.size() == 3);
    CHECK(r.frequencies_hz[0] > 100);
    CHECK(r.frequencies_hz[0] < r.frequencies_hz[1]);

    // Linear elasticity: ten times the shock, ten times the displacement and stress.
    in.shock_g = 10000;
    in.modes = 0;
    const cases::PartCaseResult r10 = cases::run_part_case(in);
    CHECK(r10.max_displacement_mm == Approx(10 * r.max_displacement_mm).epsilon(1e-6));
    CHECK(r10.max_von_mises_mpa == Approx(10 * r.max_von_mises_mpa).epsilon(1e-6));

    // Clamped holes: the bore nodes do not move; the middle of the plate does.
    for (const auto& c : r.model.fixed) CHECK(r.solution.u[3 * c.node + c.comp] == 0.0);
    // A part without holes cannot be supported.
    in.shape = BRepPrimAPI_MakeBox(10, 4, 0.6).Shape();
    CHECK_THROWS_AS(cases::run_part_case(in), Error);
    // The size guard refuses meshes that would not fit in memory.
    in.shape = plate_with_holes();
    in.max_dofs = 1000;
    CHECK_THROWS_AS(cases::run_part_case(in), Error);
}

TEST_CASE("a window or slot is not a support: only round pivot holes are clamped") {
    cases::PartCaseInput in;
    in.element_size = 0.4;
    in.material = brass();
    in.shock_g = 1000;

    in.shape = plate_with_holes();
    const cases::PartCaseResult solid = cases::run_part_case(in);
    CHECK(solid.clamped_holes == 2);
    CHECK(solid.free_openings == 0);

    // A tiny window mid-span changes almost nothing: the old behaviour clamped its edge and cut the stress by a large factor.
    TopoDS_Shape with_window = plate_with_holes();
    with_window = BRepAlgoAPI_Cut(with_window, BRepPrimAPI_MakeBox(gp_Pnt(4.7, 1.85, -0.1), 0.6, 0.3, 0.8).Shape()).Shape();
    in.shape = with_window;
    const cases::PartCaseResult small = cases::run_part_case(in);
    CHECK(small.clamped_holes == 2);
    CHECK(small.free_openings == 1);
    CHECK(small.model.fixed.size() == solid.model.fixed.size());
    CHECK(small.max_von_mises_mpa == Approx(solid.max_von_mises_mpa).epsilon(0.25));
    CHECK(small.max_displacement_mm == Approx(solid.max_displacement_mm).epsilon(0.1));

    // A round hole wider than the limit (2 mm here) is an opening, not a pivot seat; the 1 mm pivot holes stay clamped.
    TopoDS_Shape big_hole = plate_with_holes();
    big_hole = BRepAlgoAPI_Cut(big_hole, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(5, 2, -0.1), gp_Dir(0, 0, 1)), 1.2, 0.8).Shape()).Shape();
    in.shape = big_hole;
    in.max_clamp_diameter = 2.0;
    const cases::PartCaseResult wide = cases::run_part_case(in);
    CHECK(wide.clamped_holes == 2);
    CHECK(wide.free_openings == 1);

    // Nothing but a window: nothing to hold the part by.
    in.max_clamp_diameter = 4.0;
    in.shape = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(10, 4, 0.6).Shape(), BRepPrimAPI_MakeBox(gp_Pnt(3, 1, -0.1), 4, 2, 0.8).Shape()).Shape();
    CHECK_THROWS_AS(cases::run_part_case(in), Error);
}

TEST_CASE("tooth bending case reports the Lewis stress next to the FE peak") {
    cases::ToothCaseInput in;
    in.spec.teeth = 20;
    in.spec.module = 0.3;
    in.face_width = 0.5;
    in.tangential_load_n = 0.5;
    in.material = brass();
    const cases::ToothCaseResult r = cases::run_tooth_case(in);
    CHECK(r.lewis_mpa == Approx(0.5 / (0.5 * 0.3 * 0.322)));
    CHECK(r.concentration > 1.0);
    CHECK(r.concentration < 2.4);
    CHECK(r.safety_factor == Approx(350.0 / r.fe_max_principal_mpa));
    CHECK(r.tip_deflection_mm > 0);
    // Stress is linear in the load.
    in.tangential_load_n = 1.0;
    CHECK(cases::run_tooth_case(in).fe_max_principal_mpa == Approx(2 * r.fe_max_principal_mpa).epsilon(1e-6));
    // A wheel too small for a sector around its bore is reported.
    in.spec.bore_radius = 2.8;
    CHECK_THROWS_AS(cases::run_tooth_case(in), Error);
}

TEST_CASE("the fast physics verification set passes") {
    const auto checks = cases::run_physics_checks(false);
    REQUIRE(checks.size() >= 4);
    for (const auto& c : checks) {
        INFO(c.name << ": expected " << c.expected << " got " << c.actual);
        CHECK(c.passed);
    }
}

TEST_CASE("VTU output describes the mesh and its fields") {
    fea::Model model;
    model.material.youngs_gpa = 210;
    model.mesh = mesh::extrude(mesh::structured_rect(0, 0, 1, 1, 1, 1), 0, 1, 1, 2);
    fea::StaticResult r;
    r.u = fea::Vec::Zero(model.dofs());
    r.stress.assign(model.mesh.nodes.size(), fea::Stress{});
    r.von_mises.assign(model.mesh.nodes.size(), 1.5);
    const std::string vtu = fea::to_vtu(model.mesh, fea::static_fields(r));
    CHECK_THAT(vtu, Catch::Matchers::ContainsSubstring("NumberOfPoints=\"" + std::to_string(model.mesh.nodes.size()) + "\""));
    CHECK_THAT(vtu, Catch::Matchers::ContainsSubstring("NumberOfCells=\"" + std::to_string(model.mesh.elems.size()) + "\""));
    CHECK_THAT(vtu, Catch::Matchers::ContainsSubstring("Name=\"von_mises\""));
    CHECK_THAT(vtu, Catch::Matchers::ContainsSubstring("24 24"));  // quadratic tetrahedra
    fea::NodeField bad{"x", {1.0}, 1};
    CHECK_THROWS_AS(fea::to_vtu(model.mesh, {bad}), Error);
    const auto lin = mesh::extrude(mesh::structured_rect(0, 0, 1, 1, 1, 1), 0, 1, 1, 1);
    CHECK_THAT(fea::to_vtu(lin, {}), Catch::Matchers::ContainsSubstring("10 10"));
}

TEST_CASE("CalculiX input deck and result parser") {
    fea::Model model;
    model.material = brass();
    model.mesh = mesh::extrude(mesh::structured_rect(0, 0, 2, 1, 2, 1), 0, 1, 1, 2);
    for (int n : mesh::nodes_where(model.mesh, [](const V3& p) { return p.x < 1e-9; })) {
        for (int comp = 0; comp < 3; ++comp) model.fixed.push_back({n, comp});  // fully clamped, or the solvers split the rigid-body modes differently
    }
    fea::Vec load = fea::Vec::Zero(model.dofs());
    fea::add_point_load(load, 5, {0, 0, -3.5});
    const std::string deck = fea::write_calculix_input(model, load);
    CHECK_THAT(deck, Catch::Matchers::ContainsSubstring("*ELEMENT, TYPE=C3D10"));
    CHECK_THAT(deck, Catch::Matchers::ContainsSubstring("*ELASTIC\n97000, 0.34"));
    CHECK_THAT(deck, Catch::Matchers::ContainsSubstring("*CLOAD\n6, 3, -3.5"));
    CHECK_THAT(deck, Catch::Matchers::ContainsSubstring("*BOUNDARY"));
    std::size_t nodes = 0;
    for (std::size_t pos = 0; (pos = deck.find('\n', pos)) != std::string::npos; ++pos) ++nodes;
    CHECK(nodes > model.mesh.nodes.size() + model.mesh.elems.size());

    fea::Model two = model;
    two.materials = {brass()};
    CHECK_THROWS_AS(fea::write_calculix_input(two, load), Error);

    // A result file with touching negative numbers, as CalculiX prints them.
    const std::string frd =
        "    1C\n -4  DISP        4    1\n -5  D1          1    2    1    0\n"
        " -1         1 0.00000E+00-1.25000E-03 2.50000E-04\n"
        " -1         2-3.00000E-01 4.00000E-02-5.00000E+00\n"
        " -3\n";
    const fea::Vec u = fea::parse_calculix_displacements(frd, 2);
    CHECK(u[1] == Approx(-1.25e-3));
    CHECK(u[2] == Approx(2.5e-4));
    CHECK(u[3] == Approx(-0.3));
    CHECK(u[5] == Approx(-5.0));
    CHECK_THROWS_AS(fea::parse_calculix_displacements(" -4  STRESS      6    1\n -1         1 0.0E+00 0.0E+00 0.0E+00\n -3\n", 2), Error);
    CHECK_THROWS_AS(fea::parse_calculix_displacements(frd, 1), Error);

    // Without ccx on the machine the cross-check says so instead of failing.
    const fea::StaticResult r = fea::solve_static(model, load);
    const auto cc = fea::run_calculix_crosscheck(model, load, r, std::filesystem::temp_directory_path() / "ohs_ccx_test");
    if (!fea::find_calculix()) {
        CHECK_FALSE(cc.available);
        CHECK_FALSE(cc.note.empty());
    } else if (cc.available) {
        CHECK(cc.max_displacement_diff < 1e-3);
    }
}
