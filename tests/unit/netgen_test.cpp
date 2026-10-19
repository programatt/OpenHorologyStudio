// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <filesystem>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <gp_Ax2.hxx>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "fea/fea.hpp"
#include "fea/material.hpp"
#include "geometry/netgen_mesh.hpp"
#include "geometry/section.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

using namespace ohs;
using Catch::Approx;

namespace {

double brep_volume(const TopoDS_Shape& s) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

std::filesystem::path work(const char* tag) { return std::filesystem::temp_directory_path() / (std::string("ohs_netgen_test_") + tag); }

TopoDS_Shape box(double x, double y, double z) { return BRepPrimAPI_MakeBox(x, y, z).Shape(); }

// A plate with a counterbored hole: the through hole (r 0.8) and a wider step (r 1.6) from z = 0.6 up. The
// mid-height section (z = 1) cuts through the step, so extruding it makes the wide hole go all the way through.
TopoDS_Shape counterbored_plate() {
    TopoDS_Shape plate = box(14, 8, 2);
    const TopoDS_Shape through = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(4, 4, -1), gp_Dir(0, 0, 1)), 0.8, 4).Shape();
    const TopoDS_Shape step = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(4, 4, 0.6), gp_Dir(0, 0, 1)), 1.6, 2).Shape();
    return BRepAlgoAPI_Cut(BRepAlgoAPI_Cut(plate, through).Shape(), step).Shape();
}

}  // namespace

TEST_CASE("Netgen meshes a solid with positive tetrahedra that fill its volume", "[netgen]") {
    if (!geometry::find_netgen_python()) SKIP("Netgen (pip install netgen-mesher) is not available");
    const mesh::TetMesh lin = geometry::mesh_solid_with_netgen(box(20, 10, 2), 2.0, 1, work("box"));
    CHECK(lin.order == 1);
    CHECK(lin.elems.size() > 100);
    CHECK(mesh::volume(lin) == Approx(400.0).epsilon(1e-9));  // a box is filled exactly
    for (const auto& e : lin.elems) {
        const auto& a = lin.nodes[static_cast<std::size_t>(e[0])];
        const auto& b = lin.nodes[static_cast<std::size_t>(e[1])];
        const auto& c = lin.nodes[static_cast<std::size_t>(e[2])];
        const auto& d = lin.nodes[static_cast<std::size_t>(e[3])];
        const double det = (b.x - a.x) * ((c.y - a.y) * (d.z - a.z) - (c.z - a.z) * (d.y - a.y)) -
                           (b.y - a.y) * ((c.x - a.x) * (d.z - a.z) - (c.z - a.z) * (d.x - a.x)) +
                           (b.z - a.z) * ((c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x));
        REQUIRE(det > 0);
    }

    const mesh::TetMesh quad = geometry::mesh_solid_with_netgen(box(20, 10, 2), 2.0, 2, work("box2"));
    CHECK(quad.order == 2);
    CHECK(quad.nodes.size() > lin.nodes.size());
    CHECK(mesh::volume(quad) == Approx(400.0).epsilon(1e-9));

    CHECK_THROWS_AS(geometry::mesh_solid_with_netgen(box(1, 1, 1), 0.0, 2, work("bad")), Error);
    CHECK_THROWS_AS(geometry::mesh_solid_with_netgen(box(1, 1, 1), 0.5, 3, work("bad")), Error);
}

TEST_CASE("Netgen keeps a counterbore that the section mesher cannot see", "[netgen]") {
    if (!geometry::find_netgen_python()) SKIP("Netgen (pip install netgen-mesher) is not available");
    const TopoDS_Shape plate = counterbored_plate();
    const double want = brep_volume(plate);
    const mesh::TetMesh netgen = geometry::mesh_solid_with_netgen(plate, 0.5, 2, work("counterbore"));

    const mesh::Region region = geometry::section_region(plate, 1.0, 0.005, 0.015);
    mesh::Mesh2dOptions options;
    options.size = 0.5;
    const double extruded = mesh::volume(mesh::extrude(mesh::triangulate(region, options), 0, 2, 2, 2));

    INFO("B-rep " << want << " mm3, Netgen " << mesh::volume(netgen) << ", extruded section " << extruded);
    CHECK(mesh::volume(netgen) == Approx(want).epsilon(0.005));       // faceting of the cylinders only
    CHECK(std::abs(extruded - want) > 0.012 * want);                  // the step through the thickness is lost
    CHECK(std::abs(mesh::volume(netgen) - want) < 0.5 * std::abs(extruded - want));
}

TEST_CASE("a cantilever meshed by Netgen deflects like the same cantilever meshed by extrusion", "[netgen]") {
    if (!geometry::find_netgen_python()) SKIP("Netgen (pip install netgen-mesher) is not available");
    fea::Material steel;
    steel.youngs_gpa = 200;
    steel.poisson = 0.3;
    const double L = 20, w = 2, h = 2, P = 1.0;
    auto tip_deflection = [&](const mesh::TetMesh& m) {
        fea::Model model;
        model.material = steel;
        model.mesh = m;
        for (std::size_t n = 0; n < m.nodes.size(); ++n) {
            if (m.nodes[n].x < 1e-9) {
                for (int c = 0; c < 3; ++c) model.fixed.push_back({static_cast<int>(n), c});
            }
        }
        // The load is spread over the nodes of the free end face.
        fea::Vec load = fea::Vec::Zero(model.dofs());
        std::vector<int> end;
        for (std::size_t n = 0; n < m.nodes.size(); ++n) {
            if (m.nodes[n].x > L - 1e-9) end.push_back(static_cast<int>(n));
        }
        for (int n : end) fea::add_point_load(load, n, {0, 0, -P / static_cast<double>(end.size())});
        const auto r = fea::solve_static(model, load);
        double mean = 0;
        for (int n : end) mean += r.u(3 * n + 2);
        return -mean / static_cast<double>(end.size());
    };
    const double netgen = tip_deflection(geometry::mesh_solid_with_netgen(box(L, w, h), 1.0, 2, work("cantilever")));
    const mesh::TetMesh extruded = mesh::extrude(mesh::structured_rect(0, 0, L, w, 20, 2), 0, h, 2, 2);
    const double reference = tip_deflection(extruded);
    const double beam = P * L * L * L / (3 * steel.youngs_mpa() * (w * h * h * h / 12));  // Euler-Bernoulli, no shear
    INFO("netgen " << netgen << ", extruded " << reference << ", beam theory " << beam);
    CHECK(netgen == Approx(reference).epsilon(0.05));
    CHECK(netgen == Approx(beam).epsilon(0.10));  // a deep beam is a little more flexible than the thin-beam formula
}

TEST_CASE("the shock case runs on a Netgen mesh and agrees with the section mesher on a prismatic part", "[netgen]") {
    if (!geometry::find_netgen_python()) SKIP("Netgen (pip install netgen-mesher) is not available");
    TopoDS_Shape plate = box(12, 6, 1);
    plate = BRepAlgoAPI_Cut(plate, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(2, 3, -1), gp_Dir(0, 0, 1)), 0.8, 3).Shape()).Shape();
    cases::PartCaseInput in;
    in.shape = plate;
    in.element_size = 0.6;
    in.material = fea::find_material("brass_leaded");
    in.shock_g = 2000;
    in.in_plane_shock = true;
    const auto section = cases::run_part_case(in);
    in.mesher = cases::Mesher::Netgen;
    const auto netgen = cases::run_part_case(in);
    CHECK(netgen.clamped_holes == 1);
    CHECK(netgen.model.fixed.size() > 12);
    CHECK(netgen.mass_g == Approx(section.mass_g).epsilon(0.02));
    CHECK(netgen.max_displacement_mm == Approx(section.max_displacement_mm).epsilon(0.15));
}
