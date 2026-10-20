// SPDX-License-Identifier: Apache-2.0
#include "cases/cases.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <random>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include "core/error.hpp"
#include "geometry/netgen_mesh.hpp"
#include "geometry/section.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"

namespace ohs::cases {

using mesh::V3;

ToothCaseResult run_tooth_case(const ToothCaseInput& in) {
    const gear::GearSpec& s = in.spec;
    const double m = s.module;
    const double rim = s.root_radius() - 1.5 * m;
    if (rim <= s.bore_radius) throw Error("the wheel is too small for a tooth sector: the rim would reach the bore");

    const double fine = in.size_factor * m;
    mesh::Mesh2dOptions mo;
    mo.size = 0.7 * m;
    mo.size_at = [&](mesh::P2 p) { return std::min(0.7 * m, fine + 0.3 * std::hypot(p.x - s.root_radius(), p.y)); };

    ToothCaseResult r;
    r.model.material = in.material;
    r.model.mesh = mesh::extrude(mesh::triangulate(gear::sector_region(s, in.teeth_in_sector, rim), mo), 0, in.face_width, 1, 2);
    for (int n : mesh::nodes_where(r.model.mesh, [&](const V3& p) { return std::hypot(p.x, p.y) < rim + 1e-6; })) {
        for (int c = 0; c < 3; ++c) r.model.fixed.push_back({n, c});
    }

    // The load acts on the flank that faces -y near the tip, pushing the tooth towards +y.
    const auto faces = mesh::boundary_faces(r.model.mesh);
    const auto patch = mesh::faces_where(r.model.mesh, faces, [&](const V3& c, const V3& n) {
        return std::abs(c.y) < 1.2 * m && std::hypot(c.x, c.y) > s.tip_radius() - 0.45 * m && c.y < 0 && n.y < -0.3;
    });
    if (patch.empty()) throw Error("tooth case: no flank faces found near the tip to load");
    double area = 0;
    for (const auto& f : patch) area += mesh::face_area(r.model.mesh, f);
    fea::Vec load = fea::Vec::Zero(r.model.dofs());
    fea::add_traction(r.model, load, patch, {0, in.tangential_load_n / area, 0});

    r.nodes = r.model.mesh.nodes.size();
    r.dofs = r.model.dofs();
    // Checked before the solve: a mesh this size (a part whose fitted module is far too small) crashed the
    // factorisation instead of failing cleanly.
    if (r.dofs > 250000) throw Error("the tooth mesh has " + std::to_string(r.dofs) + " dofs: use a larger --size-factor");
    r.solution = fea::solve_static(r.model, load);
    r.lewis_mpa = gear::lewis_stress(s, in.tangential_load_n, in.face_width);
    r.fe_max_principal_mpa = r.solution.max_principal;
    r.fe_max_von_mises_mpa = r.solution.max_von_mises;
    r.concentration = r.lewis_mpa > 0 ? r.fe_max_principal_mpa / r.lewis_mpa : 0.0;
    r.safety_factor = in.material.strength_mpa > 0 && r.fe_max_principal_mpa > 0 ? in.material.strength_mpa / r.fe_max_principal_mpa : 0.0;
    double tip = 0;
    int count = 0;
    for (const auto& f : patch) {
        for (int k = 0; k < 3; ++k) {
            tip += r.solution.u[3 * f.nodes[k] + 1];
            ++count;
        }
    }
    r.tip_deflection_mm = count ? tip / count : 0.0;
    return r;
}

PartCaseResult run_part_case(const PartCaseInput& in) {
    Bnd_Box box;
    BRepBndLib::AddOptimal(in.shape, box, false, false);
    double b[6];
    box.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
    const double z0 = b[2], z1 = b[5];
    if (z1 - z0 <= 0) throw Error("the part has no thickness");

    const mesh::Region region = geometry::section_region(in.shape, 0.5 * (z0 + z1), 0.01 * in.element_size, 0.03 * in.element_size);
    PartCaseResult r;
    r.model.material = in.material;
    if (in.mesher == Mesher::Netgen) {
        r.model.mesh = geometry::mesh_solid_with_netgen(
            in.shape, in.element_size, 2, std::filesystem::temp_directory_path() / ("ohs_netgen_" + std::to_string(std::random_device{}())));
    } else {
        mesh::Mesh2dOptions mo;
        mo.size = in.element_size;
        r.model.mesh = mesh::extrude(mesh::triangulate(region, mo), z0, z1, std::max(in.layers, 1), 2);
    }
    r.nodes = r.model.mesh.nodes.size();
    r.dofs = r.model.dofs();
    if (r.dofs > in.max_dofs) {
        throw Error("the mesh has " + std::to_string(r.dofs) + " dofs, over the limit of " + std::to_string(in.max_dofs) + ": use a larger --size");
    }

    // Clamp the pivot and screw holes: nodes of the 2D mesh that lie on a small round inner ring.
    // A window or slot is not a support; clamping it would stiffen the part it was cut to lighten.
    std::vector<std::size_t> clamp_rings;
    for (std::size_t k = 1; k < region.rings.size(); ++k) {
        const auto& ring = region.rings[k];
        double area = 0, perimeter = 0, lo_x = 1e30, hi_x = -1e30, lo_y = 1e30, hi_y = -1e30;
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const auto& a = ring[i];
            const auto& c = ring[(i + 1) % ring.size()];
            area += 0.5 * (a.x * c.y - c.x * a.y);
            perimeter += std::hypot(c.x - a.x, c.y - a.y);
            lo_x = std::min(lo_x, a.x); hi_x = std::max(hi_x, a.x);
            lo_y = std::min(lo_y, a.y); hi_y = std::max(hi_y, a.y);
        }
        const double circularity = perimeter > 0 ? 4 * std::numbers::pi * std::abs(area) / (perimeter * perimeter) : 0.0;
        const double width = std::max(hi_x - lo_x, hi_y - lo_y);
        if (circularity >= in.min_clamp_circularity && width <= in.max_clamp_diameter) clamp_rings.push_back(k);
    }
    r.clamped_holes = static_cast<int>(clamp_rings.size());
    r.free_openings = static_cast<int>(region.rings.size()) - 1 - r.clamped_holes;
    // Netgen's nodes lie on the true circle of a hole, the section's ring is a polygon inscribed in it.
    const double tol = (in.mesher == Mesher::Netgen ? 0.02 : 1e-4) * in.element_size;
    auto on_hole = [&](const mesh::P2 p) {
        for (std::size_t k : clamp_rings) {
            const auto& ring = region.rings[k];
            for (std::size_t i = 0; i < ring.size(); ++i) {
                const auto& a = ring[i];
                const auto& c = ring[(i + 1) % ring.size()];
                const double dx = c.x - a.x, dy = c.y - a.y;
                const double len2 = dx * dx + dy * dy;
                const double t = len2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
                if (std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy)) < tol) return true;
            }
        }
        return false;
    };
    for (std::size_t n = 0; n < r.model.mesh.nodes.size(); ++n) {
        const V3& p = r.model.mesh.nodes[n];
        if (on_hole({p.x, p.y})) {
            for (int c = 0; c < 3; ++c) r.model.fixed.push_back({static_cast<int>(n), c});
        }
    }
    if (r.model.fixed.empty()) throw Error("the part has no round pivot or screw hole (up to " + std::to_string(in.max_clamp_diameter) + " mm across) to clamp; other openings are left free");

    fea::Vec load = fea::Vec::Zero(r.model.dofs());
    const double a = in.shock_g * 9810.0;
    fea::add_gravity(r.model, load, in.in_plane_shock ? V3{-a, 0, 0} : V3{0, 0, -a});
    fea::SolveOptions solve;
    solve.solver = in.solver;
    r.solution = fea::solve_static(r.model, load, solve);
    r.mass_g = mesh::volume(r.model.mesh) * in.material.density_t_mm3() * 1e6;  // tonne -> g
    for (std::size_t n = 0; n < r.model.mesh.nodes.size(); ++n) {
        r.max_displacement_mm = std::max(r.max_displacement_mm, r.solution.u.segment<3>(3 * n).norm());
    }
    r.max_von_mises_mpa = r.solution.max_von_mises;
    r.safety_factor = in.material.strength_mpa > 0 && r.max_von_mises_mpa > 0 ? in.material.strength_mpa / r.max_von_mises_mpa : 0.0;
    if (in.modes > 0) r.frequencies_hz = fea::solve_modal(r.model, in.modes).frequencies_hz;
    return r;
}

}  // namespace ohs::cases
