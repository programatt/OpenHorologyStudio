// SPDX-License-Identifier: Apache-2.0
// Micro and meso benchmarks for the engines. `ohs_perf` prints JSON {name: seconds} (best of N) and
// scripts/perf_compare.py compares two such files and flags slowdowns above 10 %.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <string>

#include "dynamics/timekeeping.hpp"
#include "dynamics/torque_chain.hpp"
#include "fea/analysis.hpp"
#include "fea/fea.hpp"
#include "gear/gear.hpp"
#include "lessons/lessons.hpp"
#include "manufacturing/collision.hpp"
#include "geometry/section.hpp"
#include "manufacturing/dfm.hpp"
#include "manufacturing/gear_export.hpp"
#include "manufacturing/tolerance.hpp"
#include "mesh/mesh2d.hpp"
#include "mesh/tetmesh.hpp"
#include "reports/report_builder.hpp"

using namespace ohs;
using Clock = std::chrono::steady_clock;

namespace {

double best_of(int runs, const std::function<void()>& f) {
    double best = 1e300;
    for (int i = 0; i < runs; ++i) {
        const auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double>(Clock::now() - t0).count());
    }
    return best;
}

fea::Model cantilever(int nx, int order) {
    fea::Model m;
    m.material.youngs_gpa = 210;
    m.material.poisson = 0.3;
    m.mesh = mesh::extrude(mesh::structured_rect(0, 0, 20, 2, nx, 2), 0, 1, 2, order);
    for (int n : mesh::nodes_where(m.mesh, [](const mesh::V3& p) { return p.x < 1e-9; })) {
        for (int c = 0; c < 3; ++c) m.fixed.push_back({n, c});
    }
    return m;
}

}  // namespace

int main(int argc, char** argv) {
    const int runs = argc > 1 ? std::max(1, std::atoi(argv[1])) : 3;
    std::map<std::string, double> out;

    const lessons::Context ctx = lessons::make_context(lessons::demo_movement());

    {
        gear::GearSpec s;
        s.teeth = 24;
        mesh::Mesh2dOptions o;
        o.size = 0.15;
        const auto region = gear::sector_region(s, 3, 9.5);
        out["mesh2d_gear_sector"] = best_of(runs, [&] { mesh::triangulate(region, o); });
    }
    {
        const fea::Model m = cantilever(40, 2);
        out["fea_assemble_tet10"] = best_of(runs, [&] { fea::assemble_stiffness(m); });
        fea::Vec f = fea::Vec::Zero(m.dofs());
        for (int n : mesh::nodes_where(m.mesh, [](const mesh::V3& p) { return p.x > 19.999; })) f[3 * n + 2] = -1.0;
        out["fea_static_solve_tet10"] = best_of(runs, [&] { fea::solve_static(m, f); });
        const fea::Model small = cantilever(20, 2);
        out["fea_modal_4_modes"] = best_of(std::max(1, runs - 1), [&] { fea::solve_modal(small, 4); });
    }
    out["oscillator_steady_state"] = best_of(runs, [&] { dynamics::steady_state(ctx.calibre, dynamics::escape_torque_nm(ctx.chain, ctx.calibre, 1.0)); });
    out["timekeeping_evaluate"] = best_of(std::max(1, runs - 1), [&] { dynamics::evaluate(ctx.calibre, ctx.chain); });
    {
        manufacturing::ToleranceConfig c;
        c.samples = 20000;
        out["tolerance_20k_samples"] = best_of(runs, [&] { manufacturing::run_tolerance(ctx.movement, c); });
    }
    {
        gear::GearSpec s;
        s.teeth = 60;
        s.module = 0.2;
        s.bore_radius = 1.0;
        manufacturing::PartGeometry g;
        g.name = "wheel";
        g.teeth = 60;
        g.module = 0.2;
        g.region = gear::wheel_region(s);
        g.thickness = 0.5;
        out["dfm_min_wall_60_tooth_wheel"] = best_of(runs, [&] { manufacturing::min_wall_thickness(g.region); });
        out["dfm_internal_radius_60_tooth_wheel"] = best_of(runs, [&] { manufacturing::min_internal_radius(g.region); });
    }
    {
        gear::GearSpec s;
        s.teeth = 75;
        s.module = 0.15;
        s.bore_radius = 0.1;
        TopoDS_Shape solid;
        out["gear_solid_75_teeth"] = best_of(runs, [&] { solid = manufacturing::gear_solid(s, 0.8); });
        out["section_region_75_tooth_solid"] = best_of(runs, [&] { geometry::section_region(solid, 0.4, 0.001, 0.002); });
    }
    {
        reports::ReportOptions o;
        o.structural = false;
        o.tolerance_config.samples = 5000;
        out["report_without_geometry"] = best_of(std::max(1, runs - 1), [&] { reports::build_report(ctx.movement, o); });
    }

    std::cout << "{\n";
    std::size_t i = 0;
    for (const auto& [k, v] : out) std::cout << "  \"" << k << "\": " << std::setprecision(6) << v << (++i < out.size() ? ",\n" : "\n");
    std::cout << "}\n";
    return 0;
}
