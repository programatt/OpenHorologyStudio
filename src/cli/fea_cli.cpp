// SPDX-License-Identifier: Apache-2.0
#include "cli/fea_cli.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>

#include <nlohmann/json.hpp>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "dynamics/calibre.hpp"
#include "dynamics/torque_chain.hpp"
#include "fea/analysis.hpp"
#include "fea/calculix.hpp"
#include "fea/mfem_adapter.hpp"
#include "fea/results.hpp"
#include "fea/vtk_adapter.hpp"
#include "geometry/features.hpp"
#include "geometry/cad_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"

using nlohmann::json;
namespace fs = std::filesystem;

namespace ohs::cli {

namespace {

constexpr int kExitOk = 0;
constexpr int kExitCheckFailed = 1;

struct FeaOptions {
    std::string project, part, case_name = "tooth-bending", material, materials_file, step_override, up, vtu, vtu_compressed, hdf5, mesher = "section";
    std::string hairspring = "nife_hairspring", balance = "brass_leaded";
    double load_n = 0, wind = 1.0, size = 0.2, size_factor = 0.14, shock_g = 5000, beat_rate = 0;
    int modes = 0;
    bool in_plane = false, full = false, crosscheck = false, mfem = false;
};

geometry::Axis axis_of(const std::string& up);
geometry::Axis axis_of(const std::string& up, const model::Movement& m) {
    return axis_of(up.empty() ? m.dial_axis : up);
}

geometry::Axis axis_of(const std::string& up) {
    if (up == "x") return geometry::Axis::X;
    if (up == "y") return geometry::Axis::Y;
    if (up == "z") return geometry::Axis::Z;
    throw Error("--up must be x, y or z");
}

std::vector<fea::Material> project_materials(const FeaOptions& o) {
    return o.materials_file.empty() ? std::vector<fea::Material>{} : fea::load_materials(o.materials_file);
}

int cmd_materials(const FeaOptions& o, bool as_json) {
    auto all = project_materials(o);
    for (const fea::Material& m : fea::builtin_materials()) all.push_back(m);
    if (as_json) {
        json j = json::array();
        for (const auto& m : all) {
            j.push_back({{"name", m.name}, {"description", m.description}, {"youngs_gpa", m.youngs_gpa}, {"poisson", m.poisson},
                         {"density", m.density}, {"cte", m.cte}, {"strength_mpa", m.strength_mpa},
                         {"thermoelastic", m.thermoelastic}, {"source", m.source}});
        }
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    std::cout << std::left << std::setw(18) << "name" << std::right << std::setw(9) << "E (GPa)" << std::setw(7) << "nu"
              << std::setw(10) << "rho" << std::setw(11) << "CTE 1/K" << std::setw(10) << "strength" << "  description\n";
    for (const auto& m : all) {
        std::cout << std::left << std::setw(18) << m.name << std::right << std::setw(9) << m.youngs_gpa << std::setw(7) << m.poisson
                  << std::setw(10) << m.density << std::setw(11) << m.cte << std::setw(10) << m.strength_mpa << "  " << m.description << '\n';
    }
    return kExitOk;
}

int cmd_thermal(const FeaOptions& o, bool as_json) {
    const auto extra = project_materials(o);
    const fea::Material& spring = fea::find_material(o.hairspring, extra);
    const fea::Material& balance = fea::find_material(o.balance, extra);
    const double c = fea::temperature_rate_coefficient(spring, balance);
    if (as_json) {
        std::cout << json{{"hairspring", spring.name}, {"balance", balance.name}, {"rate_s_per_day_per_k", c},
                          {"over_range_c", {std::max(spring.temp_range_c.first, balance.temp_range_c.first), std::min(spring.temp_range_c.second, balance.temp_range_c.second)}}}.dump(2)
                  << '\n';
    } else {
        std::cout << "hairspring " << spring.name << " (e = " << spring.thermoelastic << " /K, alpha = " << spring.cte << " /K), balance "
                  << balance.name << " (alpha = " << balance.cte << " /K)\nrate change " << std::showpos << std::fixed << std::setprecision(2) << c
                  << " s/day per kelvin (rate = 86400 (e/2 + 3 alpha_spring/2 - alpha_balance))\n";
    }
    return kExitOk;
}

struct Rig {
    model::Movement movement;
    kinematics::TrainAnalysis train;
    int component{-1};
};

Rig load_rig_part(const FeaOptions& o) {
    Rig r;
    r.movement = model::load_rig(o.project);
    const double vph = o.beat_rate > 0 ? o.beat_rate : r.movement.beat_rate_vph;
    r.train = kinematics::analyze_train(r.movement, vph);
    r.component = r.movement.find_component(o.part);
    if (r.component < 0) throw Error("no part called '" + o.part + "' in the project");
    return r;
}

int cmd_fea(const FeaOptions& o, bool as_json) {
    const Rig rig = load_rig_part(o);
    const auto& comp = rig.movement.components[rig.component];
    const auto extra = project_materials(o);
    json j;
    j["part"] = o.part;
    j["case"] = o.case_name;

    if (o.case_name == "tooth-bending") {
        if (comp.features.teeth < 6) throw Error("'" + o.part + "' has no detected teeth");
        cases::ToothCaseInput in;
        in.spec.teeth = comp.features.teeth;
        in.spec.module = comp.gear && comp.gear->module > 0 ? comp.gear->module : model::module_from_tip(comp.features.outer_radius, comp.features.teeth);
        if (comp.gear && comp.gear->pressure_angle_deg > 5) in.spec.pressure_angle_deg = comp.gear->pressure_angle_deg;
        if (comp.gear && comp.gear->profile == "cycloidal") {
            in.spec.profile = gear::Profile::Cycloidal;
            in.spec.rolling_radius = comp.gear->rolling_radius;
        }
        in.spec.bore_radius = comp.features.bore_radius;
        in.face_width = comp.features.thickness();
        in.size_factor = o.size_factor;
        in.material = fea::find_material(o.material.empty() ? (comp.role == model::Role::Pinion ? "steel_hardened" : "brass_leaded") : o.material, extra);

        double force = o.load_n;
        std::string source = "given with --load-n";
        if (force <= 0) {
            dynamics::Calibre cal = dynamics::default_calibre(rig.movement.beat_rate_vph > 0 ? rig.movement.beat_rate_vph : 25200);
            const auto chain = dynamics::build_chain(rig.movement, rig.train, cal);
            const std::string mobile = rig.movement.mobiles[static_cast<std::size_t>(rig.movement.mobile_of(rig.component))].name;
            for (const auto& row : dynamics::torque_table(chain, cal, o.wind)) {
                if (row.mobile == mobile) force = row.torque_nmm / in.spec.pitch_radius();
            }
            if (force <= 0) throw Error("'" + o.part + "' is not on the torque path from the barrel; give --load-n");
            source = "torque of mobile " + mobile + " at wind " + std::to_string(o.wind) + " over the pitch radius";
        }
        in.tangential_load_n = force;
        const cases::ToothCaseResult r = cases::run_tooth_case(in);
        j["module"] = in.spec.module;
        j["teeth"] = in.spec.teeth;
        j["face_width_mm"] = in.face_width;
        j["material"] = in.material.name;
        j["load_n"] = force;
        j["load_source"] = source;
        j["lewis_mpa"] = r.lewis_mpa;
        j["fe_max_principal_mpa"] = r.fe_max_principal_mpa;
        j["fe_max_von_mises_mpa"] = r.fe_max_von_mises_mpa;
        j["concentration_over_lewis"] = r.concentration;
        j["safety_factor"] = r.safety_factor;
        j["tip_deflection_mm"] = r.tip_deflection_mm;
        j["dofs"] = r.dofs;
        if (!o.vtu.empty()) fea::write_vtu(o.vtu, r.model.mesh, fea::static_fields(r.solution));
        if (!o.hdf5.empty()) fea::write_hdf5(o.hdf5, r.model.mesh, fea::static_fields(r.solution));
        if (!o.vtu_compressed.empty()) fea::write_vtu_compressed(o.vtu_compressed, r.model.mesh, fea::static_fields(r.solution));
        if (o.crosscheck) {
            j["crosscheck"] = {{"note", "the tooth case uses traction loads; use the shock case for the CalculiX cross-check"}};
        }
        if (!as_json) {
            std::cout << "tooth bending of " << o.part << ": " << in.spec.teeth << " teeth, module " << in.spec.module << ", face " << in.face_width
                      << " mm, " << in.material.name << "\nload " << force << " N at the tip (" << source << ")\n"
                      << std::defaultfloat << std::setprecision(4) << "Lewis hand calculation " << r.lewis_mpa << " MPa\nFE root stress " << r.fe_max_principal_mpa
                      << " MPa (x" << std::setprecision(2) << r.concentration << " the Lewis value: the fillet concentrates the stress)\n";
            if (r.safety_factor > 0) std::cout << "safety factor against " << in.material.strength_mpa << " MPa: " << r.safety_factor << '\n';
        }
    } else if (o.case_name == "shock") {
        fs::path step = o.step_override;
        if (step.empty()) step = fs::path(o.project).parent_path() / rig.movement.source_geometry;
        const auto parts = geometry::import_cad(step);
        TopoDS_Shape shape;
        bool found = false;
        for (const auto& p : parts) {
            if (p.name == o.part) {
                shape = geometry::orient_dial_axis_to_z(p.shape, axis_of(o.up, rig.movement));
                found = true;
            }
        }
        if (!found) throw Error("the STEP file has no part called '" + o.part + "'");
        cases::PartCaseInput in;
        in.shape = shape;
        in.element_size = o.size;
        if (o.mesher == "netgen") in.mesher = cases::Mesher::Netgen;
        else if (o.mesher != "section") throw Error("--mesher must be section or netgen");
        in.shock_g = o.shock_g;
        in.in_plane_shock = o.in_plane;
        in.modes = o.modes;
        in.material = fea::find_material(o.material.empty() ? "brass_leaded" : o.material, extra);
        const cases::PartCaseResult r = cases::run_part_case(in);
        j["material"] = in.material.name;
        j["mesher"] = o.mesher;
        j["shock_g"] = o.shock_g;
        j["mass_g"] = r.mass_g;
        j["max_displacement_mm"] = r.max_displacement_mm;
        j["max_von_mises_mpa"] = r.max_von_mises_mpa;
        j["safety_factor"] = r.safety_factor;
        j["frequencies_hz"] = r.frequencies_hz;
        j["dofs"] = r.dofs;
        j["clamped_holes"] = r.clamped_holes;
        j["free_openings"] = r.free_openings;
        if (!o.vtu.empty()) fea::write_vtu(o.vtu, r.model.mesh, fea::static_fields(r.solution));
        if (!o.hdf5.empty()) fea::write_hdf5(o.hdf5, r.model.mesh, fea::static_fields(r.solution));
        if (!o.vtu_compressed.empty()) fea::write_vtu_compressed(o.vtu_compressed, r.model.mesh, fea::static_fields(r.solution));
        if (o.crosscheck || o.mfem) {
            fea::Vec load = fea::Vec::Zero(r.model.dofs());
            fea::add_gravity(r.model, load, in.in_plane_shock ? mesh::V3{-o.shock_g * 9810.0, 0, 0} : mesh::V3{0, 0, -o.shock_g * 9810.0});
            if (o.crosscheck) {
                const auto cc = fea::run_calculix_crosscheck(r.model, load, r.solution, fs::temp_directory_path() / "ohs_ccx");
                j["crosscheck"] = {{"available", cc.available}, {"max_displacement_diff", cc.max_displacement_diff}, {"note", cc.note}};
            }
            if (o.mfem) {
                const auto mc = fea::run_mfem_crosscheck(r.model, load, r.solution);
                j["crosscheck_mfem"] = {{"available", mc.available}, {"max_displacement_diff", mc.max_displacement_diff}, {"note", mc.note}};
            }
        }
        if (!as_json) {
            std::cout << "shock of " << o.shock_g << " g on " << o.part << " (" << in.material.name << ", " << r.mass_g << " g, " << r.dofs << " dofs; " << r.clamped_holes << " round hole(s) clamped, " << r.free_openings << " opening(s) free)\n"
                      << std::setprecision(4) << "max displacement " << r.max_displacement_mm << " mm, peak von Mises " << r.max_von_mises_mpa << " MPa";
            if (r.safety_factor > 0) std::cout << ", safety factor " << r.safety_factor;
            std::cout << '\n';
            for (std::size_t k = 0; k < r.frequencies_hz.size(); ++k) std::cout << "mode " << k + 1 << ": " << r.frequencies_hz[k] << " Hz\n";
            if (j.contains("crosscheck_mfem")) {
                const auto& mc = j["crosscheck_mfem"];
                std::cout << "MFEM cross-check: " << (mc["available"].get<bool>() ? "max displacement difference " + std::to_string(mc["max_displacement_diff"].get<double>()) + " (" + mc["note"].get<std::string>() + ")" : mc["note"].get<std::string>()) << '\n';
            }
            if (j.contains("crosscheck")) {
                std::cout << "CalculiX cross-check: " << (j["crosscheck"]["available"].get<bool>() ? "max displacement difference " + std::to_string(j["crosscheck"]["max_displacement_diff"].get<double>()) : j["crosscheck"]["note"].get<std::string>()) << '\n';
            }
        }
    } else {
        throw Error("unknown case '" + o.case_name + "' (tooth-bending or shock)");
    }
    if (as_json) std::cout << j.dump(2) << '\n';
    return kExitOk;
}

int cmd_verify_physics(const FeaOptions& o, bool as_json) {
    const auto checks = cases::run_physics_checks(o.full);
    bool ok = true;
    json j = json::array();
    for (const auto& c : checks) {
        ok &= c.passed;
        j.push_back({{"name", c.name}, {"category", c.category}, {"expected", c.expected}, {"actual", c.actual}, {"tolerance", c.tolerance},
                     {"passed", c.passed}, {"note", c.note}});
        if (!as_json) {
            std::cout << (c.passed ? "pass  " : "FAIL  ") << std::left << std::setw(70) << c.name << std::right << std::setprecision(5)
                      << " expected " << c.expected << ", got " << c.actual << " (tolerance " << c.tolerance * 100 << " %)\n";
        }
    }
    if (as_json) std::cout << json{{"ok", ok}, {"checks", j}}.dump(2) << '\n';
    else std::cout << (ok ? "all physics checks passed" : "SOME PHYSICS CHECKS FAILED") << " (" << checks.size() << " checks" << (o.full ? ", full set" : ", fast set; add --full") << ")\n";
    return ok ? kExitOk : kExitCheckFailed;
}

}  // namespace

void add_fea_commands(CLI::App& app, const bool& json_flag, std::function<int()>& chosen) {
    auto o = std::make_shared<FeaOptions>();
    auto pick = [&chosen, &json_flag, o](int (*fn)(const FeaOptions&, bool)) {
        return [&chosen, &json_flag, o, fn] { chosen = [&json_flag, o, fn] { return fn(*o, json_flag); }; };
    };

    auto* mats = app.add_subcommand("materials", "List the material catalogue");
    mats->add_option("--materials", o->materials_file, "Project materials TOML (added to the catalogue)")->check(CLI::ExistingFile);
    mats->callback(pick(cmd_materials));

    auto* th = app.add_subcommand("thermal", "Rate change per kelvin of a hairspring and balance pair");
    th->add_option("--hairspring", o->hairspring, "Hairspring material")->capture_default_str();
    th->add_option("--balance", o->balance, "Balance material")->capture_default_str();
    th->add_option("--materials", o->materials_file, "Project materials TOML")->check(CLI::ExistingFile);
    th->callback(pick(cmd_thermal));

    auto* fea = app.add_subcommand("fea", "Finite-element check of a part: tooth bending or shock and modes");
    fea->add_option("project", o->project, "Project file")->required()->check(CLI::ExistingFile);
    fea->add_option("--case", o->case_name, "tooth-bending or shock")->capture_default_str();
    fea->add_option("--part", o->part, "Part name")->required();
    fea->add_option("--material", o->material, "Material name (default: brass, steel for pinions)");
    fea->add_option("--materials", o->materials_file, "Project materials TOML")->check(CLI::ExistingFile);
    fea->add_option("--load-n", o->load_n, "Tangential tooth load in N (default: from the torque chain)");
    fea->add_option("--wind", o->wind, "State of wind for the torque chain")->capture_default_str();
    fea->add_option("--size-factor", o->size_factor, "Root element size in modules")->capture_default_str();
    fea->add_option("--size", o->size, "Element size in mm for the shock case")->capture_default_str();
    fea->add_option("--g", o->shock_g, "Shock acceleration in g")->capture_default_str();
    fea->add_flag("--in-plane", o->in_plane, "Shock along the dial instead of end shake");
    fea->add_option("--modes", o->modes, "Also compute this many natural frequencies");
    fea->add_option("--step", o->step_override, "STEP file (default: next to the project)");
    fea->add_option("--up", o->up, "CAD axis normal to the dial (default: the one found at import)");
    fea->add_option("--mesher", o->mesher, "Shock case mesher: section (mid-height section, extruded) or netgen (the whole B-rep; needs 'pip install netgen-mesher')")->capture_default_str();
    fea->add_option("--vtu", o->vtu, "Write the result for ParaView");
    fea->add_option("--vtu-compressed", o->vtu_compressed, "Write the result as a binary, zlib-compressed VTU written by VTK (needs a build with VTK)");
    fea->add_option("--hdf5", o->hdf5, "Write mesh and fields to an HDF5 file (needs a build with libhdf5)");
    fea->add_flag("--crosscheck", o->crosscheck, "Compare with CalculiX if it is installed");
    fea->add_flag("--mfem", o->mfem, "Compare with MFEM, an independent finite element code, if this build has it (shock case)");
    fea->add_option("--beat-rate", o->beat_rate, "Beat rate in vph (default: from project)");
    fea->callback(pick(cmd_fea));

    CLI::App* verify = nullptr;
    try {
        verify = app.get_subcommand("verify");
    } catch (const CLI::OptionNotFound&) {
        verify = app.add_subcommand("verify", "Cross-checks of the simulation");
        verify->require_subcommand(1);
    }
    auto* phys = verify->add_subcommand("physics", "Re-run the solvers against analytical answers on this machine");
    phys->add_flag("--full", o->full, "Finer meshes, benchmarks and convergence rates (slower)");
    phys->callback(pick(cmd_verify_physics));
}

}  // namespace ohs::cli
