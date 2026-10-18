// SPDX-License-Identifier: Apache-2.0
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "app/import.hpp"
#include "core/error.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

constexpr int kExitOk = 0;
constexpr int kExitCheckFailed = 1;
constexpr int kExitError = 2;

const char* severity_name(ohs::rigging::Severity s) {
    switch (s) {
        case ohs::rigging::Severity::Info: return "info";
        case ohs::rigging::Severity::Warning: return "warning";
        case ohs::rigging::Severity::Error: return "error";
    }
    return "?";
}

json movement_summary(const ohs::model::Movement& m) {
    json j;
    j["name"] = m.name;
    j["components"] = json::array();
    for (const auto& c : m.components) {
        j["components"].push_back({{"name", c.features.name},
                                   {"role", std::string(to_string(c.role))},
                                   {"confidence", c.role_confidence},
                                   {"teeth", c.features.teeth}});
    }
    j["mobiles"] = m.mobiles.size();
    j["meshes"] = m.meshes.size();
    return j;
}

struct Options {
    bool json_output = false;
    bool verbose = false;

    // import
    std::string step_file, project_out, preset_file, up = "z";
    bool auto_rig = true;
    double beat_rate = 0.0;
    int min_teeth = 6;

    // rig check / kinematics
    std::string project;
    bool strict = false;
    std::vector<std::string> expect;
    std::string root_mobile;

    // dev gen-movement
    std::string gen_out;

    // thirdparty
    std::vector<std::string> om10_args;
};

int cmd_import(const Options& o) {
    ohs::app::ImportOptions opt;
    const std::map<std::string, ohs::geometry::Axis> axes{
        {"x", ohs::geometry::Axis::X}, {"y", ohs::geometry::Axis::Y}, {"z", ohs::geometry::Axis::Z}};
    const auto axis = axes.find(o.up);
    if (axis == axes.end()) throw ohs::Error("--up must be x, y or z");
    opt.features.dial_axis = axis->second;
    opt.features.min_teeth = o.min_teeth;
    opt.rig.beat_rate_vph = o.beat_rate;
    if (!o.preset_file.empty()) opt.rig.presets = ohs::app::load_role_presets(o.preset_file);

    ohs::model::Movement m = ohs::app::import_movement(o.step_file, opt);
    ohs::model::save_rig(m, o.project_out);

    if (o.json_output) {
        json j = movement_summary(m);
        j["project"] = o.project_out;
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << "imported " << m.components.size() << " parts, " << m.mobiles.size() << " mobiles, "
                  << m.meshes.size() << " meshes\nwrote " << o.project_out << '\n';
    }
    return kExitOk;
}

int cmd_rig_check(const Options& o) {
    const ohs::model::Movement m = ohs::model::load_rig(o.project);
    const auto issues = ohs::rigging::check_rig(m);
    int errors = 0, warnings = 0;
    for (const auto& i : issues) {
        errors += i.severity == ohs::rigging::Severity::Error;
        warnings += i.severity == ohs::rigging::Severity::Warning;
    }
    if (o.json_output) {
        json j = movement_summary(m);
        j["issues"] = json::array();
        for (const auto& i : issues) j["issues"].push_back({{"severity", severity_name(i.severity)}, {"message", i.message}});
        j["errors"] = errors;
        j["warnings"] = warnings;
        std::cout << j.dump(2) << '\n';
    } else {
        for (const auto& c : m.components) {
            std::cout << std::left << std::setw(24) << c.features.name << std::setw(14) << to_string(c.role)
                      << " conf " << std::fixed << std::setprecision(2) << c.role_confidence << "  teeth "
                      << c.features.teeth << "  (" << c.role_source << ")\n";
        }
        for (const auto& i : issues) std::cout << severity_name(i.severity) << ": " << i.message << '\n';
        std::cout << errors << " error(s), " << warnings << " warning(s)\n";
    }
    return (errors > 0 || (o.strict && warnings > 0)) ? kExitCheckFailed : kExitOk;
}

int cmd_kinematics(const Options& o) {
    const ohs::model::Movement m = ohs::model::load_rig(o.project);
    const double vph = o.beat_rate > 0 ? o.beat_rate : m.beat_rate_vph;
    int root = -1;
    if (!o.root_mobile.empty()) {
        for (std::size_t i = 0; i < m.mobiles.size(); ++i) {
            if (m.mobiles[i].name == o.root_mobile) root = static_cast<int>(i);
        }
        if (root < 0) throw ohs::Error("no mobile named " + o.root_mobile);
    }
    const auto t = ohs::kinematics::analyze_train(m, vph, root);

    // --expect NAME=RPM checks a mobile's speed against a known value (0.1 % tolerance).
    bool failed = false;
    json expectations = json::array();
    for (const std::string& e : o.expect) {
        const auto eq = e.find('=');
        if (eq == std::string::npos) throw ohs::Error("--expect needs NAME=RPM, got '" + e + "'");
        const std::string name = e.substr(0, eq);
        const double want = std::stod(e.substr(eq + 1));
        const auto* mm = t.find(name);
        if (!mm || !mm->reached || vph <= 0) throw ohs::Error("cannot check '" + name + "': unknown, unreached or no beat rate");
        const double got = mm->revs_per_hour / 60.0;
        const bool ok = std::abs(got - want) <= 1e-3 * std::abs(want);
        failed |= !ok;
        expectations.push_back({{"mobile", name}, {"expected_rpm", want}, {"actual_rpm", got}, {"ok", ok}});
    }

    if (o.json_output) {
        json j;
        j["root"] = m.mobiles[t.root].name;
        j["beat_rate_vph"] = vph;
        j["escape_teeth"] = t.escape_teeth;
        j["beats_per_root_rev"] = t.beats_per_root_rev;
        j["root_period_hours"] = t.root_period_hours;
        j["mobiles"] = json::array();
        for (const auto& mm : t.mobiles) {
            j["mobiles"].push_back({{"name", mm.name}, {"reached", mm.reached}, {"ratio", mm.ratio},
                                    {"rph", mm.revs_per_hour}, {"rpm", mm.revs_per_hour / 60.0}});
        }
        j["warnings"] = t.warnings;
        j["expectations"] = expectations;
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << "root " << m.mobiles[t.root].name << ", beat rate " << vph << " vph, escape wheel "
                  << t.escape_teeth << " teeth\n";
        if (t.root_period_hours > 0) std::cout << "root turns once per " << t.root_period_hours << " h\n";
        for (std::size_t i = 0; i < t.mobiles.size(); ++i) {
            const auto& mm = t.mobiles[i];
            std::cout << std::left << std::setw(6) << mm.name << " ratio " << std::setw(12) << mm.ratio << " rpm "
                      << mm.revs_per_hour / 60.0 << "  [";
            for (std::size_t k = 0; k < m.mobiles[i].components.size(); ++k) {
                std::cout << (k ? ", " : "") << m.components[m.mobiles[i].components[k]].features.name;
            }
            std::cout << "]\n";
        }
        for (const auto& w : t.warnings) std::cout << "warning: " << w << '\n';
        for (const auto& e : expectations) {
            std::cout << (e["ok"].get<bool>() ? "ok: " : "FAIL: ") << e["mobile"].get<std::string>() << " expected "
                      << e["expected_rpm"] << " rpm, got " << e["actual_rpm"] << " rpm\n";
        }
    }
    return failed ? kExitCheckFailed : kExitOk;
}

int cmd_gen_movement(const Options& o) {
    ohs::geometry::export_step(ohs::geometry::make_test_movement(), o.gen_out);
    std::cout << "wrote " << o.gen_out << '\n';
    return kExitOk;
}

fs::path find_script(const std::string& name) {
    if (const char* dir = std::getenv("OHS_SCRIPTS_DIR")) return fs::path(dir) / "thirdparty" / name;
    return fs::path(OHS_SOURCE_DIR) / "scripts" / "thirdparty" / name;
}

int cmd_om10_fetch(const Options& o) {
    const fs::path script = find_script("fetch_om10.py");
    if (!fs::exists(script)) throw ohs::Error("script not found: " + script.string());
    std::string cmd = "python3 '" + script.string() + "'";
    for (const std::string& a : o.om10_args) cmd += " '" + a + "'";
    return std::system(cmd.c_str()) == 0 ? kExitOk : kExitCheckFailed;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    CLI::App app{"Open Horology Studio: watch movement design and analysis", "ohs"};
    app.set_version_flag("--version", OHS_VERSION);
    app.require_subcommand(1);
    app.add_flag("--json", o.json_output, "Machine-readable output");
    app.add_flag("-v,--verbose", o.verbose, "Debug logging");

    int (*handler)(const Options&) = nullptr;
    auto pick = [&handler](int (*fn)(const Options&)) { return [&handler, fn] { handler = fn; }; };

    auto* import = app.add_subcommand("import", "Import a STEP assembly and auto-rig it");
    import->add_option("step", o.step_file, "STEP file")->required()->check(CLI::ExistingFile);
    import->add_option("-o,--output", o.project_out, "Project (rig) file to write")->required();
    import->add_flag("--auto-rig,!--no-auto-rig", o.auto_rig, "Infer roles, mobiles and meshes (default on)");
    import->add_option("--preset", o.preset_file, "Role preset TOML (name regex -> role)")->check(CLI::ExistingFile);
    import->add_option("--up", o.up, "CAD axis normal to the dial: x, y or z")->capture_default_str();
    import->add_option("--beat-rate", o.beat_rate, "Beat rate in vph recorded in the project");
    import->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    import->callback(pick(cmd_import));

    auto* rig = app.add_subcommand("rig", "Inspect a rig");
    rig->require_subcommand(1);
    auto* check = rig->add_subcommand("check", "Validate a project's rig");
    check->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    check->add_flag("--strict", o.strict, "Treat warnings as failures");
    check->callback(pick(cmd_rig_check));

    auto* kin = app.add_subcommand("kinematics", "Rigid-body ratios and speeds of the going train");
    kin->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    kin->add_option("--beat-rate", o.beat_rate, "Beat rate in vph (default: from project)");
    kin->add_option("--root", o.root_mobile, "Root mobile name (default: the barrel's mobile)");
    kin->add_option("--expect", o.expect, "Check a mobile's speed, NAME=RPM (repeatable)");
    kin->callback(pick(cmd_kinematics));

    auto* dev = app.add_subcommand("dev", "Developer tools");
    dev->require_subcommand(1);
    auto* gen = dev->add_subcommand("gen-movement", "Write the procedural test movement as STEP");
    gen->add_option("-o,--output", o.gen_out, "STEP file to write")->required();
    gen->callback(pick(cmd_gen_movement));

    auto* third = app.add_subcommand("thirdparty", "Third-party data (never committed to the repo)");
    third->require_subcommand(1);
    auto* om10 = third->add_subcommand("om10", "openmovement OM10");
    om10->require_subcommand(1);
    auto* fetch = om10->add_subcommand("fetch", "Install and import a user-supplied OM10 STEP file");
    fetch->allow_extras();
    fetch->callback([&] {
        o.om10_args = fetch->remaining();
        handler = cmd_om10_fetch;
    });

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e) == 0 ? kExitOk : kExitError;
    }
    spdlog::set_level(o.verbose ? spdlog::level::debug : spdlog::level::warn);
    try {
        return handler ? handler(o) : kExitError;
    } catch (const ohs::Error& e) {
        std::cerr << "error: " << e.what() << '\n';
        return kExitError;
    } catch (const std::exception& e) {
        std::cerr << "internal error: " << e.what() << '\n';
        return kExitError;
    }
}
