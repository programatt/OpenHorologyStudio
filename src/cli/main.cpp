// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "animation/gltf_animation.hpp"
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <spdlog/sinks/stdout_sinks.h>

#include <chrono>
#include <csignal>
#include <thread>

#include "cli/cli_run.hpp"
#include "cli/daemon.hpp"
#include "cli/daemon_protocol.hpp"
#include "cli/dynamics_cli.hpp"
#include "cli/lessons_cli.hpp"
#include "cli/fea_cli.hpp"
#include "cli/manufacturing_cli.hpp"
#include "app/import.hpp"
#include "core/error.hpp"
#include "explain/explain.hpp"
#include "geometry/procedural.hpp"
#include "geometry/cad_io.hpp"
#include "geometry/part_cache.hpp"
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
        if (!c.label.empty()) j["components"].back()["label"] = c.label;
    }
    j["mobiles"] = m.mobiles.size();
    j["meshes"] = m.meshes.size();
    return j;
}

json train_json(const ohs::model::Movement& m) {
    const auto chain = ohs::rigging::train_chain(m);
    json j;
    j["connected"] = chain.connected;
    j["description"] = ohs::rigging::describe_chain(m, chain);
    j["path"] = json::array();
    for (int mobile : chain.path) j["path"].push_back(m.mobiles[static_cast<std::size_t>(mobile)].name);
    return j;
}

struct Options {
    bool json_output = false;
    bool verbose = false;

    // import
    std::string step_file, project_out, preset_file, up;
    bool auto_rig = true;
    double beat_rate = 0.0;
    int min_teeth = 6;

    // rig check / kinematics
    std::string project;
    bool strict = false;
    std::vector<std::string> expect;
    std::string root_mobile;

    // rig set
    std::string part_name, part_b, role_name, label_text;
    std::vector<std::string> part_names, assignments;  ///< rig set: the parts that get --role, and PART=ROLE pairs
    std::vector<std::string> label_sets, mesh_pairs;   ///< rig label --set PART=LABEL, rig mesh --pair A=B
    bool in_train{false};  ///< rig winding: put the mesh back in the train
    bool apply_solution{false};  ///< rig solve: declare the mesh of a candidate
    int pick{0};           ///< rig solve: which candidate

    // rig auto / suggest
    bool apply = false;
    bool reset = false;
    double min_confidence = 0.0;
    double coaxial_tolerance = 0.05;

    // animate
    std::string step_override, glb_out;
    double duration = 10.0, fps = 30.0, time_scale = 600.0;
    bool stepped = false;

    // explain
    std::string explain_part;
    bool explain_train = false;

    // dev gen-movement
    std::string gen_out;
    std::string socket_path;
    double sleep_seconds = 1.0;
    std::vector<std::string> warm_projects;

    // thirdparty
    std::vector<std::string> om10_args;
};

ohs::rigging::RigOptions rig_options(const Options& o) {
    ohs::rigging::RigOptions rig;
    rig.min_teeth = o.min_teeth;
    rig.coaxial_tolerance = o.coaxial_tolerance;
    return rig;
}

ohs::geometry::Axis parse_axis(const std::string& up);
ohs::geometry::Axis parse_axis(const std::string& up, const ohs::model::Movement& m) {
    return parse_axis(up.empty() ? m.dial_axis : up);
}

ohs::geometry::Axis parse_axis(const std::string& up) {
    if (up == "x") return ohs::geometry::Axis::X;
    if (up == "y") return ohs::geometry::Axis::Y;
    if (up == "z") return ohs::geometry::Axis::Z;
    throw ohs::Error("--up must be x, y or z");
}

int cmd_import(const Options& o) {
    ohs::app::ImportOptions opt;
    if (o.up.empty() || o.up == "auto") opt.auto_axis = true;
    else opt.features.dial_axis = parse_axis(o.up);
    opt.features.min_teeth = o.min_teeth;
    opt.cache_project = o.project_out;
    const ohs::cli::ImportHooks hooks = ohs::cli::import_hooks();
    opt.progress = hooks.progress;
    opt.cancel = hooks.cancel;
    opt.rig.beat_rate_vph = o.beat_rate;
    if (!o.preset_file.empty()) {
        opt.rig.presets = ohs::app::load_role_presets(o.preset_file);
        opt.mesh_presets = ohs::app::load_mesh_presets(o.preset_file);
    }

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
        for (const auto& i : issues) {
            j["issues"].push_back({{"severity", severity_name(i.severity)}, {"message", i.message}, {"parts", i.parts}});
        }
        j["errors"] = errors;
        j["warnings"] = warnings;
        j["train"] = train_json(m);
        std::cout << j.dump(2) << '\n';
    } else {
        for (const auto& c : m.components) {
            std::cout << std::left << std::setw(24) << c.features.name << std::setw(14) << to_string(c.role)
                      << " conf " << std::fixed << std::setprecision(2) << c.role_confidence << "  teeth "
                      << c.features.teeth << "  (" << c.role_source << ")";
            if (c.gear) {
                std::cout << "  m=" << c.gear->module << " " << c.gear->profile;
                if (c.gear->profile == "involute") std::cout << " " << std::setprecision(3) << c.gear->pressure_angle_deg << "deg";
                if (c.gear->profile == "cycloidal") std::cout << " rolling " << std::setprecision(3) << c.gear->rolling_radius << "m";
            }
            std::cout << '\n';
        }
        for (const auto& i : issues) std::cout << severity_name(i.severity) << ": " << i.message << '\n';
        const auto chain = ohs::rigging::train_chain(m);
        std::cout << "train: " << (chain.connected ? "connected, " : "") << ohs::rigging::describe_chain(m, chain) << '\n';
        const auto suggestions = ohs::rigging::suggest_meshes(m, rig_options(o));
        if (!suggestions.empty()) {
            std::cout << suggestions.size() << " possible mesh(es) to confirm: run 'ohs rig suggest " << o.project << "'\n";
        }
        std::cout << errors << " error(s), " << warnings << " warning(s)\n";
    }
    return (errors > 0 || (o.strict && warnings > 0)) ? kExitCheckFailed : kExitOk;
}

int cmd_rig_auto(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    ohs::rigging::RigOptions rig = rig_options(o);
    const auto report = ohs::rigging::re_rig(m, rig, !o.reset);
    ohs::model::save_rig(m, o.project);
    const auto chain = ohs::rigging::train_chain(m);
    if (o.json_output) {
        json j = movement_summary(m);
        j["role_changes"] = json::array();
        for (const auto& c : report.role_changes) {
            j["role_changes"].push_back({{"part", c.part},
                                         {"from", std::string(to_string(c.from))},
                                         {"to", std::string(to_string(c.to))},
                                         {"confidence", c.confidence}});
        }
        j["meshes_before"] = report.meshes_before;
        j["train"] = train_json(m);
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    std::cout << "re-rigged " << m.components.size() << " parts: " << report.role_changes.size() << " role(s) changed, "
              << report.mobiles_before << " -> " << report.mobiles_after << " mobiles, " << report.meshes_before << " -> "
              << report.meshes_after << " meshes\n";
    for (const auto& c : report.role_changes) {
        std::cout << "  " << c.part << ": " << to_string(c.from) << " -> " << to_string(c.to) << " (confidence " << std::fixed
                  << std::setprecision(2) << c.confidence << ")\n";
    }
    std::cout << "train: " << (chain.connected ? "connected, " : "") << ohs::rigging::describe_chain(m, chain) << '\n';
    return kExitOk;
}

int cmd_rig_walk(const Options& o) {
    const ohs::model::Movement m = ohs::model::load_rig(o.project);
    const auto chain = ohs::rigging::train_chain(m);
    const auto suggestions = ohs::rigging::suggest_meshes(m, rig_options(o));
    const auto walk = ohs::rigging::walk_train(m, chain, suggestions);
    const auto name = [&m](int component) { return m.components[static_cast<std::size_t>(component)].features.name; };
    if (o.json_output) {
        json j;
        j["complete"] = walk.complete;
        j["steps"] = json::array();
        for (const auto& s : walk.steps) {
            const char* kind = s.kind == ohs::rigging::WalkStep::Kind::Mesh ? "mesh"
                               : s.kind == ohs::rigging::WalkStep::Kind::Gap ? "gap" : "missing";
            json step = {{"kind", kind}, {"text", s.text}};
            if (s.driver >= 0) step["driver"] = name(s.driver);
            if (s.driven >= 0) step["driven"] = name(s.driven);
            j["steps"].push_back(std::move(step));
        }
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    int n = 0;
    for (const auto& s : walk.steps) {
        ++n;
        const bool mesh = s.kind == ohs::rigging::WalkStep::Kind::Mesh;
        std::cout << (mesh ? "" : "!") << n << ". " << s.text << '\n';
        for (int k : s.suggestions) {
            const auto& sg = suggestions[static_cast<std::size_t>(k)];
            std::cout << "     could be " << name(sg.a) << " <-> " << name(sg.b) << " (likelihood " << std::fixed << std::setprecision(2)
                      << sg.confidence << "): ohs rig mesh " << o.project << " " << name(sg.a) << " " << name(sg.b) << '\n';
        }
    }
    std::cout << (walk.complete ? "the train is complete from the barrel to the escape wheel\n"
                                : "the train is not complete: the last step above is what is missing\n");
    return kExitOk;
}

int cmd_rig_suggest(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    const ohs::rigging::RigOptions rig = rig_options(o);
    std::vector<ohs::rigging::MeshSuggestion> suggestions;
    for (const auto& s : ohs::rigging::suggest_meshes(m, rig)) {
        if (s.confidence >= o.min_confidence) suggestions.push_back(s);
    }
    const auto name = [&m](int component) { return m.components[static_cast<std::size_t>(component)].features.name; };
    if (o.apply) {
        for (const auto& s : suggestions) ohs::rigging::add_manual_mesh(m, name(s.a), name(s.b), rig);
        ohs::model::save_rig(m, o.project);
    }
    if (o.json_output) {
        json j = json::array();
        for (const auto& s : suggestions) {
            j.push_back({{"a", name(s.a)}, {"b", name(s.b)}, {"centre_distance", s.centre_distance}, {"module", s.module},
                         {"confidence", s.confidence}});
        }
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    for (const auto& s : suggestions) {
        std::cout << std::left << std::setw(24) << name(s.a) << " <-> " << std::setw(24) << name(s.b) << " centre distance "
                  << std::fixed << std::setprecision(3) << s.centre_distance << " mm, module " << s.module << ", confidence "
                  << std::setprecision(2) << s.confidence << '\n';
    }
    if (o.apply) {
        std::cout << "declared " << suggestions.size() << " mesh(es); " << m.meshes.size() << " meshes in all\n";
    } else if (suggestions.empty()) {
        std::cout << "no mesh to suggest\n";
    } else {
        std::cout << suggestions.size() << " suggestion(s); add --apply to declare them, or 'ohs rig mesh' for one\n";
    }
    return kExitOk;
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
        j["parts"] = json::array();
        for (const auto& a : t.articulations) {
            const bool spin = a.kind == ohs::kinematics::Articulation::Kind::Spin;
            json part = {{"part", m.components[static_cast<std::size_t>(a.component)].features.name},
                         {"role", std::string(ohs::model::to_string(m.components[static_cast<std::size_t>(a.component)].role))},
                         {"motion", spin ? "spin" : "follow"}, {"point", {a.point[0], a.point[1], a.point[2]}}, {"axis", {a.axis[0], a.axis[1], a.axis[2]}}};
            if (spin) {
                part["turns_per_root_turn"] = a.gain;
            } else {
                part["driver"] = m.mobiles[static_cast<std::size_t>(a.driver)].name;
                part["teeth"] = a.teeth;
                part["swing_deg"] = a.amplitude * 180.0 / std::numbers::pi;
            }
            j["parts"].push_back(std::move(part));
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
        for (const auto& a : t.articulations) {
            const auto& c = m.components[static_cast<std::size_t>(a.component)];
            std::cout << std::left << std::setw(6) << to_string(c.role) << ' ' << c.features.name;
            if (a.kind == ohs::kinematics::Articulation::Kind::Spin) {
                std::cout << " spins about its own axis, " << -a.gain << " turns per turn of the root\n";
            } else {
                std::cout << " swings " << std::abs(a.amplitude) * 180.0 / std::numbers::pi << " deg about (" << a.point[0] << ", "
                          << a.point[1] << ") for each of the " << a.teeth << " teeth of " << m.mobiles[static_cast<std::size_t>(a.driver)].name << '\n';
            }
        }
        for (const auto& w : t.warnings) std::cout << "warning: " << w << '\n';
        for (const auto& e : expectations) {
            std::cout << (e["ok"].get<bool>() ? "ok: " : "FAIL: ") << e["mobile"].get<std::string>() << " expected "
                      << e["expected_rpm"] << " rpm, got " << e["actual_rpm"] << " rpm\n";
        }
    }
    return failed ? kExitCheckFailed : kExitOk;
}

int cmd_rig_set(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    std::vector<std::pair<std::string, ohs::model::Role>> roles;
    if (!o.part_names.empty()) {
        if (o.role_name.empty()) throw ohs::Error("give --role for the parts named, or use --assign PART=ROLE");
        const auto role = ohs::model::role_from_string(o.role_name);
        if (!role) throw ohs::Error("unknown role '" + o.role_name + "'");
        for (const std::string& part : o.part_names) roles.emplace_back(part, *role);
    }
    for (const std::string& assignment : o.assignments) {
        // The last '=' splits them: a part name may hold a colon (OM10-00197:2) but a role name holds no '='.
        const auto eq = assignment.rfind('=');
        if (eq == std::string::npos || eq == 0) throw ohs::Error("--assign wants PART=ROLE, not '" + assignment + "'");
        const std::string role_name = assignment.substr(eq + 1);
        const auto role = ohs::model::role_from_string(role_name);
        if (!role) throw ohs::Error("unknown role '" + role_name + "'");
        roles.emplace_back(assignment.substr(0, eq), *role);
    }
    if (roles.empty()) throw ohs::Error("name the parts to change, with --role, or give --assign PART=ROLE");
    ohs::rigging::RigOptions rig;
    rig.min_teeth = o.min_teeth;
    ohs::rigging::set_roles(m, roles, rig);
    ohs::model::save_rig(m, o.project);
    if (roles.size() == 1) {
        std::cout << roles.front().first << " is now " << ohs::model::to_string(roles.front().second) << "; ";
    } else {
        std::cout << roles.size() << " parts changed; ";
    }
    std::cout << m.mobiles.size() << " mobiles, " << m.meshes.size() << " meshes\n";
    return kExitOk;
}

int cmd_rig_label(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    if (!o.label_sets.empty()) {
        // Several at once: nothing is written unless every part is there.
        if (!o.part_name.empty()) throw ohs::Error("give a part and a label, or --set PART=LABEL, not both");
        for (const std::string& set : o.label_sets) {
            const auto eq = set.find('=');
            if (eq == std::string::npos || eq == 0) throw ohs::Error("--set wants PART=LABEL, not '" + set + "'");
            const int idx = m.find_component(set.substr(0, eq));
            if (idx < 0) throw ohs::Error("no component named '" + set.substr(0, eq) + "'");
            m.components[static_cast<std::size_t>(idx)].label = set.substr(eq + 1);
        }
        ohs::model::save_rig(m, o.project);
        std::cout << o.label_sets.size() << " labels set\n";
        return kExitOk;
    }
    if (o.part_name.empty()) throw ohs::Error("name a part, or give --set PART=LABEL");
    const int idx = m.find_component(o.part_name);
    if (idx < 0) throw ohs::Error("no component named '" + o.part_name + "'");
    m.components[static_cast<std::size_t>(idx)].label = o.label_text;
    ohs::model::save_rig(m, o.project);
    if (o.label_text.empty()) std::cout << o.part_name << " has no label\n";
    else std::cout << o.part_name << " is labelled \"" << o.label_text << "\"\n";
    return kExitOk;
}

int cmd_rig_beat_rate(const Options& o) {
    if (!(o.beat_rate > 0.0)) throw ohs::Error("the beat rate must be above zero (vibrations per hour, for example 28800)");
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    m.beat_rate_vph = o.beat_rate;
    ohs::model::save_rig(m, o.project);
    std::cout << "beat rate " << o.beat_rate << " vph (" << o.beat_rate / 7200.0 << " Hz)\n";
    return kExitOk;
}

int cmd_rig_solve(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    ohs::rigging::RigOptions rig = rig_options(o);
    const double vph = o.beat_rate > 0 ? o.beat_rate : m.beat_rate_vph;
    auto solution = ohs::rigging::solve_train(m, vph, rig);
    const auto name = [&m](int component) { return m.components[static_cast<std::size_t>(component)].display_name(); };
    const auto part = [&m](int component) { return m.components[static_cast<std::size_t>(component)].features.name; };

    bool applied = false;
    const std::string solved_input = solution.input >= 0 ? part(solution.input) : std::string{};
    if (o.apply_solution && !solution.candidates.empty()) {
        const auto pick = static_cast<std::size_t>(o.pick);
        if (pick >= solution.candidates.size()) throw ohs::Error("there is no candidate " + std::to_string(o.pick));
        ohs::rigging::apply_solution(m, solution.candidates[pick], rig);
        ohs::model::save_rig(m, o.project);
        applied = true;
        solution = ohs::rigging::solve_train(m, vph, rig);  // the hands, now that the train is joined up
    }

    if (o.json_output) {
        json j;
        j["connected"] = ohs::rigging::train_chain(m).connected;
        j["beat_rate_vph"] = vph;
        j["applied"] = applied;
        j["candidates"] = json::array();
        for (const auto& c : solution.candidates) {
            j["candidates"].push_back({{"driver", part(c.driver)}, {"input", part(c.input)}, {"teeth", c.teeth},
                                       {"estimated", c.estimated}, {"centre_distance", c.centre}, {"module", c.module},
                                       {"residual", c.residual}, {"driver_turns_per_hour", c.driver_rate}});
        }
        j["hands"] = json::array();
        for (const auto& h : solution.hands) j["hands"].push_back({{"hand", h.hand}, {"mobile", h.description}, {"turns_per_hour", h.rate}});
        j["notes"] = solution.notes;
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }

    const bool connected = ohs::rigging::train_chain(m).connected;
    if (applied) std::cout << "declared the mesh that drives " << solved_input << "\n";
    std::cout << "the train is " << (connected ? "connected from the barrel to the escape wheel" : "not connected") << ", beat rate " << vph << " vph\n";
    if (!solution.candidates.empty()) {
        std::cout << "what could drive " << name(solution.input) << " (" << part(solution.input) << "):\n";
        std::size_t k = 0;
        for (const auto& c : solution.candidates) {
            std::cout << "  " << k++ << ": " << name(c.driver) << " (" << part(c.driver) << "), " << c.teeth << " teeth"
                      << (c.estimated ? " (worked out, none found)" : "") << ", centre distance " << c.centre << " mm, module " << c.module
                      << ", fit " << std::fixed << std::setprecision(3) << c.residual << std::defaultfloat << ", would turn " << c.driver_rate
                      << " times an hour\n";
        }
        std::cout << "declare the first with `ohs rig solve --apply` (or `--pick N`)\n";
    }
    for (const auto& h : solution.hands) std::cout << h.hand << " wheel: " << h.description << ", " << h.rate << " turns an hour\n";
    for (const auto& n : solution.notes) std::cout << "note: " << n << '\n';
    return kExitOk;
}

int cmd_rig_unmesh(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    ohs::rigging::remove_manual_mesh(m, o.part_name, o.part_b, rig_options(o));
    ohs::model::save_rig(m, o.project);
    std::cout << o.part_name << " / " << o.part_b << " no longer mesh; " << m.meshes.size() << " meshes\n";
    return kExitOk;
}

int cmd_rig_winding(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    ohs::rigging::set_mesh_winding(m, o.part_name, o.part_b, !o.in_train);
    ohs::model::save_rig(m, o.project);
    const auto chain = ohs::rigging::train_chain(m);
    std::cout << o.part_name << " / " << o.part_b << (o.in_train ? " is a link of the train again" : " is left out of the train (winding)")
              << "; the train is " << (chain.connected ? "connected" : "not connected") << '\n';
    return kExitOk;
}

int cmd_rig_mesh(const Options& o) {
    ohs::model::Movement m = ohs::model::load_rig(o.project);
    ohs::rigging::RigOptions rig;
    std::vector<std::pair<std::string, std::string>> pairs;
    if (!o.part_name.empty() || !o.part_b.empty()) pairs.emplace_back(o.part_name, o.part_b);
    for (const std::string& pair : o.mesh_pairs) {
        // Part names hold colons (OM10-00197:2) but no '=': the pair splits there.
        const auto eq = pair.find('=');
        if (eq == std::string::npos || eq == 0 || eq + 1 == pair.size()) throw ohs::Error("--pair wants A=B, not '" + pair + "'");
        pairs.emplace_back(pair.substr(0, eq), pair.substr(eq + 1));
    }
    if (pairs.empty() || (pairs.front().first.empty() || pairs.front().second.empty())) {
        throw ohs::Error("name two parts, or give --pair A=B");
    }
    // Nothing is written unless every mesh could be declared.
    for (const auto& [a, b] : pairs) ohs::rigging::add_manual_mesh(m, a, b, rig);
    ohs::model::save_rig(m, o.project);
    if (pairs.size() == 1) std::cout << pairs.front().first << " now meshes with " << pairs.front().second << "; " << m.meshes.size() << " meshes\n";
    else std::cout << pairs.size() << " meshes declared; " << m.meshes.size() << " meshes\n";
    return kExitOk;
}

int cmd_animate(const Options& o) {
    const ohs::model::Movement m = ohs::model::load_rig(o.project);
    const double vph = o.beat_rate > 0 ? o.beat_rate : m.beat_rate_vph;
    const auto train = ohs::kinematics::analyze_train(m, vph);

    fs::path step = o.step_override;
    if (step.empty()) step = fs::path(o.project).parent_path() / m.source_geometry;
    const auto parts = ohs::geometry::load_parts(step, o.project, o.step_override.empty());

    ohs::animation::AnimationOptions opt;
    opt.duration_seconds = o.duration;
    opt.fps = o.fps;
    opt.time_scale = o.time_scale;
    opt.stepped = o.stepped;
    const bool usd = fs::path(o.glb_out).extension() == ".usda";
    const auto stats = usd ? ohs::animation::write_animated_usd(m, parts, train, parse_axis(o.up, m), opt, o.glb_out)
                           : ohs::animation::write_animated_glb(m, parts, train, parse_axis(o.up, m), opt, o.glb_out);
    if (o.json_output) {
        json j = {{"output", o.glb_out}, {"keyframes", stats.keyframes}, {"triangles", stats.triangles},
                  {"animated_nodes", stats.animated_nodes}, {"max_degrees_per_keyframe", stats.max_degrees_per_keyframe}};
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << "wrote " << o.glb_out << ": " << stats.keyframes << " keyframes, " << stats.triangles
                  << " triangles, " << stats.animated_nodes << " animated parts (max "
                  << stats.max_degrees_per_keyframe << " deg between keyframes)\n";
    }
    return kExitOk;
}

int cmd_explain(const Options& o) {
    const ohs::model::Movement m = ohs::model::load_rig(o.project);
    const double vph = o.beat_rate > 0 ? o.beat_rate : m.beat_rate_vph;
    const auto train = ohs::kinematics::analyze_train(m, vph);
    if (!o.explain_train && o.explain_part.empty()) throw ohs::Error("give a part name or --train");

    json j;
    if (!o.explain_part.empty()) {
        const auto e = ohs::explain::explain_part(m, train, o.explain_part);
        if (o.json_output) {
            j["part"] = {{"name", e.name}, {"role", e.role}, {"summary", e.summary}, {"facts", e.facts}};
        } else {
            std::cout << ohs::explain::to_text(e);
        }
    }
    if (o.explain_train) {
        const auto w = ohs::explain::walk_train(m, train);
        if (o.json_output) {
            j["train"] = {{"root", w.root}, {"conclusion", w.conclusion}, {"steps", json::array()}};
            for (const auto& s : w.steps) {
                j["train"]["steps"].push_back({{"driver", s.driver}, {"driven", s.driven}, {"driver_teeth", s.driver_teeth},
                                               {"driven_teeth", s.driven_teeth}, {"mobile", s.driven_mobile},
                                               {"step_ratio", s.step_ratio}, {"cumulative_ratio", s.cumulative_ratio},
                                               {"rph", s.revs_per_hour}});
            }
        } else {
            std::cout << ohs::explain::to_text(w);
        }
    }
    if (o.json_output) std::cout << j.dump(2) << '\n';
    return kExitOk;
}

int cmd_serve(const Options& o) {
    return ohs::cli::serve(o.socket_path.empty() ? ohs::cli::daemon::default_socket_path() : o.socket_path, o.warm_projects);
}

int cmd_dev_crash(const Options&) {
    std::cerr << "about to crash on purpose\n" << std::flush;
    std::raise(SIGSEGV);
    return kExitError;
}

int cmd_dev_sleep(const Options& o) {
    std::cout << "sleeping " << o.sleep_seconds << " s\n" << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(o.sleep_seconds * 1000)));
    std::cout << "slept\n";
    return kExitOk;
}

int cmd_gen_movement(const Options& o) {
    ohs::geometry::export_step(ohs::geometry::make_test_movement(), o.gen_out);
    std::cout << "wrote " << o.gen_out << '\n';
    return kExitOk;
}

fs::path find_script(const std::string& name) {
    if (const char* dir = std::getenv("OHS_SCRIPTS_DIR")) return fs::path(dir) / "thirdparty" / name;
    // The source tree first (development), then where the packages install it.
    for (const char* base : {OHS_SOURCE_DIR "/scripts", "/usr/share/ohs/scripts", "/usr/local/share/ohs/scripts"}) {
        const fs::path p = fs::path(base) / "thirdparty" / name;
        if (fs::exists(p)) return p;
    }
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

int ohs::cli::run(int argc, char** argv) {
    // OCCT prints transfer statistics to standard output, which would corrupt --json output.
    Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
    {
        static const bool stderr_logging = [] {
            spdlog::set_default_logger(spdlog::stderr_logger_mt("ohs"));
            return true;
        }();
        (void)stderr_logging;
    }
    Options o;
    CLI::App app{"Open Horology Studio: watch movement design and analysis", "ohs"};
    app.set_version_flag("--version", OHS_VERSION);
    app.require_subcommand(1);
    app.add_flag("--json", o.json_output, "Machine-readable output");
    app.add_flag("-v,--verbose", o.verbose, "Debug logging");

    int (*handler)(const Options&) = nullptr;
    auto pick = [&handler](int (*fn)(const Options&)) { return [&handler, fn] { handler = fn; }; };

    auto* import = app.add_subcommand("import", "Import a STEP, IGES or DXF file and auto-rig it");
    import->add_option("step", o.step_file, "CAD file: .step, .stp, .iges, .igs or .dxf (a DXF profile is extruded by its own thickness, else 1 mm)")->required()->check(CLI::ExistingFile);
    import->add_option("-o,--output", o.project_out, "Project (rig) file to write")->required();
    import->add_flag("--auto-rig,!--no-auto-rig", o.auto_rig, "Infer roles, mobiles and meshes (default on)");
    import->add_option("--preset", o.preset_file, "Role preset TOML (name regex -> role)")->check(CLI::ExistingFile);
    import->add_option("--up", o.up, "CAD axis normal to the dial: x, y, z or auto (default: auto, the axis the parts are thinnest along)");
    import->add_option("--beat-rate", o.beat_rate, "Beat rate in vph recorded in the project");
    import->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    import->callback(pick(cmd_import));

    auto* rig = app.add_subcommand("rig", "Inspect a rig");
    rig->require_subcommand(1);
    auto* check = rig->add_subcommand("check", "Validate a project's rig");
    check->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    check->add_flag("--strict", o.strict, "Treat warnings as failures");
    check->callback(pick(cmd_rig_check));

    auto* set = rig->add_subcommand("set", "Override a part's role and re-infer mobiles and meshes");
    set->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    set->add_option("parts", o.part_names, "Parts to give the --role (any number)");
    set->add_option("--role", o.role_name, "plate, bridge, barrel, arbor, wheel, pinion, escape_wheel, ...");
    set->add_option("--assign", o.assignments, "PART=ROLE, repeatable: several different roles in one edit");
    set->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    set->callback(pick(cmd_rig_set));

    auto* label = rig->add_subcommand("label", "Give a part a label to show in place of its name (none clears it)");
    label->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    label->add_option("part", o.part_name, "Part name");
    label->add_option("label", o.label_text, "The label; leave out to remove it");
    label->add_option("--set", o.label_sets, "PART=LABEL, repeatable: several labels in one edit");
    label->callback(pick(cmd_rig_label));

    auto* beat = rig->add_subcommand("beat-rate", "Set the beat rate of the project, in vibrations per hour");
    beat->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    beat->add_option("rate", o.beat_rate, "Vibrations per hour: 18000, 21600, 25200, 28800 ...")->required();
    beat->callback(pick(cmd_rig_beat_rate));

    auto* solve = rig->add_subcommand("solve", "Find what drives the loose end of the train, and which wheels turn the hands");
    solve->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    solve->add_option("--beat-rate", o.beat_rate, "Beat rate in vph (the project's when left out)");
    solve->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    solve->add_flag("--apply", o.apply_solution, "Declare the mesh of the best candidate");
    solve->add_option("--pick", o.pick, "With --apply: the candidate to declare, as numbered in the list")->capture_default_str();
    solve->callback(pick(cmd_rig_solve));

    auto* unmesh = rig->add_subcommand("unmesh", "Take away a mesh that was declared by hand");
    unmesh->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    unmesh->add_option("part", o.part_name, "One of the two parts")->required();
    unmesh->add_option("partner", o.part_b, "The other")->required();
    unmesh->callback(pick(cmd_rig_unmesh));

    auto* winding = rig->add_subcommand("winding", "Leave a mesh out of the going train (the winding work), or put it back");
    winding->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    winding->add_option("part", o.part_name, "One of the two meshed parts")->required();
    winding->add_option("partner", o.part_b, "The other")->required();
    winding->add_flag("--in-train", o.in_train, "Put the mesh back in the train");
    winding->callback(pick(cmd_rig_winding));

    auto* mesh = rig->add_subcommand("mesh", "Declare that two toothed parts mesh although the geometry does not show it");
    mesh->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    mesh->add_option("part", o.part_name, "First part");
    mesh->add_option("part_b", o.part_b, "Second part");
    mesh->add_option("--pair", o.mesh_pairs, "A=B, repeatable: several meshes in one edit");
    mesh->callback(pick(cmd_rig_mesh));

    auto* autorig = rig->add_subcommand("auto", "Run the auto-rig again on an imported project: roles, mobiles and meshes");
    autorig->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    autorig->add_flag("--reset", o.reset, "Also forget the roles you set and the meshes you declared");
    autorig->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    autorig->add_option("--coaxial-tolerance", o.coaxial_tolerance, "Largest centre offset in mm for parts of one mobile")
        ->capture_default_str();
    autorig->callback(pick(cmd_rig_auto));

    auto* suggest = rig->add_subcommand("suggest", "List toothed parts that probably mesh but were not accepted, and optionally declare them");
    suggest->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    suggest->add_flag("--apply", o.apply, "Declare the suggested meshes (as 'ohs rig mesh' does)");
    suggest->add_option("--min-confidence", o.min_confidence, "Leave out suggestions below this confidence (0 to 0.5)")
        ->capture_default_str();
    suggest->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    suggest->callback(pick(cmd_rig_suggest));

    auto* walk = rig->add_subcommand("walk", "Step through the train from the barrel to the escape wheel, mesh by mesh, and show what is missing");
    walk->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    walk->add_option("--min-teeth", o.min_teeth, "Fewest teeth to call a part toothed")->capture_default_str();
    walk->callback(pick(cmd_rig_walk));

    auto* anim = app.add_subcommand("animate", "Export the running train as an animated binary glTF (.glb) or OpenUSD text file (.usda)");
    anim->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    anim->add_option("-o,--output", o.glb_out, "File to write: .glb (glTF 2.0) or .usda (OpenUSD)")->required();
    anim->add_option("--step", o.step_override, "STEP file (default: the project's source next to it)");
    anim->add_option("--up", o.up, "CAD axis normal to the dial (default: the one found at import)");
    anim->add_option("--beat-rate", o.beat_rate, "Beat rate in vph (default: from project)");
    anim->add_option("--duration", o.duration, "Animation length in seconds")->capture_default_str();
    anim->add_option("--fps", o.fps, "Minimum keyframes per second")->capture_default_str();
    anim->add_option("--time-scale", o.time_scale, "Movement seconds per animation second")->capture_default_str();
    anim->add_flag("--stepped", o.stepped, "Advance the train one escapement step per beat");
    anim->callback(pick(cmd_animate));

    auto* expl = app.add_subcommand("explain", "Explain a part or walk through the gear train with this movement's numbers");
    expl->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    expl->add_option("part", o.explain_part, "Part name");
    expl->add_flag("--train", o.explain_train, "Walk through the going train step by step");
    expl->add_option("--beat-rate", o.beat_rate, "Beat rate in vph (default: from project)");
    expl->callback(pick(cmd_explain));

    auto* kin = app.add_subcommand("kinematics", "Rigid-body ratios and speeds of the going train");
    kin->add_option("project", o.project, "Project file")->required()->check(CLI::ExistingFile);
    kin->add_option("--beat-rate", o.beat_rate, "Beat rate in vph (default: from project)");
    kin->add_option("--root", o.root_mobile, "Root mobile name (default: the barrel's mobile)");
    kin->add_option("--expect", o.expect, "Check a mobile's speed, NAME=RPM (repeatable)");
    kin->callback(pick(cmd_kinematics));

    auto* serve = app.add_subcommand("serve", "Run commands for `ohs-client` from a resident process (Unix sockets): no start-up, warm geometry");
    serve->add_option("--socket", o.socket_path, "Socket path (default: $OHS_SOCKET, else $XDG_RUNTIME_DIR/ohs.sock)");
    serve->add_option("--warm", o.warm_projects, "A project whose geometry is read now and kept for every request; repeatable");
    serve->callback(pick(cmd_serve));

    auto* dev = app.add_subcommand("dev", "Developer tools");
    dev->require_subcommand(1);
    auto* gen = dev->add_subcommand("gen-movement", "Write the procedural test movement as STEP");
    gen->add_option("-o,--output", o.gen_out, "STEP file to write")->required();
    gen->callback(pick(cmd_gen_movement));
    auto* crash = dev->add_subcommand("crash", "Crash the process on purpose (tests that `ohs serve` survives a crashing command)");
    crash->callback(pick(cmd_dev_crash));
    auto* nap = dev->add_subcommand("sleep", "Wait, then finish (tests cancelling a long command)");
    nap->add_option("seconds", o.sleep_seconds, "How long")->capture_default_str();
    nap->callback(pick(cmd_dev_sleep));

    auto* third = app.add_subcommand("thirdparty", "Third-party data (never committed to the repo)");
    third->require_subcommand(1);
    auto* om10 = third->add_subcommand("om10", "openmovement OM10");
    om10->require_subcommand(1);
    auto* fetch = om10->add_subcommand("fetch", "Install and import a user-supplied OM10 STEP file. Options after the command go to scripts/thirdparty/fetch_om10.py: --file PATH, --accept, --no-import, --ohs PATH");
    fetch->allow_extras();
    fetch->callback([&] {
        o.om10_args = fetch->remaining();
        handler = cmd_om10_fetch;
    });

    std::function<int()> custom;
    ohs::cli::add_dynamics_commands(app, o.json_output, custom);
    ohs::cli::add_fea_commands(app, o.json_output, custom);
    ohs::cli::add_manufacturing_commands(app, o.json_output, custom);
    ohs::cli::add_lesson_commands(app, o.json_output, custom);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e) == 0 ? kExitOk : kExitError;
    }
    spdlog::set_level(o.verbose ? spdlog::level::debug : spdlog::level::warn);
    try {
        if (handler) return handler(o);
        return custom ? custom() : kExitError;
    } catch (const ohs::Error& e) {
        std::cerr << "error: " << e.what() << '\n';
        return kExitError;
    } catch (const std::exception& e) {
        std::cerr << "internal error: " << e.what() << '\n';
        return kExitError;
    }
}
