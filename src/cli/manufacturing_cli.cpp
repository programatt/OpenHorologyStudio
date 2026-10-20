// SPDX-License-Identifier: Apache-2.0
#include "cli/manufacturing_cli.hpp"

#include <map>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "dynamics/calibre.hpp"
#include "geometry/section.hpp"
#include "geometry/shape_groups.hpp"
#include "geometry/cad_io.hpp"
#include "geometry/part_cache.hpp"
#include "kinematics/kinematics.hpp"
#include "manufacturing/collision.hpp"
#include "manufacturing/dfm.hpp"
#include "manufacturing/gear_export.hpp"
#include "manufacturing/part_drawing.hpp"
#include "manufacturing/tolerance.hpp"
#include "plugins/plugin_loader.hpp"
#include "model/rig_io.hpp"
#include "report/document.hpp"
#include "reports/report_builder.hpp"

using nlohmann::json;
namespace fs = std::filesystem;

namespace ohs::cli {

namespace {

constexpr int kExitOk = 0;
constexpr int kExitCheckFailed = 1;

struct MfgOptions {
    std::string project, part, step_override, up, out, rules_file, process = "mill,wheel-cutting", tolerance_file, calibre_file, material = "brass_leaded";
    std::string out_step, out_dxf, out_svg, out_pdf, name = "gear", formats = "html,pdf", gear_profile = "involute";
    std::vector<std::string> plugins;
    gear::GearSpec gear;
    double thickness = 0.5, beat_rate = 0, fail_above = 0.02;
    int samples = 10000;
    unsigned long seed = 42;
    bool pmi = false, profile_only = false, list = false, no_fea = false, no_timekeeping = false, no_geometry_sections = false;
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

void write_text(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw Error("cannot write " + path);
    f << text;
}

std::string extension(const std::string& path) {
    const auto dot = path.rfind('.');
    std::string e = dot == std::string::npos ? "" : path.substr(dot + 1);
    for (char& c : e) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return e;
}

std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::vector<manufacturing::ProcessRules> extra_rules(const MfgOptions& o) {
    return o.rules_file.empty() ? std::vector<manufacturing::ProcessRules>{} : manufacturing::load_rule_sets(o.rules_file);
}

std::vector<geometry::ImportedPart> load_parts(const MfgOptions& o, const model::Movement& m) {
    fs::path step = o.step_override;
    if (step.empty()) step = fs::path(o.project).parent_path() / m.source_geometry;
    return geometry::load_parts(step, o.project, o.step_override.empty());
}

int cmd_gear(const MfgOptions& options, bool as_json) {
    MfgOptions o = options;
    if (o.gear_profile == "involute") o.gear.profile = gear::Profile::Involute;
    else if (o.gear_profile == "cycloidal") o.gear.profile = gear::Profile::Cycloidal;
    else throw Error("--profile must be involute or cycloidal");
    if (o.out_step.empty() && o.out_dxf.empty() && o.out_svg.empty() && o.out_pdf.empty()) throw Error("give at least one of --step-out, --dxf, --svg, --pdf");
    manufacturing::DrawingNotes notes;
    notes.title = o.name;
    notes.thickness = o.thickness;
    notes.material = o.material;
    const auto info = manufacturing::gear_info(o.gear, notes);
    json j = {{"teeth", o.gear.teeth}, {"module", o.gear.module}, {"profile", o.gear_profile}, {"pressure_angle_deg", o.gear.pressure_angle_deg}, {"tip_diameter", 2 * o.gear.tip_radius()},
              {"pitch_diameter", 2 * o.gear.pitch_radius()}, {"root_diameter", 2 * o.gear.root_radius()}, {"base_diameter", o.gear.profile == gear::Profile::Involute ? 2 * o.gear.base_radius() : 0.0},
              {"thickness", o.thickness}, {"outputs", json::array()}};
    if (!o.out_step.empty()) {
        manufacturing::export_gear_step(o.gear, o.thickness, o.name, o.out_step);
        j["outputs"].push_back(o.out_step);
    }
    const report::Drawing drawing = o.profile_only ? manufacturing::gear_profile(o.gear) : manufacturing::gear_drawing(o.gear, notes);
    if (!o.out_dxf.empty()) {
        write_text(o.out_dxf, report::to_dxf(drawing));
        j["outputs"].push_back(o.out_dxf);
    }
    if (!o.out_svg.empty()) {
        write_text(o.out_svg, report::to_svg(drawing));
        j["outputs"].push_back(o.out_svg);
    }
    if (!o.out_pdf.empty()) {
        write_text(o.out_pdf, report::drawing_to_pdf(drawing, o.name, info));
        j["outputs"].push_back(o.out_pdf);
    }
    if (as_json) {
        std::cout << j.dump(2) << '\n';
    } else {
        for (const auto& l : info) std::cout << l << '\n';
        for (const auto& f : j["outputs"]) std::cout << "wrote " << f.get<std::string>() << '\n';
    }
    return kExitOk;
}

int cmd_draw(const MfgOptions& o, bool as_json) {
    const model::Movement m = model::load_rig(o.project);
    const int ci = m.find_component(o.part);
    if (ci < 0) throw Error("no part called '" + o.part + "' in the project");
    const auto& c = m.components[static_cast<std::size_t>(ci)];
    fs::path step = o.step_override;
    if (step.empty()) step = fs::path(o.project).parent_path() / m.source_geometry;
    const geometry::ImportedPart part = geometry::load_part(step, o.project, o.part, o.step_override.empty());  // this part only
    const geometry::ImportedPart* found = &part;
    manufacturing::PartDrawingInput in;
    in.title = o.part;
    in.thickness = c.features.thickness();
    in.material = o.material;
    in.teeth = c.features.teeth;
    in.module = c.gear ? c.gear->module : 0.0;
    in.region = geometry::section_region(geometry::orient_dial_axis_to_z(found->shape, axis_of(o.up, m)), 0.5 * (c.features.z_min + c.features.z_max), 0.002, 0.0015);
    const manufacturing::PartDrawing pd = manufacturing::make_part_drawing(in);
    if (o.out.empty()) throw Error("give -o with an .svg, .dxf or .pdf name");
    const std::string ext = extension(o.out);
    if (ext == "svg") write_text(o.out, report::to_svg(pd.drawing));
    else if (ext == "dxf") write_text(o.out, report::to_dxf(pd.drawing));
    else if (ext == "pdf") write_text(o.out, report::drawing_to_pdf(pd.drawing, o.part, pd.info));
    else throw Error("the drawing file must end in .svg, .dxf or .pdf");
    if (as_json) {
        json j = {{"part", o.part}, {"output", o.out}, {"width", pd.width}, {"height", pd.height}, {"holes", json::array()}};
        for (const auto& h : pd.holes) j["holes"].push_back({{"x", h.x}, {"y", h.y}, {"diameter", h.diameter}, {"round", h.round}});
        std::cout << j.dump(2) << '\n';
    } else {
        for (const auto& l : pd.info) std::cout << l << '\n';
        std::cout << "wrote " << o.out << '\n';
    }
    return kExitOk;
}

json stat_json(const manufacturing::Stat& s) { return {{"mean", s.mean}, {"sd", s.sd}, {"min", s.min}, {"max", s.max}, {"p01", s.p01}, {"p99", s.p99}}; }

int cmd_tolerance(const MfgOptions& o, bool as_json) {
    const model::Movement m = model::load_rig(o.project);
    manufacturing::ToleranceConfig cfg = o.tolerance_file.empty() ? manufacturing::ToleranceConfig{} : manufacturing::load_tolerance(o.tolerance_file);
    std::vector<manufacturing::PmiApplied> pmi;
    if (o.pmi) {
        const auto parts_step = o.step_override.empty() ? fs::path(o.project).parent_path() / m.source_geometry : fs::path(o.step_override);
        pmi = manufacturing::apply_pmi(cfg, geometry::read_step_pmi(parts_step), m);
    }
    cfg.samples = o.samples;
    cfg.seed = o.seed;
    const auto r = manufacturing::run_tolerance(m, cfg);
    if (as_json) {
        json j = {{"samples", r.samples}, {"seed", r.seed}, {"failing_fraction", r.failing_fraction}, {"meshes", json::array()}, {"mobiles", json::array()}};
        j["pmi"] = json::array();
        for (const auto& a : pmi) j["pmi"].push_back({{"field", a.field}, {"value_mm", a.value}, {"source", a.source}});
        for (const auto& mt : r.meshes) {
            json sens = json::array();
            for (const auto& s : mt.backlash_sensitivity) sens.push_back({{"input", s.input}, {"share", s.share}, {"correlation", s.correlation}});
            j["meshes"].push_back({{"a", mt.a}, {"b", mt.b}, {"nominal_backlash", mt.nominal_backlash}, {"nominal_contact_ratio", mt.nominal_contact_ratio},
                                   {"backlash", stat_json(mt.backlash)}, {"contact_ratio", stat_json(mt.contact_ratio)}, {"centre_distance", stat_json(mt.centre_distance)},
                                   {"bind_fraction", mt.bind_fraction}, {"under_engage_fraction", mt.under_engage_fraction}, {"backlash_sensitivity", sens}});
        }
        for (const auto& mt : r.mobiles) {
            j["mobiles"].push_back({{"mobile", mt.mobile}, {"end_shake", stat_json(mt.end_shake)}, {"side_shake", stat_json(mt.side_shake)},
                                    {"end_shake_out_fraction", mt.end_shake_out_fraction}, {"side_shake_out_fraction", mt.side_shake_out_fraction}});
        }
        std::cout << j.dump(2) << '\n';
    } else {
        if (o.pmi) {
            std::cout << "tolerances from the STEP file's PMI: " << (pmi.empty() ? "none applied (no matching dimensions)" : "") << "\n";
            for (const auto& a : pmi) std::cout << "  " << a.field << " = +-" << a.value << " mm (" << a.source << ")\n";
        }
        std::cout << r.samples << " builds, seed " << r.seed << "\n" << std::fixed << std::setprecision(4);
        for (const auto& mt : r.meshes) {
            std::cout << mt.a << " / " << mt.b << ": backlash " << mt.backlash.mean << " +- " << mt.backlash.sd << " mm (nominal " << mt.nominal_backlash << "), contact ratio "
                      << std::setprecision(2) << mt.contact_ratio.mean << std::setprecision(4) << ", binds " << std::setprecision(2) << mt.bind_fraction * 100 << " %, under-engages "
                      << mt.under_engage_fraction * 100 << " %" << std::setprecision(4) << "\n";
            if (!mt.backlash_sensitivity.empty()) {
                std::cout << "    biggest contributors:";
                for (std::size_t k = 0; k < std::min<std::size_t>(2, mt.backlash_sensitivity.size()); ++k) {
                    std::cout << ' ' << mt.backlash_sensitivity[k].input << " (" << std::setprecision(0) << mt.backlash_sensitivity[k].share * 100 << " %)" << std::setprecision(4) << ';';
                }
                std::cout << '\n';
            }
        }
        for (const auto& mt : r.mobiles) {
            std::cout << mt.mobile << ": end shake " << mt.end_shake.mean << " +- " << mt.end_shake.sd << " mm, " << std::setprecision(2) << mt.end_shake_out_fraction * 100
                      << " % outside; side shake " << std::setprecision(4) << mt.side_shake.mean << " +- " << mt.side_shake.sd << " mm, " << std::setprecision(2)
                      << mt.side_shake_out_fraction * 100 << " % outside\n" << std::setprecision(4);
        }
        std::cout << std::setprecision(2) << r.failing_fraction * 100 << " % of builds have a problem\n";
    }
    return r.failing_fraction > o.fail_above ? kExitCheckFailed : kExitOk;
}

int cmd_dfm(const MfgOptions& o, bool as_json) {
    const auto extra = extra_rules(o);
    if (o.list) {
        std::vector<manufacturing::ProcessRules> all = extra;
        for (const auto& p : manufacturing::builtin_rule_sets()) all.push_back(p);
        json j = json::array();
        for (const auto& p : all) {
            j.push_back({{"name", p.name}, {"description", p.description}, {"rules", p.rules}});
            if (!as_json) std::cout << std::left << std::setw(16) << p.name << p.description << '\n';
        }
        if (as_json) std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    const model::Movement m = model::load_rig(o.project);
    const auto step_parts = load_parts(o, m);

    // One section per part, shared by every process and plugin.
    std::map<std::string, const geometry::ImportedPart*> by_name;
    for (const auto& p : step_parts) by_name[p.name] = &p;
    std::vector<manufacturing::PartGeometry> built;
    std::vector<TopoDS_Shape> shapes;
    std::vector<std::size_t> from;  // the component each geometry belongs to
    for (std::size_t ci = 0; ci < m.components.size(); ++ci) {
        const auto& c = m.components[ci];
        if (c.features.volume <= 0) continue;
        const auto found = by_name.find(c.features.name);
        if (found == by_name.end()) continue;
        manufacturing::PartGeometry g;
        g.name = c.features.name;
        g.role = c.role;
        g.thickness = c.features.thickness();
        g.teeth = c.features.teeth;
        g.module = c.gear ? c.gear->module : 0.0;
        if (c.gear && model::has_pressure_angle(*c.gear)) g.pressure_angle_deg = c.gear->pressure_angle_deg;
        if (c.gear && c.gear->profile == "cycloidal") g.rolling_radius = c.gear->rolling_radius;
        shapes.push_back(geometry::orient_dial_axis_to_z(found->second->shape, axis_of(o.up, m)));
        from.push_back(ci);
        built.push_back(std::move(g));
    }
    std::vector<char> ok(built.size(), 0);
    const std::string cad = o.step_override.empty() ? (fs::path(o.project).parent_path() / m.source_geometry).string() : o.step_override;
    geometry::SectionCache cache(o.project, cad, o.step_override.empty());
    const char axis_char = static_cast<char>('0' + static_cast<int>(axis_of(o.up, m)));
    geometry::parallel_for_shapes(shapes, [&](std::size_t k) {
        const auto& f = m.components[from[k]].features;
        try {
            built[k].region = cache.get(built[k].name, axis_char, shapes[k], 0.5 * (f.z_min + f.z_max), 0.002, 0.002);
            ok[k] = 1;
        } catch (const Error&) {
        }
    });
    std::vector<manufacturing::PartGeometry> parts;
    for (std::size_t k = 0; k < built.size(); ++k) {
        if (ok[k]) parts.push_back(std::move(built[k]));
    }

    int errors = 0, warnings = 0;
    json out = json::array();
    auto record = [&](const manufacturing::Finding& f) {
        errors += f.severity == manufacturing::Severity::Error;
        warnings += f.severity == manufacturing::Severity::Warning;
        out.push_back({{"process", f.process}, {"rule", f.rule}, {"part", f.part}, {"severity", to_string(f.severity)}, {"message", f.message}, {"fix", f.fix},
                       {"value", f.value}, {"limit", f.limit}});
        if (!as_json && f.severity != manufacturing::Severity::Info) {
            std::cout << to_string(f.severity) << ": [" << f.process << "/" << f.rule << "] " << f.part << ": " << f.message << (f.fix.empty() ? "" : " -> " + f.fix) << '\n';
        }
    };
    for (const std::string& pname : split(o.process)) {
        const auto& rules = manufacturing::find_process(pname, extra);
        for (const auto& g : parts) {
            for (const auto& f : manufacturing::run_dfm(rules, g)) record(f);
        }
    }
    for (const std::string& path : o.plugins) {
        const auto plugin = plugins::DfmPlugin::load(path);
        for (const auto& g : parts) {
            for (const auto& f : plugin->check(g)) record(f);
        }
    }
    if (as_json) std::cout << json{{"errors", errors}, {"warnings", warnings}, {"findings", out}}.dump(2) << '\n';
    else std::cout << out.size() << " checks, " << errors << " error(s), " << warnings << " warning(s)\n";
    return errors > 0 ? kExitCheckFailed : kExitOk;
}

int cmd_collisions(const MfgOptions& o, bool as_json) {
    const model::Movement m = model::load_rig(o.project);
    const double vph = o.beat_rate > 0 ? o.beat_rate : m.beat_rate_vph;
    const auto train = kinematics::analyze_train(m, vph);
    const std::string cad = o.step_override.empty() ? (fs::path(o.project).parent_path() / m.source_geometry).string() : o.step_override;
    geometry::SectionCache cache(o.project, cad, o.step_override.empty());
    const auto parts = manufacturing::collision_parts(m, load_parts(o, m), axis_of(o.up, m), 0.003, &cache);
    const auto r = manufacturing::sweep_collisions(m, train, parts);
    if (as_json) {
        json j = {{"pairs_checked", r.pairs_checked}, {"overlaps", r.overlaps}, {"close_calls", r.close_calls}, {"pairs", json::array()}};
        for (const auto& p : r.pairs) {
            j["pairs"].push_back({{"a", p.a}, {"b", p.b}, {"meshing", p.meshing}, {"steps", p.steps}, {"overlapping_steps", p.overlapping_steps}, {"min_clearance", p.min_clearance}, {"at_degrees", p.at_degrees}});
        }
        std::cout << j.dump(2) << '\n';
    } else {
        for (const auto& p : r.pairs) {
            std::cout << std::left << std::setw(36) << (p.a + " / " + p.b) << std::right << (p.meshing ? " mesh  " : " sweep ") << std::fixed << std::setprecision(4) << std::setw(9) << p.min_clearance
                      << " mm closest" << (p.overlapping_steps ? "  OVERLAP in " + std::to_string(p.overlapping_steps) + " of " + std::to_string(p.steps) + " poses" : "") << '\n';
        }
        std::cout << r.pairs_checked << " pairs swept: " << r.overlaps << " overlap(s), " << r.close_calls << " close call(s)\n";
    }
    return r.overlaps > 0 ? kExitCheckFailed : kExitOk;
}

int cmd_report(const MfgOptions& o, bool as_json) {
    const model::Movement m = model::load_rig(o.project);
    reports::ReportOptions ro;
    ro.cache_project = o.project;
    ro.update_cache = o.step_override.empty();
    ro.step_file = o.step_override.empty() ? (fs::path(o.project).parent_path() / m.source_geometry).string() : o.step_override;
    if (!fs::exists(ro.step_file)) ro.step_file.clear();
    ro.up = axis_of(o.up, m);
    ro.processes = split(o.process);
    ro.extra_rules = extra_rules(o);
    ro.plugins = o.plugins;
    ro.tolerance_config = o.tolerance_file.empty() ? manufacturing::ToleranceConfig{} : manufacturing::load_tolerance(o.tolerance_file);
    ro.tolerance_config.samples = o.samples;
    ro.tolerance_config.seed = o.seed;
    if (!o.calibre_file.empty()) ro.calibre = dynamics::load_calibre(o.calibre_file);
    ro.structural = !o.no_fea;
    ro.timekeeping = !o.no_timekeeping;
    ro.beat_rate_vph = o.beat_rate;
    const report::Document doc = reports::build_report(m, ro);
    const std::string prefix = o.out.empty() ? (fs::path(o.project).replace_extension("").string() + "-report") : o.out;
    json written = json::array();
    for (const std::string& f : split(o.formats)) {
        if (f == "html") {
            write_text(prefix + ".html", report::to_html(doc));
            written.push_back(prefix + ".html");
        } else if (f == "pdf") {
            write_text(prefix + ".pdf", report::to_pdf(doc));
            written.push_back(prefix + ".pdf");
        } else {
            throw Error("unknown report format '" + f + "' (html or pdf)");
        }
    }
    const report::Status overall = doc.overall();
    if (as_json) std::cout << json{{"outputs", written}, {"overall", report::status_word(overall)}}.dump(2) << '\n';
    else {
        for (const auto& w : written) std::cout << "wrote " << w.get<std::string>() << '\n';
        std::cout << "overall: " << report::status_word(overall) << '\n';
    }
    return overall == report::Status::Fail ? kExitCheckFailed : kExitOk;
}

}  // namespace

void add_manufacturing_commands(CLI::App& app, const bool& json_flag, std::function<int()>& chosen) {
    auto o = std::make_shared<MfgOptions>();
    auto pick = [&chosen, &json_flag, o](int (*fn)(const MfgOptions&, bool)) {
        return [&chosen, &json_flag, o, fn] { chosen = [&json_flag, o, fn] { return fn(*o, json_flag); }; };
    };
    auto project = [o](CLI::App* cmd, bool required = true) {
        auto* p = cmd->add_option("project", o->project, "Project file");
        if (required) p->required()->check(CLI::ExistingFile);
        cmd->add_option("--step", o->step_override, "STEP file (default: next to the project)");
        cmd->add_option("--up", o->up, "CAD axis normal to the dial (default: the one found at import)");
    };

    auto* g = app.add_subcommand("gear", "Generate an involute or cycloidal wheel as STEP, DXF, SVG or PDF drawing");
    g->add_option("--teeth", o->gear.teeth, "Number of teeth")->required();
    g->add_option("--module", o->gear.module, "Module in mm")->required();
    g->add_option("--profile", o->gear_profile, "Tooth profile: involute or cycloidal")->capture_default_str();
    g->add_option("--pressure-angle", o->gear.pressure_angle_deg, "Pressure angle in degrees (involute)")->capture_default_str();
    g->add_option("--rolling-radius", o->gear.rolling_radius, "Rolling circle radius in modules (cycloidal; wheels that mesh share it)")->capture_default_str();
    g->add_option("--addendum", o->gear.addendum, "Addendum in modules")->capture_default_str();
    g->add_option("--dedendum", o->gear.dedendum, "Dedendum in modules")->capture_default_str();
    g->add_option("--fillet", o->gear.root_fillet, "Root fillet radius in modules")->capture_default_str();
    g->add_option("--bore", o->gear.bore_radius, "Bore radius in mm")->capture_default_str();
    g->add_option("--thickness", o->thickness, "Wheel thickness in mm")->capture_default_str();
    g->add_option("--material", o->material, "Material note for the drawing")->capture_default_str();
    g->add_option("--name", o->name, "Part name")->capture_default_str();
    g->add_option("--step-out", o->out_step, "Write a STEP solid");
    g->add_option("--dxf", o->out_dxf, "Write a DXF drawing");
    g->add_option("--svg", o->out_svg, "Write an SVG drawing");
    g->add_option("--pdf", o->out_pdf, "Write a PDF drawing at a standard scale");
    g->add_flag("--profile-only", o->profile_only, "DXF/SVG/PDF carry only the outline, without views and dimensions (for cutting set-up)");
    g->callback(pick(cmd_gear));

    auto* d = app.add_subcommand("draw", "Dimensioned drawing of one part of a project (SVG, DXF or PDF)");
    project(d);
    d->add_option("--part", o->part, "Part name")->required();
    d->add_option("-o,--output", o->out, "File ending in .svg, .dxf or .pdf");
    d->add_option("--material", o->material, "Material note")->capture_default_str();
    d->callback(pick(cmd_draw));

    auto* t = app.add_subcommand("tolerance", "Monte Carlo tolerance stack-up: backlash, engagement, end and side shake");
    project(t);
    t->add_option("--samples", o->samples, "Number of virtual builds")->capture_default_str();
    t->add_option("--seed", o->seed, "Random seed")->capture_default_str();
    t->add_flag("--pmi", o->pmi, "Take pivot, hole, arbor-length and bridge-gap tolerances from the STEP file's PMI (AP242 dimensional tolerances), on top of the table");
    t->add_option("--tolerances", o->tolerance_file, "Tolerance table TOML ([tolerance])")->check(CLI::ExistingFile);
    t->add_option("--fail-above", o->fail_above, "Exit 1 when more than this fraction of builds has a problem")->capture_default_str();
    t->callback(pick(cmd_tolerance));

    auto* f = app.add_subcommand("dfm", "Design-for-manufacturing rules for your processes");
    f->add_option("project", o->project, "Project file")->check(CLI::ExistingFile);
    f->add_option("--step", o->step_override, "STEP file (default: next to the project)");
    f->add_option("--up", o->up, "CAD axis normal to the dial (default: the one found at import)");
    f->add_option("--process", o->process, "Comma-separated processes (ohs dfm --list)")->capture_default_str();
    f->add_option("--rules", o->rules_file, "Your own rule sets (TOML, same format as data/dfm/rules.toml)")->check(CLI::ExistingFile);
    f->add_flag("--list", o->list, "List the processes and exit");
    f->add_option("--plugin", o->plugins, "A DFM plugin (shared library, see docs/plugins.md); repeatable")->check(CLI::ExistingFile);
    f->callback([&chosen, &json_flag, o] {
        chosen = [&json_flag, o] {
            if (!o->list && o->project.empty()) throw Error("give a project file, or --list");
            return cmd_dfm(*o, json_flag);
        };
    });

    auto* c = app.add_subcommand("collisions", "Sweep the moving train for overlaps and close approaches");
    project(c);
    c->add_option("--beat-rate", o->beat_rate, "Beat rate in vph (default: from project)");
    c->callback(pick(cmd_collisions));

    auto* r = app.add_subcommand("report", "Write an HTML and PDF report of the analyses");
    project(r);
    r->add_option("-o,--output", o->out, "Output path without extension (default: next to the project)");
    r->add_option("--format", o->formats, "html, pdf or html,pdf")->capture_default_str();
    r->add_option("--process", o->process, "Processes for the DFM section")->capture_default_str();
    r->add_option("--rules", o->rules_file, "Your own DFM rule sets")->check(CLI::ExistingFile);
    r->add_option("--plugin", o->plugins, "A DFM plugin library; repeatable")->check(CLI::ExistingFile);
    r->add_option("--tolerances", o->tolerance_file, "Tolerance table TOML")->check(CLI::ExistingFile);
    r->add_option("--calibre", o->calibre_file, "Calibre parameters TOML")->check(CLI::ExistingFile);
    r->add_option("--samples", o->samples, "Tolerance samples")->capture_default_str();
    r->add_option("--seed", o->seed, "Tolerance seed")->capture_default_str();
    r->add_option("--beat-rate", o->beat_rate, "Beat rate in vph (default: from project)");
    r->add_flag("--no-fea", o->no_fea, "Skip the structural section");
    r->add_flag("--no-timekeeping", o->no_timekeeping, "Skip the timekeeping section");
    r->callback(pick(cmd_report));
}

}  // namespace ohs::cli
