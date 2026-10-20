// SPDX-License-Identifier: Apache-2.0
#include "reports/report_builder.hpp"

#include <map>
#include <exception>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "core/parallel.hpp"
#include "plugins/plugin_loader.hpp"
#include "dynamics/timekeeping.hpp"
#include "dynamics/torque_chain.hpp"
#include "explain/explain.hpp"
#include "geometry/features.hpp"
#include "geometry/section.hpp"
#include "geometry/shape_groups.hpp"
#include "geometry/cad_io.hpp"
#include "geometry/part_cache.hpp"
#include "kinematics/kinematics.hpp"
#include "manufacturing/collision.hpp"
#include "rigging/rigging.hpp"

namespace ohs::reports {

namespace {

using report::Status;

std::string num(double v, int prec = 2) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

std::string pct(double fraction) { return num(fraction * 100.0, 2) + " %"; }

Status worse(Status a, Status b) { return static_cast<int>(a) >= static_cast<int>(b) ? a : b; }

struct Summary {
    std::vector<std::vector<std::string>> rows;
    std::vector<Status> status;
    void add(const std::string& section, Status s, const std::string& text) {
        rows.push_back({section, text});
        status.push_back(s);
    }
};

}  // namespace

report::Document build_report(const model::Movement& m, const ReportOptions& o) {
    if (m.components.empty()) throw Error("the project has no parts to report on");
    report::Document doc;
    doc.title = "Movement analysis: " + m.name;
    doc.subtitle = "Open Horology Studio report";
    Summary summary;
    std::vector<report::Block> body;  // sections are collected, then placed after the summary

    auto section = [&](auto&& fill) {
        report::Document part;
        fill(part);
        for (auto& b : part.blocks) body.push_back(std::move(b));
    };

    const double vph = o.beat_rate_vph > 0 ? o.beat_rate_vph : m.beat_rate_vph;
    kinematics::TrainAnalysis train;
    bool have_train = false;
    try {
        train = kinematics::analyze_train(m, vph);
        have_train = true;
    } catch (const Error&) {
    }

    // ---- rig and train ---------------------------------------------------------------------------------------------
    section([&](report::Document& d) {
        d.heading("Rig and gear train");
        const auto issues = rigging::check_rig(m);
        int errors = 0, warnings = 0;
        for (const auto& i : issues) {
            errors += i.severity == rigging::Severity::Error;
            warnings += i.severity == rigging::Severity::Warning;
        }
        std::vector<std::vector<std::string>> rows;
        std::vector<Status> st;
        for (const auto& c : m.components) {
            rows.push_back({c.features.name, std::string(to_string(c.role)), num(c.role_confidence), c.features.teeth ? std::to_string(c.features.teeth) : "-"});
            st.push_back(c.role_confidence >= 0.6 ? Status::Pass : Status::Warn);
        }
        d.table({"part", "role", "confidence", "teeth"}, rows, st);
        for (const auto& i : issues) d.paragraph(std::string(i.severity == rigging::Severity::Error ? "Error: " : i.severity == rigging::Severity::Warning ? "Warning: " : "Note: ") + i.message);
        summary.add("Rig", errors ? Status::Fail : warnings ? Status::Warn : Status::Pass,
                    std::to_string(m.components.size()) + " parts, " + std::to_string(m.mobiles.size()) + " mobiles, " + std::to_string(m.meshes.size()) + " meshes; " +
                        std::to_string(errors) + " errors, " + std::to_string(warnings) + " warnings");
        if (have_train) {
            const auto walk = explain::walk_train(m, train);
            std::vector<std::vector<std::string>> steps;
            for (const auto& s : walk.steps) {
                steps.push_back({s.driver + " -> " + s.driven, std::to_string(s.driver_teeth) + ":" + std::to_string(s.driven_teeth), num(s.step_ratio, 4), num(s.cumulative_ratio, 4),
                                 s.revs_per_hour > 0 ? num(s.revs_per_hour, 4) : "-"});
            }
            d.heading("Gear train", 3);
            d.table({"mesh", "teeth", "step ratio", "cumulative", "revolutions per hour"}, steps);
            for (const auto& l : walk.conclusion) d.paragraph(l);
        } else {
            d.callout(Status::Fail, "The train could not be analysed: there is no barrel or the escape wheel is not connected.");
        }
    });

    // ---- timekeeping -------------------------------------------------------------------------------------------------
    if (o.timekeeping && have_train) {
        section([&](report::Document& d) {
            d.heading("Timekeeping");
            try {
                dynamics::Calibre cal = o.calibre ? *o.calibre : dynamics::default_calibre(vph > 0 ? vph : 25200);
                cal.beat_rate_vph = vph > 0 ? vph : cal.beat_rate_vph;
                const auto chain = dynamics::build_chain(m, train, cal);
                dynamics::adopt_train(cal, chain, cal.beat_rate_vph);
                const dynamics::Timekeeping t = dynamics::evaluate(cal, chain);
                d.paragraph("Level-2 reduced model with the calibre parameters " + std::string(o.calibre ? "from the calibre file" : "of the reference lever calibre") +
                            ". Replace them with your movement's values (ohs calibre init) before trusting absolute numbers.");
                d.key_values({{"amplitude, fully wound, dial up", num(t.full.amplitude_deg, 1) + " deg"},
                              {"amplitude, half wound", num(t.half.amplitude_deg, 1) + " deg"},
                              {"amplitude, 10 % wound", num(t.low.amplitude_deg, 1) + " deg"},
                              {"rate, half wound, dial up", num(t.half.rate_s_per_day, 1) + " s/day"},
                              {"isochronism", num(t.isochronism_s_per_day_per_deg, 3) + " s/day per degree"},
                              {"positional spread", num(t.positions.spread_s_per_day, 1) + " s/day"},
                              {"power reserve", num(t.power_reserve_h, 1) + " h of " + num(t.full_wind_h, 1) + " h of spring"},
                              {"energy reaching the balance", num(t.budget.efficiency * 100, 0) + " % of the escape wheel's work"}});
                std::vector<report::Series> series;
                for (dynamics::Position pos : {dynamics::Position::DialUp, dynamics::Position::CrownUp}) {
                    report::Series s;
                    s.name = dynamics::to_string(pos);
                    for (const auto& p : dynamics::amplitude_vs_wind(cal, chain, 6, pos)) s.points.push_back({p.hours, p.amplitude_deg});
                    series.push_back(std::move(s));
                }
                d.figure(report::line_chart("Amplitude as the spring unwinds", "hours since full wind", "amplitude, deg", series), "Steady-state amplitude of the balance against running time.");
                std::vector<std::vector<std::string>> rows;
                for (const auto& p : t.positions.points) rows.push_back({dynamics::to_string(p.position), num(p.amplitude_deg, 1), num(p.rate_s_per_day, 1), num(p.beat_error_ms, 2)});
                d.table({"position (half wound)", "amplitude, deg", "rate, s/day", "beat error, ms"}, rows);
                Status s = Status::Pass;
                std::string text = "power reserve " + num(t.power_reserve_h, 1) + " h";
                if (t.power_reserve_h < o.target_power_reserve_h) {
                    s = Status::Fail;
                    text += " is below the " + num(o.target_power_reserve_h, 0) + " h target";
                }
                if (t.full.amplitude_deg > 320) {
                    s = worse(s, Status::Warn);
                    text += "; amplitude above 320 deg risks knocking";
                }
                d.callout(s, text);
                summary.add("Timekeeping", s, text);
            } catch (const Error& e) {
                d.callout(Status::Fail, std::string("Timekeeping could not be analysed: ") + e.what());
                summary.add("Timekeeping", Status::Fail, e.what());
            }
        });
    }

    // ---- tolerance -------------------------------------------------------------------------------------------------------
    if (o.tolerance && !m.meshes.empty()) {
        section([&](report::Document& d) {
            d.heading("Tolerance stack-up");
            try {
                const auto r = manufacturing::run_tolerance(m, o.tolerance_config);
                d.paragraph(std::to_string(r.samples) + " virtual builds, seed " + std::to_string(r.seed) + ". Every dimension is a random variable of +-tol at three standard deviations (position " +
                            num(o.tolerance_config.position_tol, 3) + " mm, tooth thickness " + num(o.tolerance_config.tooth_thickness_tol, 3) + " mm).");
                std::vector<std::vector<std::string>> mesh_rows;
                std::vector<Status> mesh_status;
                for (const auto& mt : r.meshes) {
                    mesh_rows.push_back({mt.a + " / " + mt.b, num(mt.backlash.mean, 4) + " +- " + num(mt.backlash.sd, 4), num(mt.contact_ratio.mean, 2), pct(mt.bind_fraction),
                                         pct(mt.under_engage_fraction), mt.backlash_sensitivity.empty() ? "-" : mt.backlash_sensitivity.front().input});
                    mesh_status.push_back(mt.bind_fraction > 0 || mt.under_engage_fraction > 0 ? (mt.bind_fraction + mt.under_engage_fraction > o.max_failing_fraction ? Status::Fail : Status::Warn) : Status::Pass);
                }
                d.table({"mesh", "backlash, mm", "contact ratio", "binds", "under-engages", "biggest contributor"}, mesh_rows, mesh_status);
                std::vector<std::vector<std::string>> mob_rows;
                std::vector<Status> mob_status;
                for (const auto& mt : r.mobiles) {
                    mob_rows.push_back({mt.mobile, num(mt.end_shake.mean, 3) + " +- " + num(mt.end_shake.sd, 3), pct(mt.end_shake_out_fraction), num(mt.side_shake.mean, 3) + " +- " + num(mt.side_shake.sd, 3),
                                        pct(mt.side_shake_out_fraction)});
                    const double out = mt.end_shake_out_fraction + mt.side_shake_out_fraction;
                    mob_status.push_back(out == 0 ? Status::Pass : out > o.max_failing_fraction ? Status::Fail : Status::Warn);
                }
                d.table({"mobile", "end shake, mm", "outside window", "side shake, mm", "outside window"}, mob_rows, mob_status);
                const Status s = r.failing_fraction == 0 ? Status::Pass : r.failing_fraction > o.max_failing_fraction ? Status::Fail : Status::Warn;
                const std::string text = pct(r.failing_fraction) + " of builds bind, under-engage or leave a shake window";
                d.callout(s, text);
                summary.add("Tolerances", s, text);
            } catch (const Error& e) {
                d.callout(Status::Fail, std::string("Tolerance analysis failed: ") + e.what());
                summary.add("Tolerances", Status::Fail, e.what());
            }
        });
    }

    // ---- geometry-dependent sections ----------------------------------------------------------------------------------------
    std::vector<geometry::ImportedPart> step_parts;
    std::string geometry_note;
    if (!o.step_file.empty()) {
        try {
            step_parts = geometry::load_parts(o.step_file, o.cache_project, o.update_cache);
        } catch (const Error& e) {
            geometry_note = e.what();
        }
    } else {
        geometry_note = "no STEP file was given";
    }

    // The mid-height section of every part, computed once and shared by the DFM and collision sections.
    struct Section {
        int component;
        manufacturing::PartGeometry geometry;
    };
    std::vector<Section> section_cache;
    bool sections_built = false;
    auto sections = [&]() -> const std::vector<Section>& {
        if (sections_built) return section_cache;
        sections_built = true;
        std::map<std::string, const geometry::ImportedPart*> by_name;
        for (const auto& p : step_parts) by_name[p.name] = &p;
        std::vector<Section> built;
        std::vector<TopoDS_Shape> shapes;
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
            shapes.push_back(geometry::orient_dial_axis_to_z(found->second->shape, o.up));
            built.push_back({static_cast<int>(ci), std::move(g)});
        }
        // The sections are the slow part and independent of each other: on threads, each into its own slot.
        std::vector<char> ok(built.size(), 0);
        geometry::SectionCache cache(o.cache_project, o.step_file, o.update_cache);
        geometry::parallel_for_shapes(shapes, [&](std::size_t k) {
            const auto& f = m.components[static_cast<std::size_t>(built[k].component)].features;
            try {
                built[k].geometry.region = cache.get(built[k].geometry.name, static_cast<char>('0' + static_cast<int>(o.up)), shapes[k],
                                                     0.5 * (f.z_min + f.z_max), 0.001, 0.002);
                ok[k] = 1;
            } catch (const Error&) {
            }
        });
        for (std::size_t k = 0; k < built.size(); ++k) {
            if (ok[k]) section_cache.push_back(std::move(built[k]));
        }
        return section_cache;
    };

    if (o.dfm) {
        section([&](report::Document& d) {
            d.heading("Design for manufacturing");
            if (step_parts.empty()) {
                d.callout(Status::Warn, "Skipped: " + geometry_note + ".");
                summary.add("Manufacturing", Status::Warn, "skipped (" + geometry_note + ")");
                return;
            }
            Status overall = Status::Pass;
            int errors = 0, warnings = 0, checks = 0;
            for (const std::string& pname : o.processes) {
                const manufacturing::ProcessRules* rules = nullptr;
                try {
                    rules = &manufacturing::find_process(pname, o.extra_rules);
                } catch (const Error& e) {
                    d.callout(Status::Fail, e.what());
                    overall = Status::Fail;
                    continue;
                }
                d.heading("Process: " + rules->name, 3);
                d.paragraph(rules->description);
                std::vector<std::vector<std::string>> rows;
                std::vector<Status> st;
                // The rules are pure functions of the outlines, so the parts are checked on threads; the
                // findings are collected in part order, so the report does not depend on scheduling.
                std::vector<std::vector<manufacturing::Finding>> per_part(sections().size());
                ohs::parallel_for(per_part.size(), [&](std::size_t i) { per_part[i] = manufacturing::run_dfm(*rules, sections()[i].geometry); });
                for (const auto& findings : per_part) {
                    for (const auto& f : findings) {
                        ++checks;
                        errors += f.severity == manufacturing::Severity::Error;
                        warnings += f.severity == manufacturing::Severity::Warning;
                        if (f.severity == manufacturing::Severity::Info) continue;  // only problems are listed; the count says how many passed
                        rows.push_back({f.part, f.rule, f.message, f.fix});
                        st.push_back(f.severity == manufacturing::Severity::Error ? Status::Fail : Status::Warn);
                    }
                }
                if (rows.empty()) d.callout(Status::Pass, "No findings for this process.");
                else d.table({"part", "rule", "finding", "fix"}, rows, st);
            }
            for (const std::string& path : o.plugins) {
                try {
                    const auto plugin = plugins::DfmPlugin::load(path);
                    d.heading("Plugin: " + plugin->name(), 3);
                    d.paragraph(plugin->description() + " (version " + plugin->version() + ")");
                    std::vector<std::vector<std::string>> rows;
                    std::vector<Status> st;
                    for (const auto& sec : sections()) {
                        for (const auto& f : plugin->check(sec.geometry)) {
                            ++checks;
                            errors += f.severity == manufacturing::Severity::Error;
                            warnings += f.severity == manufacturing::Severity::Warning;
                            if (f.severity == manufacturing::Severity::Info) continue;
                            rows.push_back({f.part, f.rule, f.message, f.fix});
                            st.push_back(f.severity == manufacturing::Severity::Error ? Status::Fail : Status::Warn);
                        }
                    }
                    if (rows.empty()) d.callout(Status::Pass, "No findings from this plugin.");
                    else d.table({"part", "rule", "finding", "fix"}, rows, st);
                } catch (const Error& e) {
                    d.callout(Status::Fail, e.what());
                    overall = Status::Fail;
                    ++errors;
                }
            }
            overall = errors ? Status::Fail : warnings ? Status::Warn : overall;
            const std::string text = std::to_string(checks) + " checks: " + std::to_string(errors) + " errors, " + std::to_string(warnings) + " warnings";
            d.callout(overall, text);
            summary.add("Manufacturing", overall, text);
        });
    }

    if (o.collisions && have_train) {
        section([&](report::Document& d) {
            d.heading("Motion and collisions");
            if (step_parts.empty()) {
                d.callout(Status::Warn, "Skipped: " + geometry_note + ".");
                summary.add("Collisions", Status::Warn, "skipped (" + geometry_note + ")");
                return;
            }
            try {
                std::vector<manufacturing::CollisionPart> parts;
                for (const auto& sec : sections()) {
                    const auto& c = m.components[static_cast<std::size_t>(sec.component)];
                    parts.push_back({c.features.name, sec.geometry.region, c.features.z_min, c.features.z_max, m.mobile_of(sec.component)});
                }
                const auto r = manufacturing::sweep_collisions(m, train, parts);
                std::vector<std::vector<std::string>> rows;
                std::vector<Status> st;
                for (const auto& p : r.pairs) {
                    rows.push_back({p.a + " / " + p.b, p.meshing ? "mesh" : "sweep", num(p.min_clearance, 4), std::to_string(p.overlapping_steps) + " of " + std::to_string(p.steps)});
                    st.push_back(p.overlapping_steps > 0 ? Status::Fail : p.min_clearance < 0.02 ? Status::Warn : Status::Pass);
                }
                if (!rows.empty()) d.table({"pair", "sweep", "closest approach, mm", "overlapping poses"}, rows, st);
                const Status s = r.overlaps ? Status::Fail : r.close_calls ? Status::Warn : Status::Pass;
                const std::string text = std::to_string(r.pairs_checked) + " pairs swept: " + std::to_string(r.overlaps) + " overlaps, " + std::to_string(r.close_calls) + " closer than 0.02 mm";
                d.callout(s, text);
                summary.add("Collisions", s, text);
            } catch (const Error& e) {
                d.callout(Status::Fail, std::string("Collision sweep failed: ") + e.what());
                summary.add("Collisions", Status::Fail, e.what());
            }
        });
    }

    // ---- structural ----------------------------------------------------------------------------------------------------------------
    if (o.structural) {
        section([&](report::Document& d) {
            d.heading("Structural checks");
            Status overall = Status::Pass;
            std::vector<std::vector<std::string>> rows;
            std::vector<Status> st;
            if (have_train) {
                try {
                    dynamics::Calibre cal = o.calibre ? *o.calibre : dynamics::default_calibre(vph > 0 ? vph : 25200);
                    const auto chain = dynamics::build_chain(m, train, cal);
                    const auto torques = dynamics::torque_table(chain, cal, 1.0);
                    struct Job {
                        std::size_t component;
                        cases::ToothCaseInput input;
                        std::optional<cases::ToothCaseResult> result;
                        std::string error;
                    };
                    std::vector<Job> jobs;
                    for (std::size_t ci = 0; ci < m.components.size(); ++ci) {
                        const auto& c = m.components[ci];
                        if (c.features.teeth < 6) continue;
                        const int mi = m.mobile_of(static_cast<int>(ci));
                        if (mi < 0) continue;
                        double torque = 0;
                        for (const auto& row : torques) {
                            if (row.mobile == m.mobiles[static_cast<std::size_t>(mi)].name) torque = row.torque_nmm;
                        }
                        if (torque <= 0) continue;
                        Job job;
                        job.component = ci;
                        job.input.spec.teeth = c.features.teeth;
                        job.input.spec.module = c.gear && c.gear->module > 0 ? c.gear->module : model::module_from_tip(c.features.outer_radius, c.features.teeth);
                        if (c.gear && model::has_pressure_angle(*c.gear)) job.input.spec.pressure_angle_deg = c.gear->pressure_angle_deg;
                        if (c.gear && c.gear->profile == "cycloidal") {
                            job.input.spec.profile = gear::Profile::Cycloidal;
                            job.input.spec.rolling_radius = c.gear->rolling_radius;
                        }
                        job.input.spec.bore_radius = c.features.bore_radius;
                        job.input.face_width = c.features.thickness();
                        job.input.tangential_load_n = torque / job.input.spec.pitch_radius();
                        job.input.material = fea::find_material(c.role == model::Role::Pinion ? "steel_hardened" : "brass_leaded");
                        jobs.push_back(std::move(job));
                    }
                    // Each wheel is an independent finite-element model: solve them on threads.
                    ohs::parallel_for(jobs.size(), [&](std::size_t k) {
                        try {
                            jobs[k].result = cases::run_tooth_case(jobs[k].input);
                        } catch (const Error& e) {
                            jobs[k].error = e.what();
                        }
                    });
                    for (const Job& job : jobs) {
                        const auto& c = m.components[job.component];
                        if (job.result) {
                            const auto& r = *job.result;
                            const Status s = r.safety_factor < 1.0 ? Status::Fail : r.safety_factor < o.min_safety_factor ? Status::Warn : Status::Pass;
                            overall = worse(overall, s);
                            rows.push_back({c.features.name, job.input.material.name, num(job.input.tangential_load_n, 4) + " N", num(r.lewis_mpa, 2) + " MPa",
                                            num(r.fe_max_principal_mpa, 2) + " MPa", num(r.concentration, 2), num(r.safety_factor, 0)});
                            st.push_back(s);
                        } else {
                            rows.push_back({c.features.name, job.input.material.name, num(job.input.tangential_load_n, 4) + " N", "-", "-", "-", job.error});
                            st.push_back(Status::Warn);
                            overall = worse(overall, Status::Warn);
                        }
                    }
                } catch (const Error& e) {
                    d.paragraph(std::string("Tooth loads could not be derived from the torque chain: ") + e.what());
                    overall = worse(overall, Status::Warn);
                }
            }
            if (!rows.empty()) {
                d.heading("Tooth bending at full wind", 3);
                d.paragraph("Tip load = torque of the mobile over the pitch radius; the FE peak is the largest principal stress of a three-tooth sector clamped at its rim. The Lewis hand calculation ignores the root fillet, so the FE value is higher by the stress concentration.");
                d.table({"wheel", "material", "tip load", "Lewis", "FE peak", "FE / Lewis", "safety factor"}, rows, st);
            }
            const auto checks = cases::run_physics_checks(false);
            int failed = 0;
            std::vector<std::vector<std::string>> vrows;
            std::vector<Status> vst;
            for (const auto& c : checks) {
                failed += !c.passed;
                vrows.push_back({c.name, num(c.expected, 4), num(c.actual, 4), pct(c.tolerance)});
                vst.push_back(c.passed ? Status::Pass : Status::Fail);
            }
            d.heading("Solver verification", 3);
            d.table({"check", "expected", "result", "tolerance"}, vrows, vst);
            if (failed) overall = Status::Fail;
            const std::string text = std::to_string(rows.size()) + " wheels checked; solver verification " + (failed ? std::to_string(failed) + " failed" : "passed");
            d.callout(overall, text);
            summary.add("Structure", overall, text);
        });
    }

    // ---- assemble ----------------------------------------------------------------------------------------------------------------------
    doc.heading("Summary");
    doc.table({"section", "result"}, summary.rows, summary.status);
    for (auto& b : body) doc.blocks.push_back(std::move(b));
    doc.notice = "Generated by Open Horology Studio (Apache-2.0).";
    if (m.source_geometry.find("om10") != std::string::npos || m.name.find("om10") != std::string::npos || m.name.find("OM10") != std::string::npos) {
        doc.notice += " This report is derived from the openmovement OM10 plans, licensed CC BY-SA 3.0: share it alike, with attribution to openmovement (openmovement.org).";
    }
    return doc;
}

}  // namespace ohs::reports
