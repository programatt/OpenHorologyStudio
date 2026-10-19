// SPDX-License-Identifier: Apache-2.0
#include "cli/dynamics_cli.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "dynamics/bench.hpp"
#include "dynamics/calibre.hpp"
#include "dynamics/hairspring.hpp"
#include "dynamics/chrono_adapter.hpp"
#include "dynamics/detailed.hpp"
#include "dynamics/oscillator.hpp"
#include "dynamics/study.hpp"
#include "dynamics/timegrapher.hpp"
#include "dynamics/timekeeping.hpp"
#include "dynamics/torque_chain.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"

using nlohmann::json;
using namespace ohs::dynamics;

namespace ohs::cli {

namespace {

constexpr int kExitOk = 0;
constexpr int kExitCheckFailed = 1;

struct DynOptions {
    std::string project, calibre_file, position = "dial_up", level = "reduced", engine = "builtin", duration = "60s", out, events_out;
    std::string study_file, readings_file, csv_out, calibre_out;
    double wind = 1.0, beat_rate = 0.0, jitter_us = 0.0;
    double amp_tolerance_percent = 2.0, rate_tolerance = 1.0;
    unsigned long seed = 1;
    double hs_youngs = 210.0, hs_width = 0.14, hs_thickness = 0.025, hs_inner_radius = 0.6, hs_pitch = 0.12, hs_turns = 12.0, hs_amplitude_deg = 300.0;
    bool fit_damping = false, verify = false, no_trace = false;
};

struct Setup {
    model::Movement movement;
    kinematics::TrainAnalysis train;
    TorqueChain chain;
    Calibre calibre;
};

Setup load_setup(const DynOptions& o) {
    Setup s;
    s.movement = model::load_rig(o.project);
    double vph = o.beat_rate > 0 ? o.beat_rate : s.movement.beat_rate_vph;
    if (vph <= 0) throw Error("the project has no beat rate; pass --beat-rate (e.g. 25200)");
    s.train = kinematics::analyze_train(s.movement, vph);
    s.calibre = o.calibre_file.empty() ? default_calibre(vph) : load_calibre(o.calibre_file);
    s.calibre.beat_rate_vph = vph;
    s.chain = build_chain(s.movement, s.train, s.calibre);
    adopt_train(s.calibre, s.chain, vph);
    return s;
}

double parse_duration(const std::string& text) {
    if (text.empty()) throw Error("empty duration");
    std::size_t used = 0;
    double v;
    try {
        v = std::stod(text, &used);
    } catch (const std::exception&) {
        throw Error("cannot read duration '" + text + "' (examples: 90s, 10m, 48h)");
    }
    const std::string unit = text.substr(used);
    if (unit.empty() || unit == "s") return v;
    if (unit == "m") return v * 60;
    if (unit == "h") return v * 3600;
    throw Error("unknown duration unit '" + unit + "' (use s, m or h)");
}

json to_json(const WindPoint& p) {
    return {{"wind", p.wind}, {"hours", p.hours}, {"position", to_string(p.position)},
            {"barrel_torque_nmm", p.barrel_torque_nmm}, {"wheel_torque_nm", p.wheel_torque_nm},
            {"amplitude_deg", p.amplitude_deg}, {"rate_s_per_day", p.rate_s_per_day},
            {"beat_error_ms", p.beat_error_ms}, {"stalled", p.stalled}};
}

json to_json(const Measurement& m) {
    return {{"beats", m.beats}, {"amplitude_deg", m.amplitude_deg}, {"period_s", m.period_s},
            {"rate_s_per_day", m.rate_s_per_day}, {"beat_error_ms", m.beat_error_ms}};
}

void write_file(const std::string& path, const std::string& text) {
    std::ofstream f(path);
    if (!f) throw Error("cannot write " + path);
    f << text;
}

int cmd_torque(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    const auto rows = torque_table(s.chain, s.calibre, o.wind);
    const double power = escape_power_w(s.chain, s.calibre, o.wind);
    if (as_json) {
        json j;
        j["wind"] = o.wind;
        j["barrel_torque_nmm"] = s.calibre.mainspring_torque_nmm(o.wind);
        j["speed_ratio"] = s.chain.speed_ratio;
        j["meshes"] = s.chain.meshes;
        j["efficiency"] = s.chain.efficiency;
        j["escape_torque_nm"] = escape_torque_nm(s.chain, s.calibre, o.wind);
        j["escape_power_w"] = power;
        j["power_reserve_full_wind_h"] = full_wind_hours(s.calibre, s.chain);
        j["stages"] = json::array();
        for (const auto& r : rows) j["stages"].push_back({{"mobile", r.mobile}, {"torque_nmm", r.torque_nmm}, {"lost_nmm", r.lost_nmm}});
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    std::cout << "state of wind " << o.wind << ": barrel torque " << std::fixed << std::setprecision(3)
              << s.calibre.mainspring_torque_nmm(o.wind) << " N·mm, " << s.chain.meshes << " meshes at "
              << s.calibre.mesh_efficiency << "\n";
    for (const auto& r : rows) {
        std::cout << std::left << std::setw(8) << r.mobile << std::right << std::setw(14) << std::scientific
                  << std::setprecision(3) << r.torque_nmm << " N·mm   lost in feeding mesh " << r.lost_nmm << " N·mm\n";
    }
    std::cout << std::defaultfloat << "escape wheel torque " << escape_torque_nm(s.chain, s.calibre, o.wind)
              << " N·m, power " << power << " W\n"
              << "mainspring lasts " << std::fixed << std::setprecision(1) << full_wind_hours(s.calibre, s.chain)
              << " h if the amplitude never limited it\n";
    return kExitOk;
}

int cmd_simulate(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    const Position pos = position_from_string(o.position);
    const double seconds = parse_duration(o.duration);
    if (o.level != "reduced" && o.level != "detailed") throw Error("--level must be reduced or detailed");
    const Engine engine = engine_from_string(o.engine);
    if (engine == Engine::Chrono && o.level != "detailed") throw Error("--engine chrono applies to --level detailed");

    json j;
    j["level"] = o.level;
    if (o.level == "detailed") j["engine"] = to_string(engine);
    j["position"] = to_string(pos);
    if (seconds <= 600.0) {
        const double torque = escape_torque_nm(s.chain, s.calibre, o.wind);
        EventLog log;
        RunSummary run;
        if (o.level == "reduced") {
            RunConditions c;
            c.position = pos;
            c.wheel_torque_nm = torque;
            c.duration_s = seconds;
            run = simulate_oscillator(s.calibre, c, log.sink());
        } else {
            DetailedConditions c;
            c.position = pos;
            c.wheel_torque_nm = torque;
            c.duration_s = seconds;
            run = engine == Engine::Chrono ? simulate_chrono(s.calibre, c, log.sink()) : simulate_detailed(s.calibre, c, log.sink());
        }
        const double from = run.end_time * 0.5;
        const Measurement m = measure(log.events, s.calibre, from, run.end_time);
        j["wind"] = o.wind;
        j["duration_s"] = run.end_time;
        j["stalled"] = run.stalled;
        j["measured_over"] = {from, run.end_time};
        j["result"] = to_json(m);
        if (!o.events_out.empty()) {
            std::ostringstream csv;
            csv << "t_s,event,theta_deg,omega_rad_s,direction\n";
            static const char* names[] = {"unlock_start", "impulse_start", "drop", "turn"};
            for (const Event& e : log.events) {
                csv << e.t << ',' << names[static_cast<int>(e.type)] << ',' << e.theta * 180.0 / std::numbers::pi << ',' << e.omega
                    << ',' << e.direction << '\n';
            }
            write_file(o.events_out, csv.str());
        }
        if (!as_json) {
            std::cout << o.level << " model, " << to_string(pos) << ", wind " << o.wind << ", " << run.end_time
                      << " s simulated" << (run.stalled ? " (STALLED)" : "") << "\n"
                      << "  amplitude " << std::fixed << std::setprecision(1) << m.amplitude_deg << " deg, rate "
                      << std::showpos << m.rate_s_per_day << std::noshowpos << " s/day, beat error "
                      << std::setprecision(3) << m.beat_error_ms << " ms (" << m.beats << " beats measured)\n";
        }
    } else {
        // Long runs: the amplitude settles in about a minute while the spring unwinds over hours,
        // so follow the run-down as a sequence of settled states.
        const double total_h = full_wind_hours(s.calibre, s.chain);
        const double hours = std::min(seconds / 3600.0, total_h);
        const int n = std::clamp(static_cast<int>(hours / 2.0) + 2, 2, 49);
        j["wind_down"] = json::array();
        double stopped_at = -1;
        for (int i = 0; i < n; ++i) {
            const double h = hours * i / (n - 1);
            const WindPoint p = wind_point(s.calibre, s.chain, std::max(1.0 - h / total_h, 0.0), pos);
            j["wind_down"].push_back(to_json(p));
            if (!as_json) {
                std::cout << std::fixed << std::setprecision(1) << std::setw(6) << p.hours << " h  wind " << std::setprecision(2)
                          << p.wind << "  amplitude " << std::setprecision(0) << p.amplitude_deg << " deg  rate "
                          << std::showpos << std::setprecision(1) << p.rate_s_per_day << std::noshowpos << " s/day\n";
            }
            if ((p.stalled || p.amplitude_deg < s.calibre.min_running_amplitude_deg) && stopped_at < 0) stopped_at = p.hours;
        }
        j["full_wind_hours"] = total_h;
        if (stopped_at >= 0) j["stopped_at_hours"] = stopped_at;
        if (!as_json && stopped_at >= 0) std::cout << "below the minimum running amplitude from about " << stopped_at << " h\n";
    }
    if (as_json) std::cout << j.dump(2) << '\n';
    if (!o.out.empty()) write_file(o.out, j.dump(2) + "\n");
    return kExitOk;
}

int cmd_timegrapher(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    const Position pos = position_from_string(o.position);
    const double seconds = parse_duration(o.duration);
    const double settle = 120.0;
    RunConditions c;
    c.position = pos;
    c.wheel_torque_nm = escape_torque_nm(s.chain, s.calibre, o.wind);
    c.duration_s = settle + seconds;
    EventLog log;
    simulate_oscillator(s.calibre, c, log.sink());
    const TimegrapherTrace tr = virtual_timegrapher(log.events, s.calibre, settle, settle + seconds, o.jitter_us, o.seed);
    if (!o.csv_out.empty()) write_file(o.csv_out, trace_csv(tr));
    if (as_json) {
        json j = {{"position", to_string(pos)}, {"wind", o.wind}, {"beats", tr.reading.beats},
                  {"rate_s_per_day", tr.reading.rate_s_per_day}, {"amplitude_deg", tr.reading.amplitude_deg},
                  {"beat_error_ms", tr.reading.beat_error_ms}};
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << "virtual timegrapher, " << to_string(pos) << ", wind " << o.wind << ", " << seconds << " s\n";
        if (!o.no_trace) std::cout << trace_ascii(tr);
        else
            std::cout << "rate " << tr.reading.rate_s_per_day << " s/d, amplitude " << tr.reading.amplitude_deg
                      << " deg, beat error " << tr.reading.beat_error_ms << " ms\n";
    }
    return kExitOk;
}

int cmd_timekeeping(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    const Timekeeping t = evaluate(s.calibre, s.chain);
    if (as_json) {
        json j;
        j["full"] = to_json(t.full);
        j["half"] = to_json(t.half);
        j["low"] = to_json(t.low);
        j["isochronism_s_per_day_per_deg"] = t.isochronism_s_per_day_per_deg;
        j["power_reserve_h"] = t.power_reserve_h;
        j["full_wind_h"] = t.full_wind_h;
        j["positional_spread_s_per_day"] = t.positions.spread_s_per_day;
        j["flat_to_vertical_amplitude_deg"] = t.positions.flat_to_vertical_amplitude_deg;
        j["positions"] = json::array();
        for (const auto& p : t.positions.points) j["positions"].push_back(to_json(p));
        j["energy_per_beat_j"] = {{"wheel_work", t.budget.wheel_work_j}, {"delivered", t.budget.delivered_j},
                                  {"lost_unlock", t.budget.lost_unlock_j}, {"lost_wheel_hit", t.budget.lost_wheel_hit_j},
                                  {"lost_friction", t.budget.lost_friction_j}, {"lost_air", t.budget.lost_air_j},
                                  {"lost_passing_spring", t.budget.lost_passing_j}, {"efficiency", t.budget.efficiency}};
        j["escapement"] = to_string(s.calibre.escapement);
        std::cout << j.dump(2) << '\n';
        return kExitOk;
    }
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "amplitude dial up: " << t.full.amplitude_deg << " deg fully wound, " << t.half.amplitude_deg
              << " half wound, " << t.low.amplitude_deg << " at 10 % wind\n"
              << "rate dial up: " << std::showpos << t.full.rate_s_per_day << " / " << t.half.rate_s_per_day << " / "
              << t.low.rate_s_per_day << std::noshowpos << " s/day; isochronism " << std::setprecision(3)
              << t.isochronism_s_per_day_per_deg << " s/day per degree\n"
              << std::setprecision(1) << "power reserve " << t.power_reserve_h << " h of " << t.full_wind_h
              << " h mainspring time (amplitude floor " << s.calibre.min_running_amplitude_deg << " deg)\n"
              << "positions at half wind:\n";
    for (const auto& p : t.positions.points) {
        std::cout << "  " << std::left << std::setw(12) << to_string(p.position) << std::right << std::setw(7)
                  << p.amplitude_deg << " deg " << std::showpos << std::setw(8) << p.rate_s_per_day << std::noshowpos
                  << " s/day\n";
    }
    std::cout << "positional spread " << t.positions.spread_s_per_day << " s/day\n"
              << std::setprecision(2) << "energy per " << (s.calibre.escapement == EscapementType::Detent ? "oscillation (one impulse)" : "beat") << " at full wind: wheel " << t.budget.wheel_work_j * 1e9
              << " nJ, balance gets " << t.budget.delivered_j * 1e9 << " nJ, unlocking " << t.budget.lost_unlock_j * 1e9
              << ", wheel pick-up " << t.budget.lost_wheel_hit_j * 1e9 << ", pivots " << t.budget.lost_friction_j * 1e9
              << ", air " << t.budget.lost_air_j * 1e9 << (t.budget.lost_passing_j > 0 ? ", passing spring " + std::to_string(t.budget.lost_passing_j * 1e9).substr(0, 5) : std::string())
              << " nJ (wheel to balance " << t.budget.efficiency * 100 << " %)\n";
    return kExitOk;
}

int cmd_whatif(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    StudyDefinition def = load_study(o.study_file);
    def.verify = def.verify || o.verify;
    const StudyResult r = run_study(s.calibre, s.chain, def);
    if (as_json) {
        json j;
        j["name"] = def.name;
        j["causes"] = r.causes;
        j["metrics"] = json::array();
        for (const auto& m : r.metrics) {
            j["metrics"].push_back({{"name", m.name}, {"unit", m.unit}, {"baseline", m.baseline}, {"modified", m.modified}, {"change", m.delta()}});
        }
        auto agreement = [](const std::optional<Agreement>& a) {
            return a ? json{{"amplitude_diff_percent", a->amplitude_diff_percent}, {"rate_diff_s_per_day", a->rate_diff_s_per_day}} : json();
        };
        j["baseline_check"] = agreement(r.baseline_check);
        j["modified_check"] = agreement(r.modified_check);
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << to_text(r);
    }
    return kExitOk;
}

int cmd_verify_levels(const DynOptions& o, bool as_json) {
    const Setup s = load_setup(o);
    const Position pos = position_from_string(o.position);
    const Engine engine = engine_from_string(o.engine);
    const Agreement a = compare_levels(s.calibre, escape_torque_nm(s.chain, s.calibre, o.wind), pos, 30.0, 20.0, engine);
    const bool ok = std::abs(a.amplitude_diff_percent) <= o.amp_tolerance_percent &&
                    std::abs(a.rate_diff_s_per_day) <= o.rate_tolerance;
    if (as_json) {
        json j = {{"engine", to_string(engine)}, {"position", to_string(pos)}, {"wind", o.wind}, {"reduced", to_json(a.level2)}, {"detailed", to_json(a.level3)},
                  {"amplitude_diff_percent", a.amplitude_diff_percent}, {"rate_diff_s_per_day", a.rate_diff_s_per_day},
                  {"tolerance_percent", o.amp_tolerance_percent}, {"tolerance_s_per_day", o.rate_tolerance}, {"ok", ok}};
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << std::fixed << std::setprecision(2) << "reduced : amplitude " << a.level2.amplitude_deg << " deg, rate "
                  << a.level2.rate_s_per_day << " s/day\ndetailed (" << to_string(engine) << "): amplitude " << a.level3.amplitude_deg << " deg, rate "
                  << a.level3.rate_s_per_day << " s/day\ndifference " << a.amplitude_diff_percent << " % amplitude, "
                  << a.rate_diff_s_per_day << " s/day rate -> " << (ok ? "agree" : "DISAGREE") << " (tolerance "
                  << o.amp_tolerance_percent << " %, " << o.rate_tolerance << " s/day)\n";
    }
    return ok ? kExitOk : kExitCheckFailed;
}

int cmd_bench(const DynOptions& o, bool as_json, bool calibrate_it) {
    const Setup s = load_setup(o);
    const auto readings = load_readings_csv(o.readings_file);
    Fit fit;
    json j;
    if (calibrate_it) {
        const Calibration cal = calibrate(s.calibre, s.chain, readings, o.fit_damping);
        fit = cal.after;
        j["fitted"] = cal.fitted;
        j["rms_amplitude_before_deg"] = cal.before.rms_amplitude_deg;
        j["pivot_friction_flat_nm"] = cal.calibre.pivot_friction_flat_nm;
        j["air_damping_nms"] = cal.calibre.air_damping_nms;
        if (!o.calibre_out.empty()) write_file(o.calibre_out, to_toml(cal.calibre));
        if (!as_json) {
            std::cout << "fitted " << cal.fitted.size() << " parameter(s): pivot friction "
                      << cal.before.rows.size() << " readings; rms amplitude error " << std::fixed << std::setprecision(1)
                      << cal.before.rms_amplitude_deg << " -> " << cal.after.rms_amplitude_deg << " deg; friction "
                      << std::scientific << std::setprecision(3) << s.calibre.pivot_friction_flat_nm << " -> "
                      << cal.calibre.pivot_friction_flat_nm << " N·m\n" << std::fixed;
        }
    } else {
        fit = compare_with_bench(s.calibre, s.chain, readings);
    }
    j["rms_amplitude_deg"] = fit.rms_amplitude_deg;
    j["rms_rate_s_per_day"] = fit.rms_rate_s_per_day;
    j["amplitude_slope_measured_deg_per_h"] = fit.amplitude_slope_measured;
    j["amplitude_slope_simulated_deg_per_h"] = fit.amplitude_slope_simulated;
    j["rows"] = json::array();
    for (const auto& r : fit.rows) {
        j["rows"].push_back({{"hours", r.measured.hours}, {"position", to_string(r.measured.position)},
                             {"measured_amplitude_deg", r.measured.amplitude_deg}, {"simulated_amplitude_deg", r.simulated_amplitude_deg},
                             {"measured_rate_s_per_day", r.measured.has_rate ? json(r.measured.rate_s_per_day) : json()},
                             {"simulated_rate_s_per_day", r.simulated_rate_s_per_day}});
    }
    if (as_json) {
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "hours position      measured/simulated amplitude    measured/simulated rate\n";
        for (const auto& r : fit.rows) {
            std::cout << std::setw(5) << r.measured.hours << "  " << std::left << std::setw(12) << to_string(r.measured.position)
                      << std::right << std::setw(10) << r.measured.amplitude_deg << " /" << std::setw(7) << r.simulated_amplitude_deg;
            if (r.measured.has_rate) std::cout << std::setw(14) << r.measured.rate_s_per_day << " /" << std::setw(7) << r.simulated_rate_s_per_day;
            std::cout << '\n';
        }
        std::cout << std::setprecision(2) << "rms amplitude error " << fit.rms_amplitude_deg << " deg; amplitude trend measured "
                  << fit.amplitude_slope_measured << " vs simulated " << fit.amplitude_slope_simulated << " deg/h\n";
    }
    return kExitOk;
}

int cmd_calibre_init(const DynOptions& o, bool as_json) {
    const Calibre c = default_calibre(o.beat_rate > 0 ? o.beat_rate : 25200);
    if (o.out.empty()) {
        std::cout << to_toml(c);
    } else {
        write_file(o.out, to_toml(c));
        if (!as_json) std::cout << "wrote " << o.out << '\n';
    }
    return kExitOk;
}

int cmd_calibre_hairspring(const DynOptions& o, bool as_json) {
    HairspringSpec s;
    s.youngs_gpa = o.hs_youngs;
    s.width_mm = o.hs_width;
    s.thickness_mm = o.hs_thickness;
    s.inner_radius_mm = o.hs_inner_radius;
    s.pitch_mm = o.hs_pitch;
    s.turns = o.hs_turns;
    const double amplitude = o.hs_amplitude_deg * std::numbers::pi / 180.0;
    const HairspringFit fit = fit_hairspring(s, amplitude);

    Calibre c = default_calibre(o.beat_rate > 0 ? o.beat_rate : 25200);
    if (!o.calibre_file.empty()) c = load_calibre(o.calibre_file);
    c.hairspring_youngs_gpa = s.youngs_gpa;
    c.hairspring_width_mm = s.width_mm;
    c.hairspring_thickness_mm = s.thickness_mm;
    c.hairspring_length_mm = s.length_mm();
    c.hairspring_cubic_nm_rad3 = fit.cubic_nm_rad3;
    const double frequency = c.natural_frequency_hz();
    // The oscillation grows slower or faster with amplitude by 3 c A^2 / 8 k, to first order.
    const double shift = 3.0 * fit.cubic_nm_rad3 * amplitude * amplitude / (8.0 * fit.stiffness_nm_rad);
    if (as_json) {
        json j = {{"length_mm", s.length_mm()}, {"stiffness_nm_rad", fit.stiffness_nm_rad}, {"euler_stiffness_nm_rad", fit.euler_stiffness},
                  {"cubic_nm_rad3", fit.cubic_nm_rad3}, {"asymmetry", fit.asymmetry}, {"max_side_force_n", fit.max_side_force_n},
                  {"frequency_shift_at_amplitude", shift}, {"natural_frequency_hz", frequency}};
        std::cout << j.dump(2) << '\n';
    } else {
        std::cout << std::setprecision(5) << "spiral of " << s.turns << " turns, strip " << s.length_mm() << " mm long\n"
                  << "stiffness " << fit.stiffness_nm_rad << " N.m/rad (E I / L would give " << fit.euler_stiffness << ")\n"
                  << "cubic term " << fit.cubic_nm_rad3 << " N.m/rad^3: the frequency changes by " << std::showpos << shift * 100 << std::noshowpos
                  << " % at " << o.hs_amplitude_deg << " degrees\n"
                  << "largest side force on the staff over the swing " << fit.max_side_force_n * 1e6 << " uN, odd/even asymmetry " << fit.asymmetry
                  << "\nfree frequency with this spring and the calibre's balance: " << frequency << " Hz ("
                  << frequency * 7200.0 << " vph)\n";
    }
    if (!o.out.empty()) {
        write_file(o.out, to_toml(c));
        if (!as_json) std::cout << "wrote " << o.out << '\n';
    }
    return kExitOk;
}

}  // namespace

void add_dynamics_commands(CLI::App& app, const bool& json_flag, std::function<int()>& chosen) {
    auto o = std::make_shared<DynOptions>();
    auto pick = [&chosen, &json_flag, o](int (*fn)(const DynOptions&, bool)) {
        return [&chosen, &json_flag, o, fn] { chosen = [&json_flag, o, fn] { return fn(*o, json_flag); }; };
    };
    auto common = [o](CLI::App* cmd, bool project_required = true) {
        auto* p = cmd->add_option("project", o->project, "Project file");
        if (project_required) p->required()->check(CLI::ExistingFile);
        cmd->add_option("--calibre", o->calibre_file, "Calibre parameters (TOML); default: a reference lever calibre")
            ->check(CLI::ExistingFile);
        cmd->add_option("--beat-rate", o->beat_rate, "Beat rate in vph (default: from project)");
    };

    auto* torque = app.add_subcommand("torque", "Torque chain from the barrel to the escape wheel");
    common(torque);
    torque->add_option("--wind", o->wind, "State of wind, 1 = full, 0 = run down")->capture_default_str();
    torque->callback(pick(cmd_torque));

    auto* sim = app.add_subcommand("simulate", "Simulate timekeeping (reduced event-driven or detailed contact model)");
    common(sim);
    sim->add_option("--level", o->level, "reduced or detailed")->capture_default_str();
    sim->add_option("--engine", o->engine, "Engine for --level detailed: builtin or chrono (needs a Chrono build)")->capture_default_str();
    sim->add_option("--duration", o->duration, "Simulated time: 90s, 10m, 48h (over 10 minutes follows the run-down)")->capture_default_str();
    sim->add_option("--position", o->position, "dial_up, dial_down, crown_up, crown_down, crown_left, crown_right")->capture_default_str();
    sim->add_option("--wind", o->wind, "State of wind for runs up to 10 minutes")->capture_default_str();
    sim->add_option("-o,--out", o->out, "Write the result as JSON");
    sim->add_option("--events", o->events_out, "Write every escapement event as CSV (short runs)");
    sim->callback(pick(cmd_simulate));

    auto* tg = app.add_subcommand("timegrapher", "Virtual timegrapher: rate, amplitude, beat error and trace");
    common(tg);
    tg->add_option("--duration", o->duration, "Listening time")->capture_default_str();
    tg->add_option("--position", o->position, "Orientation")->capture_default_str();
    tg->add_option("--wind", o->wind, "State of wind")->capture_default_str();
    tg->add_option("--jitter-us", o->jitter_us, "Timing noise in microseconds rms")->capture_default_str();
    tg->add_option("--seed", o->seed, "Noise seed")->capture_default_str();
    tg->add_option("--csv", o->csv_out, "Write the beats as CSV");
    tg->add_flag("--no-trace", o->no_trace, "Print only the readout");
    tg->callback(pick(cmd_timegrapher));

    auto* tk = app.add_subcommand("timekeeping", "Amplitude, rate, isochronism, positions, power reserve and energy budget");
    common(tk);
    tk->callback(pick(cmd_timekeeping));

    auto* wi = app.add_subcommand("whatif", "Compare the baseline with a modified design from a study file");
    common(wi);
    wi->add_option("study", o->study_file, "Study TOML")->required()->check(CLI::ExistingFile);
    wi->add_flag("--verify", o->verify, "Also check both designs against the detailed model");
    wi->callback(pick(cmd_whatif));

    auto* verify = app.add_subcommand("verify", "Cross-checks of the simulation");
    verify->require_subcommand(1);
    auto* levels = verify->add_subcommand("levels", "Detailed contact model against the reduced event-driven model");
    common(levels);
    levels->add_option("--wind", o->wind, "State of wind")->capture_default_str();
    levels->add_option("--engine", o->engine, "Engine for the detailed model: builtin or chrono (needs a Chrono build)")->capture_default_str();
    levels->add_option("--position", o->position, "Orientation")->capture_default_str();
    levels->add_option("--amplitude-tolerance", o->amp_tolerance_percent, "Allowed amplitude difference in percent")->capture_default_str();
    levels->add_option("--rate-tolerance", o->rate_tolerance, "Allowed rate difference in s/day")->capture_default_str();
    levels->callback(pick(cmd_verify_levels));

    auto* bench = app.add_subcommand("bench", "Compare or calibrate against timegrapher readings from a real movement");
    bench->require_subcommand(1);
    auto* compare = bench->add_subcommand("compare", "Simulate each reading and compare");
    common(compare);
    compare->add_option("readings", o->readings_file, "CSV: hours,position,amplitude_deg,rate_s_per_day,beat_error_ms")
        ->required()->check(CLI::ExistingFile);
    compare->callback(pick([](const DynOptions& d, bool j) { return cmd_bench(d, j, false); }));
    auto* cal = bench->add_subcommand("calibrate", "Fit pivot friction (and air damping) to measured amplitudes");
    common(cal);
    cal->add_option("readings", o->readings_file, "Readings CSV")->required()->check(CLI::ExistingFile);
    cal->add_flag("--fit-damping", o->fit_damping, "Also fit the air damping");
    cal->add_option("-o,--output", o->calibre_out, "Write the calibrated calibre as TOML");
    cal->callback(pick([](const DynOptions& d, bool j) { return cmd_bench(d, j, true); }));

    auto* calibre = app.add_subcommand("calibre", "Calibre parameter files");
    calibre->require_subcommand(1);
    auto* init = calibre->add_subcommand("init", "Write the default calibre parameters to edit");
    init->add_option("-o,--output", o->out, "File to write (default: stdout)");
    init->add_option("--beat-rate", o->beat_rate, "Beat rate in vph")->capture_default_str();
    init->callback(pick(cmd_calibre_init));
    auto* hs = calibre->add_subcommand("hairspring", "Fit a calibre's hairspring from a beam model of a flat spiral: stiffness and the cubic term");
    hs->add_option("--calibre", o->calibre_file, "Start from this calibre (default: the reference one)")->check(CLI::ExistingFile);
    hs->add_option("--youngs", o->hs_youngs, "Young's modulus in GPa")->capture_default_str();
    hs->add_option("--width", o->hs_width, "Strip height along the staff in mm")->capture_default_str();
    hs->add_option("--thickness", o->hs_thickness, "Strip radial thickness in mm")->capture_default_str();
    hs->add_option("--inner-radius", o->hs_inner_radius, "Radius of the inner end, mm")->capture_default_str();
    hs->add_option("--pitch", o->hs_pitch, "Distance between coils, mm")->capture_default_str();
    hs->add_option("--turns", o->hs_turns, "Number of coils")->capture_default_str();
    hs->add_option("--amplitude", o->hs_amplitude_deg, "Amplitude to fit over, degrees")->capture_default_str();
    hs->add_option("--beat-rate", o->beat_rate, "Beat rate in vph for the rest of the calibre")->capture_default_str();
    hs->add_option("-o,--output", o->out, "Write the calibre with the fitted spring as TOML");
    hs->callback(pick(cmd_calibre_hairspring));
}

}  // namespace ohs::cli
