// SPDX-License-Identifier: Apache-2.0
#include "dynamics/study.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "core/error.hpp"

namespace ohs::dynamics {

namespace {

bool in(const std::string& name, std::initializer_list<const char*> names) {
    return std::any_of(names.begin(), names.end(), [&](const char* n) { return name == n; });
}

std::string num(double v, int precision = 3) {
    std::ostringstream os;
    os << std::setprecision(precision) << v;
    return os.str();
}

std::string percent(double ratio) {
    std::ostringstream os;
    os << std::showpos << std::fixed << std::setprecision(1) << (ratio - 1.0) * 100.0 << " %";
    return os.str();
}

std::string describe(const Change& ch, double before, double after) {
    std::ostringstream os;
    os << ch.parameter << " " << num(before) << " -> " << num(after);
    if (before != 0) os << " (" << percent(after / before) << ")";
    return os.str();
}

}  // namespace

void apply(Calibre& c, const Change& change) {
    double& v = parameter(c, change.parameter);
    switch (change.mode) {
        case Change::Mode::Set: v = change.value; break;
        case Change::Mode::Scale: v *= change.value; break;
        case Change::Mode::Add: v += change.value; break;
    }
}

StudyDefinition study_from_toml(const std::string& text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        throw Error(std::string("study file: ") + std::string(e.description()));
    }
    StudyDefinition d;
    if (const toml::table* s = root["study"].as_table()) {
        d.name = (*s)["name"].value_or(d.name);
        d.regulate = (*s)["regulate"].value_or(d.regulate);
        d.verify = (*s)["verify"].value_or(d.verify);
    }
    if (const toml::array* arr = root["change"].as_array()) {
        for (const auto& el : *arr) {
            const toml::table* t = el.as_table();
            if (!t) throw Error("study file: [[change]] entries must be tables");
            Change ch;
            ch.parameter = (*t)["parameter"].value_or(std::string());
            if (ch.parameter.empty()) throw Error("study file: a [[change]] has no parameter");
            int given = 0;
            if (auto v = (*t)["scale"].value<double>()) {
                ch.mode = Change::Mode::Scale;
                ch.value = *v;
                ++given;
            }
            if (auto v = (*t)["set"].value<double>()) {
                ch.mode = Change::Mode::Set;
                ch.value = *v;
                ++given;
            }
            if (auto v = (*t)["add"].value<double>()) {
                ch.mode = Change::Mode::Add;
                ch.value = *v;
                ++given;
            }
            if (given != 1) throw Error("study file: change of " + ch.parameter + " needs exactly one of scale, set, add");
            parameter_names();  // validates lazily in apply
            d.changes.push_back(ch);
        }
    }
    if (d.changes.empty()) throw Error("study file: no [[change]] entries, nothing to compare");
    return d;
}

StudyDefinition load_study(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << f.rdbuf();
    return study_from_toml(buf.str());
}

namespace {

/// A sentence per change, with the governing relationship evaluated for this design.
std::string explain_change(const Change& ch, const Calibre& base, const Calibre& mod, const TorqueChain& chain,
                           const Timekeeping& tb, const Timekeeping& tm) {
    Calibre b = base, m = mod;
    const double before = parameter(b, ch.parameter), after = parameter(m, ch.parameter);
    std::ostringstream os;
    os << describe(ch, before, after) << ": ";

    if (in(ch.parameter, {"hairspring_length_mm", "hairspring_width_mm", "hairspring_thickness_mm",
                          "hairspring_youngs_gpa", "balance_inertia_mgcm2"})) {
        const double k_ratio = mod.hairspring_stiffness() / base.hairspring_stiffness();
        const double i_ratio = mod.inertia() / base.inertia();
        const double f_ratio = std::sqrt(k_ratio / i_ratio);
        const double predicted = (f_ratio - 1.0) * 86400.0;
        const double simulated = tm.half.rate_s_per_day - tb.half.rate_s_per_day;
        os << "the oscillation frequency is sqrt(k/I)/2pi. Hairspring stiffness k changes " << percent(k_ratio)
           << " (k = E b h^3 / 12 L) and balance inertia I " << percent(i_ratio) << ", so the frequency changes "
           << percent(f_ratio) << " and the rate by " << std::showpos << std::fixed << std::setprecision(0) << predicted
           << " s/day predicted, " << simulated << " s/day simulated (the difference is the escapement's own disturbance).";
        return os.str();
    }
    if (in(ch.parameter, {"barrel_torque_nmm", "mesh_efficiency", "escape_teeth", "impulse_efficiency", "drop_angle_deg",
                          "lift_angle_deg", "escape_pivot_loss_nmm", "unlock_friction_ratio", "unlock_angle_deg"})) {
        const double eb = tb.budget.delivered_j - tb.budget.lost_unlock_j - tb.budget.lost_wheel_hit_j;
        const double em = tm.budget.delivered_j - tm.budget.lost_unlock_j - tm.budget.lost_wheel_hit_j;
        os << "this changes the energy each beat puts into the balance, from " << num(eb * 1e9) << " nJ to "
           << num(em * 1e9) << " nJ (" << percent(eb != 0 ? em / eb : 1.0) << ") at full wind. The balance settles where "
           << "that equals what friction and air take, so the amplitude moves from "
           << std::fixed << std::setprecision(1) << tb.full.amplitude_deg << " to " << tm.full.amplitude_deg << " deg"
           << " and the power reserve from " << tb.power_reserve_h << " to " << tm.power_reserve_h << " h.";
        if (ch.parameter == "escape_teeth") {
            os << " The wheel advances pi/teeth per beat, so more teeth mean less work per beat for the same torque; the"
                  " train ratio is assumed to be changed to keep the beat rate.";
        }
        (void)chain;
        return os.str();
    }
    if (in(ch.parameter, {"pivot_friction_flat_nm", "vertical_friction_factor", "air_damping_nms"})) {
        const double lb = tb.budget.lost_friction_j + tb.budget.lost_air_j;
        const double lm = tm.budget.lost_friction_j + tm.budget.lost_air_j;
        os << "friction and air take " << num(lb * 1e9) << " nJ per beat at full wind in the baseline against "
           << num(lm * 1e9) << " nJ here (" << percent(lb != 0 ? lm / lb : 1.0) << "), so the amplitude moves from "
           << std::fixed << std::setprecision(1) << tb.full.amplitude_deg << " to " << tm.full.amplitude_deg
           << " deg; the flat-to-vertical amplitude difference goes from " << tb.positions.flat_to_vertical_amplitude_deg
           << " to " << tm.positions.flat_to_vertical_amplitude_deg << " deg.";
        return os.str();
    }
    if (in(ch.parameter, {"unpoise_um", "unpoise_angle_deg", "balance_mass_mg"})) {
        os << "an unpoised balance gets a gravity torque m g e sin(theta + phase) in the hanging positions, which "
              "shifts the rate there and not when lying flat. The positional spread moves from "
           << std::fixed << std::setprecision(1) << tb.positions.spread_s_per_day << " to "
           << tm.positions.spread_s_per_day << " s/day.";
        return os.str();
    }
    if (in(ch.parameter, {"beat_offset_deg", "impulse_lead_deg"})) {
        os << "the escapement's phase against the hairspring decides the beat error and how the rate depends on "
              "amplitude. Beat error goes from "
           << std::fixed << std::setprecision(2) << tb.positions.points.front().beat_error_ms << " to "
           << tm.positions.points.front().beat_error_ms << " ms and the isochronism slope from "
           << std::setprecision(3) << tb.isochronism_s_per_day_per_deg << " to " << tm.isochronism_s_per_day_per_deg
           << " s/day per degree.";
        return os.str();
    }
    os << "no closed-form rule is attached to this parameter; compare the simulated metrics below.";
    return os.str();
}

}  // namespace

StudyResult run_study(const Calibre& baseline, const TorqueChain& chain, const StudyDefinition& def) {
    StudyResult r;
    r.definition = def;
    r.baseline = def.regulate ? regulate(baseline, chain, 0.0, 0.5) : baseline;
    r.modified = r.baseline;
    for (const Change& ch : def.changes) apply(r.modified, ch);
    r.baseline_result = evaluate(r.baseline, chain);
    r.modified_result = evaluate(r.modified, chain);

    const Timekeeping& b = r.baseline_result;
    const Timekeeping& m = r.modified_result;
    auto add = [&](const char* name, const char* unit, double x, double y) { r.metrics.push_back({name, unit, x, y}); };
    add("amplitude, full wind", "deg", b.full.amplitude_deg, m.full.amplitude_deg);
    add("amplitude, half wind", "deg", b.half.amplitude_deg, m.half.amplitude_deg);
    add("amplitude, 10 % wind", "deg", b.low.amplitude_deg, m.low.amplitude_deg);
    add("rate, dial up, half wind", "s/day", b.half.rate_s_per_day, m.half.rate_s_per_day);
    add("isochronism slope", "s/day per deg", b.isochronism_s_per_day_per_deg, m.isochronism_s_per_day_per_deg);
    add("positional spread", "s/day", b.positions.spread_s_per_day, m.positions.spread_s_per_day);
    add("flat - vertical amplitude", "deg", b.positions.flat_to_vertical_amplitude_deg,
        m.positions.flat_to_vertical_amplitude_deg);
    add("power reserve", "h", b.power_reserve_h, m.power_reserve_h);
    add("beat error, dial up", "ms", b.half.beat_error_ms, m.half.beat_error_ms);
    add("energy efficiency wheel to balance", "", b.budget.efficiency, m.budget.efficiency);

    for (const Change& ch : def.changes) {
        r.causes.push_back(explain_change(ch, r.baseline, r.modified, chain, b, m));
    }
    if (def.verify) {
        r.baseline_check =
            compare_levels(r.baseline, escape_torque_nm(chain, r.baseline, 0.5), Position::DialUp);
        r.modified_check =
            compare_levels(r.modified, escape_torque_nm(chain, r.modified, 0.5), Position::DialUp);
    }
    return r;
}

std::string to_text(const StudyResult& r) {
    std::ostringstream os;
    os << "what-if: " << r.definition.name << "\n";
    for (const std::string& c : r.causes) os << "  * " << c << "\n";
    os << "\n" << std::left << std::setw(38) << "metric" << std::right << std::setw(12) << "baseline" << std::setw(12)
       << "modified" << std::setw(12) << "change" << "  unit\n";
    for (const MetricRow& m : r.metrics) {
        os << std::left << std::setw(38) << m.name << std::right << std::fixed << std::setprecision(2) << std::setw(12)
           << m.baseline << std::setw(12) << m.modified << std::showpos << std::setw(12) << m.delta() << std::noshowpos
           << "  " << m.unit << "\n";
    }
    auto check = [&](const char* label, const std::optional<Agreement>& a) {
        if (!a) return;
        os << label << ": detailed model vs reduced model: amplitude " << std::showpos << std::setprecision(2)
           << a->amplitude_diff_percent << " %, rate " << a->rate_diff_s_per_day << " s/day" << std::noshowpos << "\n";
    };
    check("baseline", r.baseline_check);
    check("modified", r.modified_check);
    return os.str();
}

}  // namespace ohs::dynamics
