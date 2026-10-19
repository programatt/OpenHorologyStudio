// SPDX-License-Identifier: Apache-2.0
#include "dynamics/calibre.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>

#include "core/error.hpp"

namespace ohs::dynamics {

namespace {

constexpr double kGravity = 9.80665;

struct Field {
    const char* name;
    double Calibre::*member;
};

const std::vector<Field>& fields() {
    static const std::vector<Field> f = {
        {"barrel_torque_nmm", &Calibre::barrel_torque_nmm},
        {"barrel_turns", &Calibre::barrel_turns},
        {"mesh_efficiency", &Calibre::mesh_efficiency},
        {"escape_pivot_loss_nmm", &Calibre::escape_pivot_loss_nmm},
        {"balance_inertia_mgcm2", &Calibre::balance_inertia_mgcm2},
        {"balance_mass_mg", &Calibre::balance_mass_mg},
        {"unpoise_um", &Calibre::unpoise_um},
        {"unpoise_angle_deg", &Calibre::unpoise_angle_deg},
        {"beat_offset_deg", &Calibre::beat_offset_deg},
        {"hairspring_youngs_gpa", &Calibre::hairspring_youngs_gpa},
        {"hairspring_width_mm", &Calibre::hairspring_width_mm},
        {"hairspring_thickness_mm", &Calibre::hairspring_thickness_mm},
        {"hairspring_length_mm", &Calibre::hairspring_length_mm},
        {"air_damping_nms", &Calibre::air_damping_nms},
        {"pivot_friction_flat_nm", &Calibre::pivot_friction_flat_nm},
        {"vertical_friction_factor", &Calibre::vertical_friction_factor},
        {"escape_teeth", &Calibre::escape_teeth},
        {"impulse_lead_deg", &Calibre::impulse_lead_deg},
        {"lift_angle_deg", &Calibre::lift_angle_deg},
        {"unlock_angle_deg", &Calibre::unlock_angle_deg},
        {"drop_angle_deg", &Calibre::drop_angle_deg},
        {"impulse_efficiency", &Calibre::impulse_efficiency},
        {"unlock_friction_ratio", &Calibre::unlock_friction_ratio},
        {"passing_spring_ratio", &Calibre::passing_spring_ratio},
        {"escape_wheel_inertia_mgcm2", &Calibre::escape_wheel_inertia_mgcm2},
        {"contact_damping_ratio", &Calibre::contact_damping_ratio},
        {"contact_frequency_hz", &Calibre::contact_frequency_hz},
        {"beat_rate_vph", &Calibre::beat_rate_vph},
        {"min_running_amplitude_deg", &Calibre::min_running_amplitude_deg},
    };
    return f;
}

}  // namespace

const char* to_string(Position p) {
    switch (p) {
        case Position::DialUp: return "dial_up";
        case Position::DialDown: return "dial_down";
        case Position::CrownUp: return "crown_up";
        case Position::CrownDown: return "crown_down";
        case Position::CrownLeft: return "crown_left";
        case Position::CrownRight: return "crown_right";
    }
    return "?";
}

Position position_from_string(const std::string& text) {
    for (Position p : all_positions()) {
        if (text == to_string(p)) return p;
    }
    throw Error("unknown position '" + text + "' (dial_up, dial_down, crown_up, crown_down, crown_left, crown_right)");
}

const std::vector<Position>& all_positions() {
    static const std::vector<Position> v = {Position::DialUp,   Position::DialDown,  Position::CrownUp,
                                            Position::CrownDown, Position::CrownLeft, Position::CrownRight};
    return v;
}

const char* to_string(EscapementType e) { return e == EscapementType::Detent ? "detent" : "swiss_lever"; }

EscapementType escapement_from_string(const std::string& text) {
    if (text == "swiss_lever") return EscapementType::SwissLever;
    if (text == "detent") return EscapementType::Detent;
    throw Error("unknown escapement '" + text + "' (swiss_lever or detent)");
}

bool is_vertical(Position p) { return p != Position::DialUp && p != Position::DialDown; }

double Calibre::inertia() const { return balance_inertia_mgcm2 * 1e-10; }
double Calibre::escape_wheel_inertia() const { return escape_wheel_inertia_mgcm2 * 1e-10; }

double Calibre::hairspring_stiffness() const {
    const double E = hairspring_youngs_gpa * 1e9;
    const double b = hairspring_width_mm * 1e-3;
    const double h = hairspring_thickness_mm * 1e-3;
    const double L = hairspring_length_mm * 1e-3;
    if (L <= 0) throw Error("hairspring length is not set");
    return E * b * h * h * h / (12.0 * L);
}

double Calibre::natural_frequency_hz() const {
    return std::sqrt(hairspring_stiffness() / inertia()) / (2.0 * std::numbers::pi);
}

double Calibre::unpoise_torque() const { return balance_mass_mg * 1e-6 * kGravity * unpoise_um * 1e-6; }

double Calibre::mainspring_torque_nmm(double wind) const {
    wind = std::clamp(wind, 0.0, 1.0);
    const auto& c = torque_curve;
    if (c.empty()) return barrel_torque_nmm;
    if (wind <= c.front().first) return barrel_torque_nmm * c.front().second;
    for (std::size_t i = 1; i < c.size(); ++i) {
        if (wind <= c[i].first) {
            const double x = (wind - c[i - 1].first) / (c[i].first - c[i - 1].first);
            return barrel_torque_nmm * (c[i - 1].second + x * (c[i].second - c[i - 1].second));
        }
    }
    return barrel_torque_nmm * c.back().second;
}

double Calibre::friction_torque(Position p) const {
    return pivot_friction_flat_nm * (is_vertical(p) ? vertical_friction_factor : 1.0);
}

Calibre default_calibre(double beat_rate_vph) {
    Calibre c;
    c.beat_rate_vph = beat_rate_vph;
    const double omega = 2.0 * std::numbers::pi * c.beat_rate_vph / 7200.0;
    const double k = c.inertia() * omega * omega;
    const double E = c.hairspring_youngs_gpa * 1e9, b = c.hairspring_width_mm * 1e-3,
                 h = c.hairspring_thickness_mm * 1e-3;
    c.hairspring_length_mm = E * b * h * h * h / (12.0 * k) * 1e3;
    return c;
}

double& parameter(Calibre& c, const std::string& name) {
    for (const Field& f : fields()) {
        if (name == f.name) return c.*(f.member);
    }
    throw Error("unknown parameter '" + name + "'");
}

const std::vector<std::string>& parameter_names() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> v;
        for (const Field& f : fields()) v.push_back(f.name);
        return v;
    }();
    return names;
}

Calibre calibre_from_toml(const std::string& text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        throw Error(std::string("calibre file: ") + std::string(e.description()));
    }
    Calibre c = default_calibre();
    const toml::table* t = root["calibre"].as_table();
    if (!t) throw Error("calibre file: missing [calibre] table");
    bool length_given = false;
    for (const auto& [key, value] : *t) {
        const std::string k(key.str());
        if (k == "name") {
            if (auto s = value.value<std::string>()) c.name = *s;
            continue;
        }
        if (k == "escapement") {
            const auto s = value.value<std::string>();
            if (!s) throw Error("calibre file: 'escapement' must be swiss_lever or detent");
            c.escapement = escapement_from_string(*s);
            continue;
        }
        if (k == "torque_curve") {
            const toml::array* a = value.as_array();
            if (!a) throw Error("calibre file: torque_curve must be an array of [wind, torque] pairs");
            c.torque_curve.clear();
            for (const auto& el : *a) {
                const toml::array* p = el.as_array();
                if (!p || p->size() != 2) throw Error("calibre file: torque_curve entries are [wind, torque]");
                c.torque_curve.emplace_back(p->at(0).value_or(0.0), p->at(1).value_or(0.0));
            }
            std::sort(c.torque_curve.begin(), c.torque_curve.end());
            continue;
        }
        const auto v = value.value<double>();
        if (!v) throw Error("calibre file: '" + k + "' must be a number");
        parameter(c, k) = *v;
        if (k == "hairspring_length_mm") length_given = true;
    }
    if (!length_given) {
        // Re-derive the length when the inertia, section or beat rate changed.
        const double omega = 2.0 * std::numbers::pi * c.beat_rate_vph / 7200.0;
        const double k = c.inertia() * omega * omega;
        const double E = c.hairspring_youngs_gpa * 1e9, b = c.hairspring_width_mm * 1e-3,
                     h = c.hairspring_thickness_mm * 1e-3;
        c.hairspring_length_mm = E * b * h * h * h / (12.0 * k) * 1e3;
    }
    return c;
}

Calibre load_calibre(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();
    return calibre_from_toml(buf.str());
}

std::string to_toml(const Calibre& calibre) {
    Calibre c = calibre;
    toml::table cal;
    cal.insert("name", c.name);
    cal.insert("escapement", std::string(to_string(c.escapement)));
    for (const Field& f : fields()) cal.insert(f.name, c.*(f.member));
    toml::array curve;
    for (const auto& [w, t] : c.torque_curve) curve.push_back(toml::array{w, t});
    cal.insert("torque_curve", curve);
    toml::table root;
    root.insert("calibre", cal);
    std::ostringstream os;
    os << root;
    return os.str();
}

}  // namespace ohs::dynamics
