// SPDX-License-Identifier: Apache-2.0
#include "manufacturing/tolerance.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <numbers>
#include <sstream>
#include <thread>

#include "core/error.hpp"

namespace ohs::manufacturing {

namespace {

constexpr double kPi = std::numbers::pi;

/// splitmix64: a counter-based generator, so sample s always sees the same numbers.
class Rng {
public:
    Rng(std::uint64_t seed, std::uint64_t sample) : state_(seed ^ (sample * 0x9E3779B97F4A7C15ULL) ^ 0xD1B54A32D192ED03ULL) { next(); }
    std::uint64_t next() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    double uniform() { return (static_cast<double>(next() >> 11) + 0.5) * (1.0 / 9007199254740992.0); }
    double normal() { return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * kPi * uniform()); }

private:
    std::uint64_t state_;
};

struct GearData {
    double teeth{}, module{}, alpha{}, ra{}, rb{};
};

GearData gear_data(const model::Component& c, double module_hint) {
    GearData g;
    g.teeth = c.features.teeth;
    g.module = c.gear && c.gear->module > 0 ? c.gear->module : (module_hint > 0 ? module_hint : model::module_from_tip(c.features.outer_radius, c.features.teeth));
    g.alpha = (c.gear && c.gear->pressure_angle_deg > 5 ? c.gear->pressure_angle_deg : 20.0) * kPi / 180;
    g.ra = c.features.outer_radius;
    g.rb = c.gear && c.gear->base_radius > 0 ? c.gear->base_radius : 0.5 * g.module * g.teeth * std::cos(g.alpha);
    return g;
}

double contact_ratio(const GearData& a, const GearData& b, double centre, double module, double alpha) {
    const double ca = (a.rb + b.rb) / centre;
    if (ca >= 1.0) return 0.0;
    const double alpha_w = std::acos(ca);
    const double lengths = std::sqrt(std::max(a.ra * a.ra - a.rb * a.rb, 0.0)) + std::sqrt(std::max(b.ra * b.ra - b.rb * b.rb, 0.0)) - centre * std::sin(alpha_w);
    return lengths / (kPi * module * std::cos(alpha));
}

Stat stats(std::vector<double> v) {
    Stat s;
    if (v.empty()) return s;
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    double ss = 0;
    for (double x : v) ss += (x - s.mean) * (x - s.mean);
    s.sd = std::sqrt(ss / static_cast<double>(std::max<std::size_t>(v.size() - 1, 1)));
    std::sort(v.begin(), v.end());
    s.min = v.front();
    s.max = v.back();
    auto at = [&](double q) { return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1) + 0.5)]; };
    s.p01 = at(0.01);
    s.p99 = at(0.99);
    return s;
}

double correlation(const std::vector<double>& x, const std::vector<double>& y) {
    const double n = static_cast<double>(x.size());
    double mx = 0, my = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        mx += x[i];
        my += y[i];
    }
    mx /= n;
    my /= n;
    double sxy = 0, sxx = 0, syy = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        sxy += (x[i] - mx) * (y[i] - my);
        sxx += (x[i] - mx) * (x[i] - mx);
        syy += (y[i] - my) * (y[i] - my);
    }
    return sxx > 0 && syy > 0 ? sxy / std::sqrt(sxx * syy) : 0.0;
}

std::vector<Sensitivity> rank(const std::vector<std::string>& names, const std::vector<std::vector<double>>& inputs, const std::vector<double>& output) {
    std::vector<Sensitivity> out;
    double total = 0;
    for (std::size_t k = 0; k < names.size(); ++k) {
        Sensitivity s;
        s.input = names[k];
        s.correlation = correlation(inputs[k], output);
        total += s.correlation * s.correlation;
        out.push_back(s);
    }
    for (Sensitivity& s : out) s.share = total > 0 ? s.correlation * s.correlation / total : 0.0;
    std::sort(out.begin(), out.end(), [](const Sensitivity& a, const Sensitivity& b) { return a.share > b.share; });
    return out;
}

template <class F>
void parallel_samples(int n, F f) {
    const unsigned threads = std::max(1u, std::min(16u, std::thread::hardware_concurrency()));
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t) {
        pool.emplace_back([=] {
            for (int s = static_cast<int>(t); s < n; s += static_cast<int>(threads)) f(s);
        });
    }
    for (auto& th : pool) th.join();
}

}  // namespace

std::vector<PmiApplied> apply_pmi(ToleranceConfig& config, const std::vector<geometry::PmiDimension>& dimensions, const model::Movement& movement) {
    struct Target {
        const char* field;
        double ToleranceConfig::*member;
    };
    std::vector<PmiApplied> applied;
    std::map<std::string, std::pair<double, std::string>> loosest;  // field -> tolerance and the dimension it came from
    std::map<std::string, double ToleranceConfig::*> members;
    for (const Target& t : {Target{"pivot_diameter_tol", &ToleranceConfig::pivot_diameter_tol}, Target{"hole_diameter_tol", &ToleranceConfig::hole_diameter_tol},
                            Target{"arbor_length_tol", &ToleranceConfig::arbor_length_tol}, Target{"bridge_gap_tol", &ToleranceConfig::bridge_gap_tol}}) {
        members[t.field] = t.member;
    }
    for (const geometry::PmiDimension& d : dimensions) {
        const int index = movement.find_component(d.part);
        if (index < 0) continue;
        const model::Role role = movement.components[static_cast<std::size_t>(index)].role;
        const char* field = nullptr;
        if (d.kind == "diameter") {
            if (role == model::Role::Plate || role == model::Role::Bridge) field = "hole_diameter_tol";
            else if (model::is_rotating_role(role)) field = "pivot_diameter_tol";
        } else if (d.kind == "length") {
            if (role == model::Role::Arbor) field = "arbor_length_tol";
            else if (role == model::Role::Bridge) field = "bridge_gap_tol";
        }
        if (!field) continue;
        const double tol = std::max(d.plus, d.minus);
        if (tol <= 0) continue;
        std::ostringstream src;
        src << d.part << ' ' << d.kind << ' ' << std::fixed << std::setprecision(3) << d.nominal << " +" << d.plus << "/-" << d.minus;
        auto it = loosest.find(field);
        if (it == loosest.end() || tol > it->second.first) loosest[field] = {tol, src.str()};
    }
    for (const auto& [field, found] : loosest) {
        config.*members[field] = found.first;
        applied.push_back({field, found.first, found.second});
    }
    return applied;
}

ToleranceConfig tolerance_from_toml(const std::string& text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        throw Error(std::string("tolerance file: ") + std::string(e.description()));
    }
    const toml::table* t = root["tolerance"].as_table();
    if (!t) throw Error("tolerance file: missing [tolerance] table");
    ToleranceConfig c;
    struct Field {
        const char* name;
        double ToleranceConfig::*member;
    };
    static const Field fields[] = {
        {"position_tol", &ToleranceConfig::position_tol}, {"tooth_thickness_tol", &ToleranceConfig::tooth_thickness_tol},
        {"pivot_diameter_tol", &ToleranceConfig::pivot_diameter_tol}, {"hole_diameter_tol", &ToleranceConfig::hole_diameter_tol},
        {"arbor_length_tol", &ToleranceConfig::arbor_length_tol}, {"bridge_gap_tol", &ToleranceConfig::bridge_gap_tol},
        {"plate_flatness_tol", &ToleranceConfig::plate_flatness_tol}, {"nominal_backlash_factor", &ToleranceConfig::nominal_backlash_factor},
        {"nominal_end_shake", &ToleranceConfig::nominal_end_shake}, {"nominal_side_shake", &ToleranceConfig::nominal_side_shake},
        {"min_backlash", &ToleranceConfig::min_backlash}, {"min_contact_ratio", &ToleranceConfig::min_contact_ratio},
        {"end_shake_min", &ToleranceConfig::end_shake_min}, {"end_shake_max", &ToleranceConfig::end_shake_max},
        {"side_shake_min", &ToleranceConfig::side_shake_min}, {"side_shake_max", &ToleranceConfig::side_shake_max}};
    for (const auto& [key, value] : *t) {
        const std::string k(key.str());
        if (k == "samples") {
            c.samples = static_cast<int>(value.value_or(c.samples));
        } else if (k == "seed") {
            c.seed = static_cast<std::uint64_t>(value.value_or(static_cast<std::int64_t>(c.seed)));
        } else if (k == "uniform") {
            c.uniform = value.value_or(false);
        } else {
            bool known = false;
            for (const Field& f : fields) {
                if (k == f.name) {
                    const auto v = value.value<double>();
                    if (!v) throw Error("tolerance file: '" + k + "' must be a number");
                    c.*(f.member) = *v;
                    known = true;
                }
            }
            if (!known) throw Error("tolerance file: unknown key '" + k + "'");
        }
    }
    return c;
}

ToleranceConfig load_tolerance(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();
    return tolerance_from_toml(buf.str());
}

ToleranceResult run_tolerance(const model::Movement& m, const ToleranceConfig& c) {
    if (c.samples < 10) throw Error("use at least 10 samples");
    if (m.meshes.empty()) throw Error("the rig has no gear meshes to analyse");
    const int n = c.samples;
    auto draw = [&](Rng& r, double tol) { return c.uniform ? (2 * r.uniform() - 1) * tol : r.normal() * tol / 3.0; };

    const int nm = static_cast<int>(m.mobiles.size());
    // Variable layout per sample: for each mobile 2 position + 5 shake variables; for each component one tooth thickness.
    const int nc = static_cast<int>(m.components.size());
    struct Vars {
        std::vector<double> px, py, bridge, flat, arbor, hole_up, pivot_up, hole_lo, pivot_lo, tooth;
    };
    std::vector<Vars> vars(static_cast<std::size_t>(n));
    for (Vars& v : vars) {
        v.px.resize(nm);
        v.py.resize(nm);
        v.bridge.resize(nm);
        v.flat.resize(nm);
        v.arbor.resize(nm);
        v.hole_up.resize(nm);
        v.pivot_up.resize(nm);
        v.hole_lo.resize(nm);
        v.pivot_lo.resize(nm);
        v.tooth.resize(nc);
    }
    parallel_samples(n, [&](int s) {
        Rng r(c.seed, static_cast<std::uint64_t>(s));
        Vars& v = vars[static_cast<std::size_t>(s)];
        for (int i = 0; i < nm; ++i) {
            v.px[i] = draw(r, c.position_tol);
            v.py[i] = draw(r, c.position_tol);
            v.bridge[i] = draw(r, c.bridge_gap_tol);
            v.flat[i] = draw(r, c.plate_flatness_tol);
            v.arbor[i] = draw(r, c.arbor_length_tol);
            v.hole_up[i] = draw(r, c.hole_diameter_tol);
            v.pivot_up[i] = draw(r, c.pivot_diameter_tol);
            v.hole_lo[i] = draw(r, c.hole_diameter_tol);
            v.pivot_lo[i] = draw(r, c.pivot_diameter_tol);
        }
        for (int i = 0; i < nc; ++i) v.tooth[i] = draw(r, c.tooth_thickness_tol);
    });

    ToleranceResult out;
    out.samples = n;
    out.seed = c.seed;
    std::vector<char> failing(static_cast<std::size_t>(n), 0);

    for (const model::MeshJoint& j : m.meshes) {
        const int ma = m.mobile_of(j.a), mb = m.mobile_of(j.b);
        if (ma < 0 || mb < 0) continue;
        const GearData ga = gear_data(m.components[j.a], j.module), gb = gear_data(m.components[j.b], j.module);
        const double module = j.module > 0 ? j.module : 0.5 * (ga.module + gb.module);
        const double alpha = 0.5 * (ga.alpha + gb.alpha);
        const double dx0 = m.mobiles[mb].center.x - m.mobiles[ma].center.x, dy0 = m.mobiles[mb].center.y - m.mobiles[ma].center.y;
        const double d0 = std::hypot(dx0, dy0);
        const double j0 = c.nominal_backlash_factor * module;

        MeshTolerance mt;
        mt.a = m.components[j.a].features.name;
        mt.b = m.components[j.b].features.name;
        mt.nominal_centre_distance = j.centre_distance;
        mt.nominal_backlash = j0;
        mt.nominal_contact_ratio = contact_ratio(ga, gb, d0, module, alpha);

        std::vector<double> cd(static_cast<std::size_t>(n)), bl(static_cast<std::size_t>(n)), cr(static_cast<std::size_t>(n));
        const std::vector<std::string> names = {"pivot position of " + m.mobiles[ma].name + " (x)", "pivot position of " + m.mobiles[ma].name + " (y)",
                                                "pivot position of " + m.mobiles[mb].name + " (x)", "pivot position of " + m.mobiles[mb].name + " (y)",
                                                "tooth thickness of " + mt.a, "tooth thickness of " + mt.b};
        std::vector<std::vector<double>> inputs(6, std::vector<double>(static_cast<std::size_t>(n)));
        int binds = 0, under = 0;
        for (int s = 0; s < n; ++s) {
            const Vars& v = vars[static_cast<std::size_t>(s)];
            const double dx = dx0 + v.px[mb] - v.px[ma], dy = dy0 + v.py[mb] - v.py[ma];
            const double centre = std::hypot(dx, dy);
            const double ds = v.tooth[j.a] + v.tooth[j.b];
            const double backlash = j0 + 2 * (centre - d0) * std::tan(alpha) - ds;
            const double eps = contact_ratio(ga, gb, centre, module, alpha);
            cd[static_cast<std::size_t>(s)] = centre;
            bl[static_cast<std::size_t>(s)] = backlash;
            cr[static_cast<std::size_t>(s)] = eps;
            inputs[0][static_cast<std::size_t>(s)] = v.px[ma];
            inputs[1][static_cast<std::size_t>(s)] = v.py[ma];
            inputs[2][static_cast<std::size_t>(s)] = v.px[mb];
            inputs[3][static_cast<std::size_t>(s)] = v.py[mb];
            inputs[4][static_cast<std::size_t>(s)] = v.tooth[j.a];
            inputs[5][static_cast<std::size_t>(s)] = v.tooth[j.b];
            const bool bind = backlash < c.min_backlash, weak = eps < c.min_contact_ratio;
            binds += bind;
            under += weak;
            if (bind || weak) failing[static_cast<std::size_t>(s)] = 1;
        }
        mt.centre_distance = stats(cd);
        mt.backlash = stats(bl);
        mt.contact_ratio = stats(cr);
        mt.bind_fraction = static_cast<double>(binds) / n;
        mt.under_engage_fraction = static_cast<double>(under) / n;
        mt.backlash_sensitivity = rank(names, inputs, bl);
        out.meshes.push_back(std::move(mt));
    }

    for (int i = 0; i < nm; ++i) {
        // A date ring rides on the mainplate, with no pivots to stack up.
        bool pivoted = false;
        for (int c : m.mobiles[i].components) pivoted |= m.components[c].role != model::Role::DateRing;
        if (!pivoted) continue;
        MobileTolerance mt;
        mt.mobile = m.mobiles[i].name;
        std::vector<double> end(static_cast<std::size_t>(n)), side(static_cast<std::size_t>(n));
        std::vector<std::vector<double>> in_end(3, std::vector<double>(static_cast<std::size_t>(n))), in_side(2, std::vector<double>(static_cast<std::size_t>(n)));
        int end_out = 0, side_out = 0;
        for (int s = 0; s < n; ++s) {
            const Vars& v = vars[static_cast<std::size_t>(s)];
            const double e = c.nominal_end_shake + v.bridge[i] + v.flat[i] - v.arbor[i];
            const double s1 = c.nominal_side_shake + v.hole_up[i] - v.pivot_up[i], s2 = c.nominal_side_shake + v.hole_lo[i] - v.pivot_lo[i];
            const double sh = std::min(s1, s2);
            end[static_cast<std::size_t>(s)] = e;
            side[static_cast<std::size_t>(s)] = sh;
            in_end[0][static_cast<std::size_t>(s)] = v.bridge[i];
            in_end[1][static_cast<std::size_t>(s)] = v.flat[i];
            in_end[2][static_cast<std::size_t>(s)] = v.arbor[i];
            in_side[0][static_cast<std::size_t>(s)] = s1 <= s2 ? v.hole_up[i] : v.hole_lo[i];
            in_side[1][static_cast<std::size_t>(s)] = s1 <= s2 ? v.pivot_up[i] : v.pivot_lo[i];
            const bool eo = e < c.end_shake_min || e > c.end_shake_max, so = sh < c.side_shake_min || sh > c.side_shake_max;
            end_out += eo;
            side_out += so;
            if (eo || so) failing[static_cast<std::size_t>(s)] = 1;
        }
        mt.end_shake = stats(end);
        mt.side_shake = stats(side);
        mt.end_shake_out_fraction = static_cast<double>(end_out) / n;
        mt.side_shake_out_fraction = static_cast<double>(side_out) / n;
        mt.end_shake_sensitivity = rank({"plate-to-bridge distance", "plate flatness", "arbor length"}, in_end, end);
        mt.side_shake_sensitivity = rank({"jewel hole diameter", "pivot diameter"}, in_side, side);
        out.mobiles.push_back(std::move(mt));
    }
    int bad = 0;
    for (char f : failing) bad += f;
    out.failing_fraction = static_cast<double>(bad) / n;
    return out;
}

}  // namespace ohs::manufacturing
