// SPDX-License-Identifier: Apache-2.0
#include "rigging/rigging.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>
#include <regex>

#include "core/error.hpp"

namespace ohs::rigging {

using model::Component;
using model::Movement;
using model::PartFeatures;
using model::Role;

namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

struct NameHint {
    std::string_view needle;
    Role role;
};

// Order matters: "escape_pinion" is a pinion, "escape_arbor" an arbor, "balance wheel" a balance.
constexpr NameHint kNameHints[] = {
    {"pinion", Role::Pinion},          {"arbor", Role::Arbor},
    {"staff", Role::Arbor},            {"axle", Role::Arbor},
    {"escape", Role::EscapeWheel},
    {"pallet", Role::PalletFork},      {"fork", Role::PalletFork},
    {"lever", Role::PalletFork},       {"hairspring", Role::Hairspring},
    {"balance spring", Role::Hairspring}, {"balance", Role::Balance},
    {"barrel", Role::Barrel},          {"mainspring", Role::Mainspring},
    {"roller", Role::Roller},          {"jewel", Role::Jewel},
    {"ruby", Role::Jewel},             {"screw", Role::Screw},
    {"bridge", Role::Bridge},          {"plate", Role::Plate},
    {"wheel", Role::Wheel},
};

bool is_toothed(const PartFeatures& p, const RigOptions& o) { return p.teeth >= o.min_teeth; }

double distance(model::Vec2 a, model::Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

}  // namespace

Classification classify(const PartFeatures& part, const RigOptions& options) {
    for (const RolePreset& preset : options.presets) {
        std::regex re;
        try {
            re = std::regex(preset.pattern, std::regex::icase | std::regex::ECMAScript);
        } catch (const std::regex_error&) {
            throw Error("invalid preset pattern '" + preset.pattern + "'");
        }
        if (std::regex_search(part.name, re)) return {preset.role, 1.0, "preset"};
    }

    const std::string name = lower(part.name);
    for (const NameHint& hint : kNameHints) {
        if (name.find(hint.needle) == std::string::npos) continue;
        double conf = 0.9;
        // A part called a wheel with no teeth, or a plate with teeth, is suspicious.
        if (model::is_toothed_role(hint.role) && !is_toothed(part, options)) conf = 0.5;
        if (!model::is_toothed_role(hint.role) && is_toothed(part, options) &&
            hint.role != Role::Balance && hint.role != Role::Roller) {
            conf = 0.5;
        }
        return {hint.role, conf, "name"};
    }

    if (is_toothed(part, options)) {
        const Role role = part.teeth <= 16 ? Role::Pinion : Role::Wheel;
        return {role, 0.5 + 0.4 * std::clamp(part.tooth_confidence, 0.0, 1.0), "geometry"};
    }
    const double extent = std::max(part.extent_x, part.extent_y);
    if (extent >= 10.0 && extent >= 4.0 * part.thickness()) return {Role::Plate, 0.6, "geometry"};
    if (part.outer_radius > 0 && part.outer_radius < 0.5 && part.thickness() > 3.0 * part.outer_radius) {
        return {Role::Arbor, 0.6, "geometry"};
    }
    return {Role::Unknown, 0.0, "geometry"};
}

namespace {

// Pairs of toothed parts that mesh: the module follows from the centre distance, then each gear's
// addendum and dedendum and the tip-to-root clearance must be plausible for that module.
bool plausible_mesh(const PartFeatures& a, const PartFeatures& b, double centre, const RigOptions& o,
                    double& module, double& confidence) {
    module = 2.0 * centre / (a.teeth + b.teeth);
    auto gear_ok = [&](const PartFeatures& f) {
        const double addendum = f.outer_radius / module - f.teeth / 2.0;
        if (addendum < o.min_addendum || addendum > o.max_addendum) return false;
        if (f.root_radius <= 0.0) return true;  // rig written before root radii were recorded
        const double dedendum = f.teeth / 2.0 - f.root_radius / module;
        return dedendum >= o.min_dedendum && dedendum <= o.max_dedendum;
    };
    if (!gear_ok(a) || !gear_ok(b)) return false;

    confidence = 1.0;
    if (a.root_radius > 0.0 && b.root_radius > 0.0) {
        const double c1 = (centre - (a.outer_radius + b.root_radius)) / module;
        const double c2 = (centre - (b.outer_radius + a.root_radius)) / module;
        for (double c : {c1, c2}) {
            if (c < o.min_clearance || c > o.max_clearance) return false;
        }
        const double spread = std::max(o.ideal_clearance - o.min_clearance, o.max_clearance - o.ideal_clearance);
        confidence = 1.0 - 0.5 * (std::abs(c1 - o.ideal_clearance) + std::abs(c2 - o.ideal_clearance)) / spread;
    }
    confidence = std::clamp(confidence, 0.0, 1.0);
    return true;
}

void fit_gears(Movement& m, const RigOptions& o) {
    std::vector<double> module_sum(m.components.size(), 0.0);
    std::vector<int> module_count(m.components.size(), 0);
    for (const model::MeshJoint& j : m.meshes) {
        for (int idx : {j.a, j.b}) {
            module_sum[idx] += j.module;
            ++module_count[idx];
        }
    }
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        Component& c = m.components[i];
        c.gear.reset();
        const PartFeatures& f = c.features;
        if (!model::is_toothed_role(c.role) || f.teeth < o.min_teeth) continue;

        model::GearFit g;
        if (module_count[i] > 0) {
            g.module = module_sum[i] / module_count[i];
            g.module_source = "mesh";
        } else {
            g.module = model::module_from_tip(f.outer_radius, f.teeth);
            g.module_source = "tip";
        }
        g.addendum = f.outer_radius / g.module - f.teeth / 2.0;
        g.dedendum = f.root_radius > 0 ? f.teeth / 2.0 - f.root_radius / g.module : 0.0;
        const double pitch_radius = g.module * f.teeth / 2.0;
        if (f.flank_rms >= 0.0 && f.flank_base_radius > 0.0 && f.flank_base_radius < pitch_radius) {
            g.base_radius = f.flank_base_radius;
            g.pressure_angle_deg = std::acos(g.base_radius / pitch_radius) * 180.0 / std::numbers::pi;
            g.profile = f.flank_rms <= o.involute_rms_modules * g.module ? "involute" : "unknown";
        } else {
            g.profile = "unknown";
        }
        c.gear = g;
    }
}

}  // namespace

void infer_structure(Movement& m, const RigOptions& options) {
    std::vector<model::MeshJoint> manual;
    for (const model::MeshJoint& j : m.meshes) {
        if (j.manual) manual.push_back(j);
    }
    m.mobiles.clear();
    m.meshes.clear();
    const int n = static_cast<int>(m.components.size());

    // Group rotating parts whose axes coincide into mobiles (greedy single linkage).
    std::vector<int> group(n, -1);
    int groups = 0;
    for (int i = 0; i < n; ++i) {
        if (!model::is_rotating_role(m.components[i].role)) continue;
        if (group[i] < 0) group[i] = groups++;
        for (int j = i + 1; j < n; ++j) {
            if (!model::is_rotating_role(m.components[j].role)) continue;
            if (distance(m.components[i].features.center, m.components[j].features.center) >
                options.coaxial_tolerance) {
                continue;
            }
            if (group[j] < 0) {
                group[j] = group[i];
            } else if (group[j] != group[i]) {
                const int from = group[j];
                const int to = group[i];
                for (int& g : group) {
                    if (g == from) g = to;
                }
            }
        }
    }
    std::vector<int> renumber(groups, -1);
    for (int i = 0; i < n; ++i) {
        if (group[i] < 0) continue;
        if (renumber[group[i]] < 0) {
            renumber[group[i]] = static_cast<int>(m.mobiles.size());
            model::Mobile mob;
            mob.name = "M" + std::to_string(m.mobiles.size() + 1);
            m.mobiles.push_back(std::move(mob));
        }
        m.mobiles[renumber[group[i]]].components.push_back(i);
    }
    for (model::Mobile& mob : m.mobiles) {
        for (int idx : mob.components) {
            mob.center.x += m.components[idx].features.center.x / mob.components.size();
            mob.center.y += m.components[idx].features.center.y / mob.components.size();
        }
    }

    for (int i = 0; i < n; ++i) {
        const Component& ci = m.components[i];
        if (!model::is_toothed_role(ci.role) || !is_toothed(ci.features, options)) continue;
        for (int j = i + 1; j < n; ++j) {
            const Component& cj = m.components[j];
            if (!model::is_toothed_role(cj.role) || !is_toothed(cj.features, options)) continue;
            if (m.mobile_of(i) == m.mobile_of(j)) continue;
            const double overlap = std::min(ci.features.z_max, cj.features.z_max) -
                                   std::max(ci.features.z_min, cj.features.z_min);
            if (overlap <= 0.0) continue;

            const double centre = distance(ci.features.center, cj.features.center);
            double module = 0.0;
            double confidence = 0.0;
            if (plausible_mesh(ci.features, cj.features, centre, options, module, confidence)) {
                m.meshes.push_back({i, j, centre, module, confidence});
            }
        }
    }
    // Declared meshes survive re-inference (component indices do not change) when both parts are
    // still on rotating mobiles; one the geometry also found is just marked as declared.
    for (const model::MeshJoint& j : manual) {
        if (m.mobile_of(j.a) < 0 || m.mobile_of(j.b) < 0 || m.mobile_of(j.a) == m.mobile_of(j.b)) continue;
        bool found = false;
        for (model::MeshJoint& existing : m.meshes) {
            if ((existing.a == j.a && existing.b == j.b) || (existing.a == j.b && existing.b == j.a)) {
                existing.manual = true;
                found = true;
            }
        }
        if (!found) m.meshes.push_back(j);
    }
    fit_gears(m, options);
}

void add_manual_mesh(Movement& m, const std::string& a, const std::string& b, const RigOptions& options) {
    const int ia = m.find_component(a);
    const int ib = m.find_component(b);
    if (ia < 0) throw Error("no component named '" + a + "'");
    if (ib < 0) throw Error("no component named '" + b + "'");
    if (ia == ib) throw Error("a part cannot mesh with itself");
    if (m.mobile_of(ia) < 0 || m.mobile_of(ib) < 0) {
        throw Error("both parts must be on rotating mobiles (set a role such as wheel, pinion or barrel first)");
    }
    if (m.mobile_of(ia) == m.mobile_of(ib)) throw Error("both parts are on the same mobile, so they cannot mesh");
    const int za = m.components[ia].features.teeth, zb = m.components[ib].features.teeth;
    if (za <= 0 || zb <= 0) throw Error("both parts need a tooth count; '" + (za <= 0 ? a : b) + "' has none detected");
    for (const model::MeshJoint& j : m.meshes) {
        if ((j.a == ia && j.b == ib) || (j.a == ib && j.b == ia)) return;  // already meshed
    }
    const double centre = distance(m.components[ia].features.center, m.components[ib].features.center);
    m.meshes.push_back({ia, ib, centre, 2.0 * centre / (za + zb), 1.0, true});
    fit_gears(m, options);
}

void set_role(Movement& m, const std::string& name, Role role, const RigOptions& options) {
    const int idx = m.find_component(name);
    if (idx < 0) throw Error("no component named '" + name + "'");
    m.components[idx].role = role;
    m.components[idx].role_confidence = 1.0;
    m.components[idx].role_source = "user";
    infer_structure(m, options);
}

Movement auto_rig(std::string movement_name, std::vector<PartFeatures> parts, const RigOptions& options) {
    Movement m;
    m.name = std::move(movement_name);
    m.beat_rate_vph = options.beat_rate_vph;
    for (PartFeatures& p : parts) {
        const Classification c = classify(p, options);
        m.components.push_back({std::move(p), std::nullopt, c.role, c.confidence, c.source});
    }
    infer_structure(m, options);
    return m;
}

std::vector<Issue> check_rig(const Movement& m, double low_confidence) {
    std::vector<Issue> issues;
    auto add = [&issues](Severity s, std::string msg) { issues.push_back({s, std::move(msg)}); };

    if (m.components.empty()) add(Severity::Error, "the movement has no components");

    bool has_barrel = false;
    bool has_escape = false;
    for (const Component& c : m.components) {
        const std::string& name = c.features.name;
        has_barrel |= c.role == Role::Barrel;
        has_escape |= c.role == Role::EscapeWheel;
        if (c.role == Role::Unknown) {
            add(Severity::Warning, "'" + name + "' could not be classified");
        } else if (c.role_confidence < low_confidence) {
            add(Severity::Warning, "'" + name + "' classified as " + std::string(to_string(c.role)) +
                                       " with low confidence " + std::to_string(c.role_confidence));
        }
    }
    if (!has_barrel) add(Severity::Warning, "no barrel found; the train has no driving root");
    if (!has_escape) add(Severity::Warning, "no escape wheel found");

    std::vector<bool> meshed(m.components.size(), false);
    for (const model::MeshJoint& j : m.meshes) {
        meshed[j.a] = meshed[j.b] = true;
        if (j.confidence < low_confidence) {
            add(Severity::Warning, "mesh '" + m.components[j.a].features.name + "' / '" +
                                       m.components[j.b].features.name + "' has low confidence " +
                                       std::to_string(j.confidence));
        }
    }
    if (m.meshes.empty()) add(Severity::Error, "no gear meshes were inferred");
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const Component& c = m.components[i];
        if (!model::is_toothed_role(c.role) || meshed[i]) continue;
        if (c.role == Role::EscapeWheel) {  // its partner is the pallet fork, not inferred yet
            add(Severity::Info, "escape wheel '" + c.features.name + "' has no pallet fork joint (not inferred in M0)");
        } else {
            add(Severity::Warning, "toothed part '" + c.features.name + "' meshes with nothing");
        }
    }
    return issues;
}

}  // namespace ohs::rigging
