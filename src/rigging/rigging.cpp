// SPDX-License-Identifier: Apache-2.0
#include "rigging/rigging.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
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

Movement auto_rig(std::string movement_name, std::vector<PartFeatures> parts,
                  const RigOptions& options) {
    Movement m;
    m.name = std::move(movement_name);
    m.beat_rate_vph = options.beat_rate_vph;

    for (PartFeatures& p : parts) {
        const Classification c = classify(p, options);
        m.components.push_back({std::move(p), c.role, c.confidence, c.source});
    }
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
        model::Mobile& mob = m.mobiles[renumber[group[i]]];
        mob.components.push_back(i);
    }
    for (model::Mobile& mob : m.mobiles) {
        for (int idx : mob.components) {
            mob.center.x += m.components[idx].features.center.x / mob.components.size();
            mob.center.y += m.components[idx].features.center.y / mob.components.size();
        }
    }

    // Meshes: toothed parts of different mobiles with equal module whose centre distance
    // equals the sum of pitch radii, and which overlap axially.
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

            const double mi = model::module_from_tip(ci.features.outer_radius, ci.features.teeth);
            const double mj = model::module_from_tip(cj.features.outer_radius, cj.features.teeth);
            const double module = 0.5 * (mi + mj);
            const double module_err = std::abs(mi - mj) / module;
            const double expected = module * (ci.features.teeth + cj.features.teeth) / 2.0;
            const double actual = distance(ci.features.center, cj.features.center);
            const double centre_err = std::abs(actual - expected) / expected;
            if (module_err > options.module_tolerance || centre_err > options.centre_tolerance) continue;

            const double conf = 1.0 - 0.5 * (module_err / options.module_tolerance +
                                             centre_err / options.centre_tolerance);
            m.meshes.push_back({i, j, actual, module, std::clamp(conf, 0.0, 1.0)});
        }
    }
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
