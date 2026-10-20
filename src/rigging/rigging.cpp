// SPDX-License-Identifier: Apache-2.0
#include "rigging/rigging.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <regex>
#include <unordered_map>

#include "core/error.hpp"
#include "model/followers.hpp"

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
    {"hairspring", Role::Hairspring},
    {"balance spring", Role::Hairspring}, {"balance", Role::Balance},
    {"barrel", Role::Barrel},          {"mainspring", Role::Mainspring},
    {"roller", Role::Roller},          {"jewel", Role::Jewel},
    {"ruby", Role::Jewel},             {"screw", Role::Screw},
    {"bridge", Role::Bridge},
    // The keyless work and its small parts. After pallet, escape, barrel and the springs of the train
    // ("mainspring", "hairspring"), so those keep their roles, and after screw ("spring screw") and bridge
    // ("bearing bridge"); a bare "lever" is a lever of the keyless work, not the pallet lever.
    {"stem", Role::Stem},
    {"spring", Role::Spring},          {"lever", Role::Lever},
    {"yoke", Role::Lever},             {"click", Role::Lever},
    {"pawl", Role::Lever},             {"jumper", Role::Lever},
    {"bearing", Role::Bearing},        {"bush", Role::Bearing},
    {"sleeve", Role::Bearing},         {"tube", Role::Bearing},
    // After screw and bridge ("rotor screw", "rotor bridge"), before plate ("rotor plate").
    {"rotor", Role::Rotor},            {"oscillating", Role::Rotor},
    // The calendar ring, by its names in English and French. The driving wheel, finger and jumper are
    // other parts (see is_date_helper), so only a "wheel", "ring" or "disc" of the date is the ring.
    {"date wheel", Role::DateRing},    {"date ring", Role::DateRing},
    {"date disc", Role::DateRing},     {"date disk", Role::DateRing},
    {"date indicator", Role::DateRing}, {"calendar ring", Role::DateRing},
    {"quantieme", Role::DateRing},     {"quantième", Role::DateRing},
    {"plate", Role::Plate},
    {"wheel", Role::Wheel},
};

bool is_toothed(const PartFeatures& p, const RigOptions& o) { return p.teeth >= o.min_teeth; }

// "date wheel jumper" and "date wheel finger" hold or push the ring; they are not it.
bool is_date_helper(const std::string& name) {
    return name.find("jumper") != std::string::npos || name.find("finger") != std::string::npos ||
           name.find("driv") != std::string::npos || name.find("click") != std::string::npos;
}

// "stem" as a word, so that "system" and "stemmed" are not stems.
bool has_word(const std::string& text, std::string_view word) {
    for (std::size_t at = text.find(word); at != std::string::npos; at = text.find(word, at + 1)) {
        const bool before = at == 0 || !std::isalpha(static_cast<unsigned char>(text[at - 1]));
        const std::size_t end = at + word.size();
        const bool after = end >= text.size() || !std::isalpha(static_cast<unsigned char>(text[end]));
        if (before && after) return true;
    }
    return false;
}

double distance(model::Vec2 a, model::Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

// A long round rod lying in the plane of the movement: the winding stem.
bool is_stem_shape(const PartFeatures& p) {
    return p.length >= 8.0 && p.length >= 8.0 * p.width && p.height >= 0.6 * p.width && std::abs(p.axis[2]) <= 0.5;
}

// A short hollow cylinder, about as long as it is wide or longer, that is not a thin washer: a bush or tube.
bool is_tube_shape(const PartFeatures& p) {
    return p.outer_radius > 0.0 && p.outer_radius <= 2.5 && p.bore_radius >= 0.25 * p.outer_radius &&
           p.thickness() >= 1.2 * p.outer_radius && p.thickness() <= 12.0 * p.outer_radius &&
           (p.length <= 0.0 || p.length <= 6.0 * p.outer_radius);  // not a wide hollow disc whose centre of mass is a hub
}

// A thin flat part with a hole to turn on, the size of a lever or a spring.
bool is_flat_small(const PartFeatures& p) {
    return p.length >= 2.5 && p.length <= 12.0 && p.height <= 0.6 && p.height <= 0.4 * p.width;
}

}  // namespace

Classification classify(const PartFeatures& part, const RigOptions& options) {
    // Compiling a regex costs far more than matching it, and every part is matched against every preset: compile
    // each pattern once per thread.
    static thread_local std::unordered_map<std::string, std::regex> compiled;
    for (const RolePreset& preset : options.presets) {
        auto found = compiled.find(preset.pattern);
        if (found == compiled.end()) {
            try {
                found = compiled.emplace(preset.pattern, std::regex(preset.pattern, std::regex::icase | std::regex::ECMAScript)).first;
            } catch (const std::regex_error&) {
                throw Error("invalid preset pattern '" + preset.pattern + "'");
            }
        }
        if (std::regex_search(part.name, found->second)) return {preset.role, 1.0, "preset"};
    }

    const std::string name = lower(part.name);
    // The date hints are several words, which CAD names often join with "_" or "-".
    std::string spaced = name;
    std::replace_if(spaced.begin(), spaced.end(), [](char c) { return c == '_' || c == '-'; }, ' ');
    for (const NameHint& hint : kNameHints) {
        if ((hint.role == Role::DateRing ? spaced : name).find(hint.needle) == std::string::npos) continue;
        if (hint.role == Role::DateRing && is_date_helper(name)) continue;
        if (hint.role == Role::Stem && !has_word(name, hint.needle)) continue;
        double conf = 0.9;
        // A part called a wheel with no teeth, or a plate with teeth, is suspicious.
        if (model::is_toothed_role(hint.role) && !is_toothed(part, options)) conf = 0.5;
        if (!model::is_toothed_role(hint.role) && is_toothed(part, options) &&
            hint.role != Role::Balance && hint.role != Role::Roller) {
            conf = 0.5;
        }
        // A date ring has its teeth on the inside, or outside on a few designs; with neither found it is a guess.
        if (hint.role == Role::DateRing && part.inner_teeth < options.min_teeth && !is_toothed(part, options)) conf = 0.7;
        return {hint.role, conf, "name"};
    }

    if (is_toothed(part, options)) {
        const Role role = part.teeth <= 16 ? Role::Pinion : Role::Wheel;
        return {role, 0.5 + 0.4 * std::clamp(part.tooth_confidence, 0.0, 1.0), "geometry"};
    }
    // A ring with teeth on the inside and none outside: the calendar ring.
    if (part.inner_teeth >= options.min_teeth) return {Role::DateRing, 0.7, "geometry"};
    if (is_stem_shape(part)) return {Role::Stem, 0.6, "geometry"};
    const double extent = std::max(part.extent_x, part.extent_y);
    if (extent >= 10.0 && extent >= 4.0 * part.thickness()) return {Role::Plate, 0.6, "geometry"};
    if (part.outer_radius > 0 && part.outer_radius < 0.5 && part.thickness() > 3.0 * part.outer_radius) {
        return {Role::Arbor, 0.6, "geometry"};
    }
    if (is_tube_shape(part)) return {Role::Bearing, 0.6, "geometry"};
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
        // A flank fit whose pressure angle the gear code would refuse (45 degrees or more) is a bad fit, not a
        // gear: a module taken from a doubtful mesh can put the pitch circle far from the real one.
        const double fitted_angle = f.flank_base_radius > 0.0 && f.flank_base_radius < pitch_radius
                                        ? std::acos(f.flank_base_radius / pitch_radius) * 180.0 / std::numbers::pi
                                        : 0.0;
        if (f.flank_rms >= 0.0 && fitted_angle > 0.0 && fitted_angle < model::kMaxPressureAngleDeg) {
            g.base_radius = f.flank_base_radius;
            g.pressure_angle_deg = fitted_angle;
            g.profile = f.flank_rms <= o.involute_rms_modules * g.module ? "involute" : "unknown";
        } else {
            g.profile = "unknown";
        }
        if (g.profile == "unknown" && f.flank_cycloid_rms >= 0.0 && f.flank_cycloid_rms <= o.cycloid_rms_modules * g.module &&
            std::abs(f.flank_cycloid_pitch_radius - pitch_radius) <= o.cycloid_pitch_tolerance * pitch_radius) {
            g.profile = "cycloidal";
            g.rolling_radius = f.flank_cycloid_rolling_radius / g.module;
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
    std::vector<std::pair<int, int>> winding;  // meshes the user took out of the train: the flag outlives the meshes
    for (const model::MeshJoint& j : m.meshes) {
        if (j.winding) winding.emplace_back(j.a, j.b);
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
        // A date ring is concentric with the motion work but turns on its own, so it joins nothing.
        if (m.components[i].role == Role::DateRing) continue;
        for (int j = i + 1; j < n; ++j) {
            if (!model::is_rotating_role(m.components[j].role) || m.components[j].role == Role::DateRing) continue;
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

    const std::vector<int> mobile_at = m.mobile_index();
    for (int i = 0; i < n; ++i) {
        const Component& ci = m.components[i];
        if (!model::is_toothed_role(ci.role) || !is_toothed(ci.features, options)) continue;
        for (int j = i + 1; j < n; ++j) {
            const Component& cj = m.components[j];
            if (!model::is_toothed_role(cj.role) || !is_toothed(cj.features, options)) continue;
            if (mobile_at[static_cast<std::size_t>(i)] == mobile_at[static_cast<std::size_t>(j)]) continue;
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
    for (model::MeshJoint& j : m.meshes) {
        j.winding = std::any_of(winding.begin(), winding.end(), [&j](const std::pair<int, int>& w) {
            return (w.first == j.a && w.second == j.b) || (w.first == j.b && w.second == j.a);
        });
    }
    fit_gears(m, options);
}

void remove_manual_mesh(Movement& m, const std::string& a, const std::string& b, const RigOptions& options) {
    const int ia = m.find_component(a);
    const int ib = m.find_component(b);
    if (ia < 0) throw Error("no component named '" + a + "'");
    if (ib < 0) throw Error("no component named '" + b + "'");
    const auto it = std::find_if(m.meshes.begin(), m.meshes.end(), [&](const model::MeshJoint& j) {
        return (j.a == ia && j.b == ib) || (j.a == ib && j.b == ia);
    });
    if (it == m.meshes.end()) throw Error("'" + a + "' and '" + b + "' are not meshed");
    if (!it->manual) {
        throw Error("that mesh was found from the geometry, so it would come back: use `ohs rig winding` to leave it out of the train");
    }
    m.meshes.erase(it);
    infer_structure(m, options);  // the other meshes and the gear fits, without it
}

void set_mesh_winding(Movement& m, const std::string& a, const std::string& b, bool winding) {
    const int ia = m.find_component(a);
    const int ib = m.find_component(b);
    if (ia < 0) throw Error("no component named '" + a + "'");
    if (ib < 0) throw Error("no component named '" + b + "'");
    for (model::MeshJoint& j : m.meshes) {
        if ((j.a == ia && j.b == ib) || (j.a == ib && j.b == ia)) {
            j.winding = winding;
            return;
        }
    }
    throw Error("'" + a + "' and '" + b + "' are not meshed");
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

void set_roles(Movement& m, const std::vector<std::pair<std::string, Role>>& roles, const RigOptions& options) {
    std::vector<int> indices;
    for (const auto& [name, role] : roles) {
        const int idx = m.find_component(name);
        if (idx < 0) throw Error("no component named '" + name + "'");
        indices.push_back(idx);
    }
    for (std::size_t i = 0; i < roles.size(); ++i) {
        Component& c = m.components[static_cast<std::size_t>(indices[i])];
        c.role = roles[i].second;
        c.role_confidence = 1.0;
        c.role_source = "user";
    }
    infer_structure(m, options);
}

void set_role(Movement& m, const std::string& name, Role role, const RigOptions& options) {
    set_roles(m, {{name, role}}, options);
}

void spot_rotors(std::vector<Component>& components) {
    // An automatic's rotor is large and thin, so on its own it passes for a plate. Against the rest of the
    // movement it does not: it spans about the whole movement, like the main plate, but it covers only
    // about half of it (a half disc), so its mass sits well to one side of the plate's. Comparing areas
    // (volume over thickness) says which of such a pair is the rotor whichever way the CAD file has it
    // turned. A wide bridge over one side of the movement looks the same, so the result is only a
    // suggestion (confidence 0.5) to check.
    double width = 0.0;
    for (const Component& c : components) width = std::max({width, c.features.extent_x, c.features.extent_y});
    if (width <= 0.0) return;
    const auto spans = [width](const PartFeatures& f) { return std::max(f.extent_x, f.extent_y) >= 0.8 * width; };
    const auto area = [](const PartFeatures& f) { return f.thickness() > 0 ? f.volume / f.thickness() : 0.0; };
    for (Component& c : components) {
        if (c.role != Role::Plate || c.role_source != "geometry" || !spans(c.features)) continue;
        for (const Component& other : components) {
            if (&other == &c || !spans(other.features)) continue;
            if (distance(c.features.center, other.features.center) >= 0.2 * width &&
                area(c.features) < 0.75 * area(other.features)) {
                c.role = Role::Rotor;
                c.role_confidence = 0.5;
                break;
            }
        }
    }
}

void spot_small_parts(std::vector<Component>& components) {
    // The parts of the keyless work and of the small works around the train have part numbers for names, so
    // only their shape and what they touch say what they are. A lever turns on a hole, and its far end rests on
    // the teeth of a ratchet, a wheel or a date ring: that is checked first, as a spring is found by pressing on a
    // lever or on teeth. A bent wire is a spring on its shape alone: little metal for its length, no holes.
    // All are suggestions (confidence 0.5 or less) to check.
    const auto unknown = [](const Component& c) { return c.role == Role::Unknown && c.role_source == "geometry"; };
    for (std::size_t i = 0; i < components.size(); ++i) {
        Component& c = components[i];
        if (!unknown(c) || !is_flat_small(c.features) || c.features.holes.empty()) continue;
        if (model::lever_contact(components, static_cast<int>(i))) {
            c.role = Role::Lever;
            c.role_confidence = 0.5;
        }
    }
    for (std::size_t i = 0; i < components.size(); ++i) {
        Component& c = components[i];
        if (!unknown(c)) continue;
        const PartFeatures& f = c.features;
        const bool wire = f.length >= 2.5 && f.volume >= 0.3 && f.volume <= 4.0 && f.volume <= 0.9 * f.length &&
                          f.holes.empty() && f.volume <= 0.5 * f.length * f.width * f.height;
        const bool strip = is_flat_small(f) && f.length >= 3.5 * f.width && model::spring_flex(components, static_cast<int>(i));
        if (wire || strip) {
            c.role = Role::Spring;
            c.role_confidence = wire ? 0.4 : 0.5;
        }
    }
}

Movement auto_rig(std::string movement_name, std::vector<PartFeatures> parts, const RigOptions& options) {
    Movement m;
    m.name = std::move(movement_name);
    m.beat_rate_vph = options.beat_rate_vph;
    for (PartFeatures& p : parts) {
        const Classification c = classify(p, options);
        m.components.push_back({std::move(p), std::nullopt, c.role, c.confidence, c.source, {}});
    }
    spot_rotors(m.components);
    spot_small_parts(m.components);
    infer_structure(m, options);
    return m;
}

RerigReport re_rig(Movement& m, const RigOptions& options, bool keep_user) {
    RerigReport report;
    report.mobiles_before = static_cast<int>(m.mobiles.size());
    report.meshes_before = static_cast<int>(m.meshes.size());
    std::vector<Role> before;
    for (Component& c : m.components) {
        before.push_back(c.role);
        // A preset is not stored in the project, so a role it gave is a choice to keep like the user's own.
        if (keep_user && (c.role_source == "user" || c.role_source == "preset")) continue;
        const Classification k = classify(c.features, options);
        c.role = k.role;
        c.role_confidence = k.confidence;
        c.role_source = k.source;
    }
    if (!keep_user) {
        m.meshes.erase(std::remove_if(m.meshes.begin(), m.meshes.end(), [](const model::MeshJoint& j) { return j.manual; }),
                       m.meshes.end());
    }
    spot_rotors(m.components);
    spot_small_parts(m.components);
    infer_structure(m, options);
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const Component& c = m.components[i];
        if (c.role != before[i]) report.role_changes.push_back({c.features.name, before[i], c.role, c.role_confidence});
    }
    report.mobiles_after = static_cast<int>(m.mobiles.size());
    report.meshes_after = static_cast<int>(m.meshes.size());
    return report;
}

std::vector<MeshSuggestion> suggest_meshes(const Movement& m, const RigOptions& o) {
    const std::size_t n = m.components.size();
    std::vector<bool> meshed(n, false);
    for (const model::MeshJoint& j : m.meshes) meshed[static_cast<std::size_t>(j.a)] = meshed[static_cast<std::size_t>(j.b)] = true;
    const auto already = [&m](int a, int b) {
        return std::any_of(m.meshes.begin(), m.meshes.end(), [a, b](const model::MeshJoint& j) {
            return (j.a == a && j.b == b) || (j.a == b && j.b == a);
        });
    };
    std::vector<MeshSuggestion> out;
    const std::vector<int> mobile_at = m.mobile_index();
    for (std::size_t i = 0; i < n; ++i) {
        const Component& ci = m.components[i];
        if (!model::is_toothed_role(ci.role) || !is_toothed(ci.features, o) || mobile_at[i] < 0) continue;
        for (std::size_t j = i + 1; j < n; ++j) {
            const Component& cj = m.components[j];
            if (!model::is_toothed_role(cj.role) || !is_toothed(cj.features, o) || mobile_at[j] < 0) continue;
            if (meshed[i] && meshed[j]) continue;  // nothing is left to rig for either
            if (mobile_at[i] == mobile_at[j]) continue;
            if (already(static_cast<int>(i), static_cast<int>(j))) continue;
            const PartFeatures& fi = ci.features;
            const PartFeatures& fj = cj.features;
            if (std::min(fi.z_max, fj.z_max) - std::max(fi.z_min, fj.z_min) <= 0.0) continue;
            const double centre = distance(fi.center, fj.center);
            if (centre <= 0.0) continue;
            // The strict test failed, so only ask that each gear's tips sit a tooth height or so from where
            // the module of the centre distance puts them.
            const double module = 2.0 * centre / (fi.teeth + fj.teeth);
            const double ai = fi.outer_radius / module - fi.teeth / 2.0;
            const double aj = fj.outer_radius / module - fj.teeth / 2.0;
            if (ai < 0.4 || ai > 2.2 || aj < 0.4 || aj > 2.2) continue;
            const double off = std::max(std::abs(ai - 1.0), std::abs(aj - 1.0));
            out.push_back({static_cast<int>(i), static_cast<int>(j), centre, module, std::clamp(0.5 * (1.0 - off / 1.2), 0.05, 0.5)});
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const MeshSuggestion& a, const MeshSuggestion& b) { return a.confidence > b.confidence; });
    return out;
}

namespace {

// Breadth-first from `start` over the meshes between mobiles: the parent of each mobile reached (start's
// is itself), -1 for the others, and the deepest mobile reached.
struct Reach {
    std::vector<int> parent;
    int deepest{-1};
};

Reach reach_from(const std::vector<std::vector<int>>& adjacent, int start) {
    Reach r;
    r.parent.assign(adjacent.size(), -1);
    if (start < 0) return r;
    std::vector<int> depth(adjacent.size(), 0), queue{start};
    r.parent[static_cast<std::size_t>(start)] = start;
    r.deepest = start;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int at = queue[head];
        for (int next : adjacent[static_cast<std::size_t>(at)]) {
            if (r.parent[static_cast<std::size_t>(next)] >= 0) continue;
            r.parent[static_cast<std::size_t>(next)] = at;
            depth[static_cast<std::size_t>(next)] = depth[static_cast<std::size_t>(at)] + 1;
            if (depth[static_cast<std::size_t>(next)] > depth[static_cast<std::size_t>(r.deepest)]) r.deepest = next;
            queue.push_back(next);
        }
    }
    return r;
}

// The way from the start of a search to `to`, start first.
std::vector<int> way_to(const Reach& r, int to) {
    std::vector<int> way;
    if (to < 0 || r.parent[static_cast<std::size_t>(to)] < 0) return way;
    for (int at = to;; at = r.parent[static_cast<std::size_t>(at)]) {
        way.push_back(at);
        if (r.parent[static_cast<std::size_t>(at)] == at) break;
    }
    std::reverse(way.begin(), way.end());
    return way;
}

}  // namespace

TrainChain train_chain(const Movement& m) {
    TrainChain chain;
    const std::size_t nm = m.mobiles.size();
    for (std::size_t i = 0; i < nm; ++i) {
        for (int idx : m.mobiles[i].components) {
            const Role role = m.components[static_cast<std::size_t>(idx)].role;
            if (role == Role::Barrel && chain.barrel < 0) chain.barrel = static_cast<int>(i);
            if (role == Role::EscapeWheel && chain.escape < 0) chain.escape = static_cast<int>(i);
        }
    }
    std::vector<std::vector<int>> adjacent(nm);
    for (const model::MeshJoint& j : m.meshes) {
        const int a = m.mobile_of(j.a), b = m.mobile_of(j.b);
        if (a < 0 || b < 0 || a == b || !m.in_train(j)) continue;
        adjacent[static_cast<std::size_t>(a)].push_back(b);
        adjacent[static_cast<std::size_t>(b)].push_back(a);
    }
    const Reach from_barrel = reach_from(adjacent, chain.barrel);
    const Reach from_escape = reach_from(adjacent, chain.escape);
    chain.fed.assign(nm, 0);
    for (std::size_t i = 0; i < nm; ++i) chain.fed[i] = from_barrel.parent[i] >= 0;
    if (chain.barrel >= 0 && chain.escape >= 0 && chain.fed[static_cast<std::size_t>(chain.escape)]) {
        chain.connected = true;
        chain.path = way_to(from_barrel, chain.escape);
        return chain;
    }
    chain.from_barrel = way_to(from_barrel, from_barrel.deepest);
    chain.from_escape = way_to(from_escape, from_escape.deepest);
    std::reverse(chain.from_escape.begin(), chain.from_escape.end());  // ends at the escape wheel
    for (std::size_t i = 0; i < nm; ++i) {
        if (from_barrel.parent[i] >= 0 || from_escape.parent[i] >= 0) continue;
        const bool geared = std::any_of(m.mobiles[i].components.begin(), m.mobiles[i].components.end(), [&m](int idx) {
            const Component& c = m.components[static_cast<std::size_t>(idx)];
            return model::is_toothed_role(c.role) && c.features.teeth > 0;
        });
        if (geared) chain.unreached.push_back(static_cast<int>(i));
    }
    return chain;
}

std::string describe_mobile(const Movement& m, int mobile) {
    const model::Mobile& mob = m.mobiles[static_cast<std::size_t>(mobile)];
    for (int idx : mob.components) {
        const Component& c = m.components[static_cast<std::size_t>(idx)];
        if (model::is_toothed_role(c.role)) return mob.name + " (" + c.display_name() + ")";
    }
    return mob.name;
}

std::string describe_chain(const Movement& m, const TrainChain& chain) {
    if (chain.barrel < 0) return "no barrel found, so the train has no driving root";
    if (chain.escape < 0) return "no escape wheel found";
    const auto join = [&m](const std::vector<int>& way) {
        std::string s;
        for (int mobile : way) s += (s.empty() ? "" : " -> ") + describe_mobile(m, mobile);
        return s;
    };
    if (chain.connected) return join(chain.path);
    std::string s = join(chain.from_barrel) + "  [no mesh]  " + join(chain.from_escape);
    if (!chain.unreached.empty()) {
        s += "; joined to neither end:";
        for (int mobile : chain.unreached) s += " " + describe_mobile(m, mobile);
    }
    return s;
}

namespace {

std::string number(double v, const char* format) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, format, v);
    return buffer;
}

}  // namespace

TrainWalk walk_train(const Movement& m, const TrainChain& chain, const std::vector<MeshSuggestion>& suggestions) {
    TrainWalk walk;
    const auto name = [&m](int component) { return m.components[static_cast<std::size_t>(component)].display_name(); };
    const auto teeth = [&m](int component) { return m.components[static_cast<std::size_t>(component)].features.teeth; };
    const auto toothed_in = [&m](int mobile, int except) {
        for (int idx : m.mobiles[static_cast<std::size_t>(mobile)].components) {
            if (idx != except && model::is_toothed_role(m.components[static_cast<std::size_t>(idx)].role)) return idx;
        }
        return -1;
    };
    if (chain.barrel < 0) {
        WalkStep step;
        step.kind = WalkStep::Kind::Missing;
        step.needs_role = Role::Barrel;
        step.text = "No barrel found, so the train has nothing to start from. Pick the barrel.";
        walk.steps.push_back(std::move(step));
        return walk;
    }

    const std::vector<int>& way = chain.connected ? chain.path : chain.from_barrel;
    int last_driven = -1;
    for (std::size_t i = 0; i + 1 < way.size(); ++i) {
        // The strongest mesh between the two mobiles.
        int best = -1;
        for (std::size_t k = 0; k < m.meshes.size(); ++k) {
            const model::MeshJoint& j = m.meshes[k];
            if (!m.in_train(j)) continue;
            const int a = m.mobile_of(j.a), b = m.mobile_of(j.b);
            if (!((a == way[i] && b == way[i + 1]) || (a == way[i + 1] && b == way[i]))) continue;
            if (best < 0 || j.confidence > m.meshes[static_cast<std::size_t>(best)].confidence) best = static_cast<int>(k);
        }
        if (best < 0) break;
        const model::MeshJoint& j = m.meshes[static_cast<std::size_t>(best)];
        WalkStep step;
        step.mesh = best;
        step.driver = m.mobile_of(j.a) == way[i] ? j.a : j.b;
        step.driven = step.driver == j.a ? j.b : j.a;
        const double ratio = static_cast<double>(teeth(step.driver)) / std::max(1, teeth(step.driven));
        step.text = name(step.driver) + " (" + std::to_string(teeth(step.driver)) + " teeth) drives " + name(step.driven) + " (" +
                    std::to_string(teeth(step.driven)) + " teeth), so the second turns " +
                    (ratio >= 1.0 ? number(ratio, "%.3g") + " times for each turn of the first"
                                  : "once for each " + number(1.0 / ratio, "%.3g") + " turns of the first") +
                    ". Module " + number(j.module, "%.3f") + ", " + (j.manual ? "declared by you" : "confidence " + number(j.confidence, "%.2f")) + ".";
        last_driven = step.driven;
        walk.steps.push_back(std::move(step));
    }
    if (chain.connected) {
        walk.complete = true;
        return walk;
    }

    WalkStep step;
    if (chain.escape < 0) {
        step.kind = WalkStep::Kind::Missing;
        step.needs_role = Role::EscapeWheel;
        step.text = "No escape wheel found, so the train has nothing to end on. Pick the escape wheel.";
        walk.steps.push_back(std::move(step));
        return walk;
    }
    step.kind = WalkStep::Kind::Gap;
    step.driver = way.empty() ? -1 : toothed_in(way.back(), last_driven);
    if (step.driver < 0 && !way.empty()) step.driver = toothed_in(way.back(), -1);
    step.text = step.driver >= 0 ? "Nothing meshes with " + name(step.driver) + " yet. Pick the part it drives."
                                 : "The chain stops here. Pick the part that comes next.";
    for (std::size_t k = 0; k < suggestions.size(); ++k) {
        const int ma = m.mobile_of(suggestions[k].a), mb = m.mobile_of(suggestions[k].b);
        if (ma < 0 || mb < 0 || chain.fed.size() <= static_cast<std::size_t>(std::max(ma, mb))) continue;
        if (chain.fed[static_cast<std::size_t>(ma)] != chain.fed[static_cast<std::size_t>(mb)]) step.suggestions.push_back(static_cast<int>(k));
    }
    walk.steps.push_back(std::move(step));
    return walk;
}

std::vector<Issue> check_rig(const Movement& m, double low_confidence) {
    std::vector<Issue> issues;
    auto add = [&issues](Severity s, std::string msg, std::vector<std::string> parts = {}) {
        issues.push_back({s, std::move(msg), std::move(parts)});
    };

    if (m.components.empty()) add(Severity::Error, "the movement has no components");

    bool has_barrel = false;
    bool has_escape = false;
    for (const Component& c : m.components) {
        const std::string& name = c.features.name;
        has_barrel |= c.role == Role::Barrel;
        has_escape |= c.role == Role::EscapeWheel;
        if (c.role == Role::Unknown) {
            add(Severity::Warning, "'" + name + "' could not be classified", {name});
        } else if (c.role_confidence < low_confidence) {
            add(Severity::Warning, "'" + name + "' classified as " + std::string(to_string(c.role)) +
                                       " with low confidence " + std::to_string(c.role_confidence), {name});
        }
    }
    if (!has_barrel) add(Severity::Warning, "no barrel found; the train has no driving root");
    if (!has_escape) add(Severity::Warning, "no escape wheel found");

    std::vector<bool> meshed(m.components.size(), false);
    for (const model::MeshJoint& j : m.meshes) {
        meshed[j.a] = meshed[j.b] = true;
        if (j.confidence < low_confidence) {
            add(Severity::Warning, "mesh '" + m.components[j.a].display_name() + "' / '" +
                                       m.components[j.b].display_name() + "' has low confidence " +
                                       std::to_string(j.confidence),
                {m.components[j.a].features.name, m.components[j.b].features.name});
        }
    }
    if (m.meshes.empty()) add(Severity::Error, "no gear meshes were inferred");
    for (const Component& c : m.components) {
        if (c.role == Role::DateRing && c.features.inner_teeth <= 0 && c.features.teeth <= 0) {
            add(Severity::Warning, "date ring '" + c.display_name() + "' shows no teeth, so its step is assumed to be 1/31 of a turn",
                {c.features.name});
        }
    }
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const Component& c = m.components[i];
        const int index = static_cast<int>(i);
        if (c.role == Role::Lever && !model::lever_contact(m.components, index)) {
            add(Severity::Info, "lever '" + c.display_name() + "' rests on no toothed part, so it is drawn at rest", {c.features.name});
        } else if (c.role == Role::Spring && !model::spring_flex(m.components, index)) {
            add(Severity::Info, "spring '" + c.display_name() + "' presses on no lever or toothed part, so it is drawn at rest",
                {c.features.name});
        } else if (c.role == Role::Stem && c.features.length <= 0.0) {
            add(Severity::Info, "stem '" + c.display_name() + "' has no recorded axis (re-import the file), so it is drawn at rest",
                {c.features.name});
        }
    }
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const Component& c = m.components[i];
        if (!model::is_toothed_role(c.role) || meshed[i]) continue;
        if (c.role == Role::EscapeWheel) {  // its partner is the pallet fork, not inferred yet
            add(Severity::Info, "escape wheel '" + c.display_name() + "' has no pallet fork joint (not inferred in M0)",
                {c.features.name});
        } else {
            add(Severity::Warning, "toothed part '" + c.display_name() + "' meshes with nothing", {c.features.name});
        }
    }
    if (has_barrel && has_escape) {
        const TrainChain chain = train_chain(m);
        if (!chain.connected) {
            std::vector<std::string> parts;
            for (int mobile : {chain.from_barrel.empty() ? -1 : chain.from_barrel.back(),
                               chain.from_escape.empty() ? -1 : chain.from_escape.front()}) {
                if (mobile < 0) continue;
                for (int idx : m.mobiles[static_cast<std::size_t>(mobile)].components) {
                    if (model::is_toothed_role(m.components[static_cast<std::size_t>(idx)].role)) {
                        parts.push_back(m.components[static_cast<std::size_t>(idx)].features.name);
                        break;
                    }
                }
            }
            add(Severity::Warning, "the train is broken: " + describe_chain(m, chain), std::move(parts));
        }
    }
    return issues;
}

}  // namespace ohs::rigging
