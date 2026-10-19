// SPDX-License-Identifier: Apache-2.0
#include "model/rig_io.hpp"

#include <toml++/toml.hpp>

#include <fstream>
#include <sstream>

#include "core/error.hpp"

namespace ohs::model {

namespace {

constexpr int kSchemaVersion = 1;

toml::table component_to_table(const Component& c) {
    const PartFeatures& f = c.features;
    toml::table t;
    t.insert("name", f.name);
    t.insert("role", std::string(to_string(c.role)));
    t.insert("role_confidence", c.role_confidence);
    t.insert("role_source", c.role_source);
    t.insert("teeth", f.teeth);
    t.insert("tooth_confidence", f.tooth_confidence);
    if (f.inner_teeth > 0) {
        t.insert("inner_teeth", f.inner_teeth);
        t.insert("inner_tip_radius", f.inner_tip_radius);
        t.insert("inner_root_radius", f.inner_root_radius);
    }
    t.insert("center", toml::array{f.center.x, f.center.y});
    t.insert("z", toml::array{f.z_min, f.z_max});
    t.insert("extent", toml::array{f.extent_x, f.extent_y});
    t.insert("outer_radius", f.outer_radius);
    t.insert("root_radius", f.root_radius);
    t.insert("bore_radius", f.bore_radius);
    t.insert("flank_base_radius", f.flank_base_radius);
    t.insert("flank_rms", f.flank_rms);
    if (f.flank_cycloid_rms >= 0) {
        t.insert("flank_cycloid_rms", f.flank_cycloid_rms);
        t.insert("flank_cycloid_pitch_radius", f.flank_cycloid_pitch_radius);
        t.insert("flank_cycloid_rolling_radius", f.flank_cycloid_rolling_radius);
    }
    t.insert("volume", f.volume);
    if (f.length > 0) {
        t.insert("axis", toml::array{f.axis[0], f.axis[1], f.axis[2]});
        t.insert("size", toml::array{f.length, f.width, f.height});
    }
    if (!f.holes.empty()) {
        toml::array holes;
        for (const Hole& h : f.holes) holes.push_back(toml::array{h.x, h.y, h.radius});
        t.insert("holes", std::move(holes));
    }
    // TOML integers are signed 64-bit, so store the fingerprint as hex text.
    std::ostringstream fp;
    fp << std::hex << f.fingerprint;
    t.insert("fingerprint", fp.str());
    if (c.gear) {
        const GearFit& g = *c.gear;
        toml::table gt;
        gt.insert("module", g.module);
        gt.insert("module_source", g.module_source);
        gt.insert("addendum", g.addendum);
        gt.insert("dedendum", g.dedendum);
        gt.insert("base_radius", g.base_radius);
        gt.insert("pressure_angle_deg", g.pressure_angle_deg);
        gt.insert("profile", g.profile);
        if (g.rolling_radius > 0) gt.insert("rolling_radius", g.rolling_radius);
        t.insert("gear", std::move(gt));
    }
    return t;
}

double number(const toml::table& t, std::string_view key, double fallback = 0.0) {
    return t[key].value_or(fallback);
}

Vec2 pair(const toml::table& t, std::string_view key) {
    const toml::array* a = t[key].as_array();
    if (!a || a->size() != 2) throw Error("rig: '" + std::string(key) + "' must be a 2-element array");
    return {a->get(0)->value_or(0.0), a->get(1)->value_or(0.0)};
}

Component component_from_table(const toml::table& t) {
    Component c;
    PartFeatures& f = c.features;
    f.name = t["name"].value_or(std::string{});
    if (f.name.empty()) throw Error("rig: component without a name");
    const std::string role = t["role"].value_or(std::string{"unknown"});
    const auto parsed = role_from_string(role);
    if (!parsed) throw Error("rig: unknown role '" + role + "' on component '" + f.name + "'");
    c.role = *parsed;
    c.role_confidence = number(t, "role_confidence");
    c.role_source = t["role_source"].value_or(std::string{"user"});
    f.teeth = t["teeth"].value_or(0);
    f.tooth_confidence = number(t, "tooth_confidence");
    f.inner_teeth = t["inner_teeth"].value_or(0);
    f.inner_tip_radius = number(t, "inner_tip_radius");
    f.inner_root_radius = number(t, "inner_root_radius");
    f.center = pair(t, "center");
    const Vec2 z = pair(t, "z");
    f.z_min = z.x;
    f.z_max = z.y;
    const Vec2 ext = pair(t, "extent");
    f.extent_x = ext.x;
    f.extent_y = ext.y;
    f.outer_radius = number(t, "outer_radius");
    f.root_radius = number(t, "root_radius");
    f.bore_radius = number(t, "bore_radius");
    f.flank_base_radius = number(t, "flank_base_radius");
    f.flank_rms = number(t, "flank_rms", -1.0);
    f.flank_cycloid_rms = number(t, "flank_cycloid_rms", -1.0);
    f.flank_cycloid_pitch_radius = number(t, "flank_cycloid_pitch_radius");
    f.flank_cycloid_rolling_radius = number(t, "flank_cycloid_rolling_radius");
    f.volume = number(t, "volume");
    if (const toml::array* a = t["axis"].as_array(); a && a->size() == 3) {
        for (std::size_t i = 0; i < 3; ++i) f.axis[i] = a->get(i)->value_or(0.0);
    }
    if (const toml::array* a = t["size"].as_array(); a && a->size() == 3) {
        f.length = a->get(0)->value_or(0.0);
        f.width = a->get(1)->value_or(0.0);
        f.height = a->get(2)->value_or(0.0);
    }
    if (const toml::array* hs = t["holes"].as_array()) {
        for (const toml::node& n : *hs) {
            const toml::array* h = n.as_array();
            if (h && h->size() == 3) f.holes.push_back({h->get(0)->value_or(0.0), h->get(1)->value_or(0.0), h->get(2)->value_or(0.0)});
        }
    }
    const std::string fp = t["fingerprint"].value_or(std::string{"0"});
    f.fingerprint = std::stoull(fp, nullptr, 16);
    if (const toml::table* gt = t["gear"].as_table()) {
        GearFit g;
        g.module = number(*gt, "module");
        g.module_source = (*gt)["module_source"].value_or(std::string{});
        g.addendum = number(*gt, "addendum");
        g.dedendum = number(*gt, "dedendum");
        g.base_radius = number(*gt, "base_radius");
        g.pressure_angle_deg = number(*gt, "pressure_angle_deg");
        g.profile = (*gt)["profile"].value_or(std::string{});
        g.rolling_radius = number(*gt, "rolling_radius");
        c.gear = g;
    }
    return c;
}

}  // namespace

std::string to_toml(const Movement& m) {
    toml::table root;
    root.insert("schema_version", kSchemaVersion);

    toml::table movement;
    movement.insert("name", m.name);
    movement.insert("source_geometry", m.source_geometry);
    movement.insert("beat_rate_vph", m.beat_rate_vph);
    movement.insert("dial_axis", m.dial_axis);
    root.insert("movement", std::move(movement));

    toml::array components;
    for (const Component& c : m.components) components.push_back(component_to_table(c));
    root.insert("component", std::move(components));

    toml::array mobiles;
    for (const Mobile& mob : m.mobiles) {
        toml::table t;
        t.insert("name", mob.name);
        t.insert("center", toml::array{mob.center.x, mob.center.y});
        toml::array names;
        for (int idx : mob.components) names.push_back(m.components.at(idx).features.name);
        t.insert("components", std::move(names));
        mobiles.push_back(std::move(t));
    }
    root.insert("mobile", std::move(mobiles));

    toml::array joints;
    for (const MeshJoint& j : m.meshes) {
        toml::table t;
        t.insert("type", "mesh");
        t.insert("a", m.components.at(j.a).features.name);
        t.insert("b", m.components.at(j.b).features.name);
        t.insert("centre_distance", j.centre_distance);
        t.insert("module", j.module);
        t.insert("confidence", j.confidence);
        if (j.manual) t.insert("manual", true);
        joints.push_back(std::move(t));
    }
    root.insert("joint", std::move(joints));

    std::ostringstream out;
    out << "# Open Horology Studio rig/project file. Safe to edit by hand.\n" << root << '\n';
    return out.str();
}

Movement from_toml(std::string_view text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        std::ostringstream msg;
        msg << "rig: " << e.description() << " at line " << e.source().begin.line;
        throw Error(msg.str());
    }

    const int version = root["schema_version"].value_or(0);
    if (version != kSchemaVersion) {
        throw Error("rig: unsupported schema_version " + std::to_string(version));
    }

    Movement m;
    if (const toml::table* mv = root["movement"].as_table()) {
        m.name = (*mv)["name"].value_or(std::string{});
        m.source_geometry = (*mv)["source_geometry"].value_or(std::string{});
        m.beat_rate_vph = (*mv)["beat_rate_vph"].value_or(0.0);
        m.dial_axis = (*mv)["dial_axis"].value_or(std::string{"z"});
    }

    auto index_of = [&m](const std::string& name) {
        for (std::size_t i = 0; i < m.components.size(); ++i) {
            if (m.components[i].features.name == name) return static_cast<int>(i);
        }
        throw Error("rig: reference to unknown component '" + name + "'");
    };

    if (const toml::array* comps = root["component"].as_array()) {
        for (const toml::node& n : *comps) {
            if (const toml::table* t = n.as_table()) m.components.push_back(component_from_table(*t));
        }
    }
    if (const toml::array* mobs = root["mobile"].as_array()) {
        for (const toml::node& n : *mobs) {
            const toml::table* t = n.as_table();
            if (!t) continue;
            Mobile mob;
            mob.name = (*t)["name"].value_or(std::string{});
            mob.center = pair(*t, "center");
            if (const toml::array* names = (*t)["components"].as_array()) {
                for (const toml::node& nn : *names) {
                    mob.components.push_back(index_of(nn.value_or(std::string{})));
                }
            }
            m.mobiles.push_back(std::move(mob));
        }
    }
    if (const toml::array* joints = root["joint"].as_array()) {
        for (const toml::node& n : *joints) {
            const toml::table* t = n.as_table();
            if (!t || (*t)["type"].value_or(std::string{}) != "mesh") continue;
            MeshJoint j;
            j.a = index_of((*t)["a"].value_or(std::string{}));
            j.b = index_of((*t)["b"].value_or(std::string{}));
            j.centre_distance = number(*t, "centre_distance");
            j.module = number(*t, "module");
            j.confidence = number(*t, "confidence");
            j.manual = (*t)["manual"].value_or(false);
            m.meshes.push_back(j);
        }
    }
    return m;
}

void save_rig(const Movement& movement, const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw Error("cannot write " + path.string());
    out << to_toml(movement);
    if (!out) throw Error("write failed: " + path.string());
}

Movement load_rig(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw Error("cannot read " + path.string());
    std::stringstream buf;
    buf << in.rdbuf();
    return from_toml(buf.str());
}

}  // namespace ohs::model
