// SPDX-License-Identifier: Apache-2.0
#include "fea/material.hpp"

#include <toml++/toml.hpp>

#include <fstream>
#include <sstream>

#include "core/error.hpp"

namespace ohs::fea {

extern const char* const kMaterialsToml;  // generated from data/materials/materials.toml

std::vector<Material> materials_from_toml(const std::string& text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        throw Error(std::string("materials file: ") + std::string(e.description()));
    }
    std::vector<Material> out;
    const toml::array* arr = root["material"].as_array();
    if (!arr) throw Error("materials file: no [[material]] tables");
    for (const auto& el : *arr) {
        const toml::table* t = el.as_table();
        if (!t) throw Error("materials file: [[material]] must be a table");
        Material m;
        m.name = (*t)["name"].value_or(std::string());
        if (m.name.empty()) throw Error("materials file: a material has no name");
        m.description = (*t)["description"].value_or(std::string());
        m.youngs_gpa = (*t)["youngs_gpa"].value_or(0.0);
        m.poisson = (*t)["poisson"].value_or(0.3);
        m.density = (*t)["density"].value_or(0.0);
        m.cte = (*t)["cte"].value_or(0.0);
        m.strength_mpa = (*t)["strength_mpa"].value_or(0.0);
        m.thermoelastic = (*t)["thermoelastic"].value_or(0.0);
        m.source = (*t)["source"].value_or(std::string());
        if (const toml::array* r = (*t)["temp_range_c"].as_array(); r && r->size() == 2) {
            m.temp_range_c = {r->at(0).value_or(-20.0), r->at(1).value_or(80.0)};
        }
        if (m.youngs_gpa <= 0) throw Error("materials file: " + m.name + " needs youngs_gpa > 0");
        if (m.poisson <= -1 || m.poisson >= 0.5) throw Error("materials file: " + m.name + " needs -1 < poisson < 0.5");
        if (m.density <= 0) throw Error("materials file: " + m.name + " needs density > 0");
        out.push_back(std::move(m));
    }
    return out;
}

std::vector<Material> load_materials(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << f.rdbuf();
    return materials_from_toml(buf.str());
}

const std::vector<Material>& builtin_materials() {
    static const std::vector<Material> v = materials_from_toml(kMaterialsToml);
    return v;
}

const Material& find_material(std::string_view name) {
    for (const Material& m : builtin_materials()) {
        if (m.name == name) return m;
    }
    throw Error("unknown material '" + std::string(name) + "' (ohs materials list shows the catalogue)");
}

const Material& find_material(std::string_view name, const std::vector<Material>& extra) {
    for (const Material& m : extra) {
        if (m.name == name) return m;
    }
    for (const Material& m : builtin_materials()) {
        if (m.name == name) return m;
    }
    throw Error("unknown material '" + std::string(name) + "' (ohs materials list shows the catalogue)");
}

}  // namespace ohs::fea
