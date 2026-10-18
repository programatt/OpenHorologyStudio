// SPDX-License-Identifier: Apache-2.0
#include "app/import.hpp"

#include <toml++/toml.hpp>

#include <chrono>
#include <regex>

#include <spdlog/spdlog.h>

#include "core/error.hpp"
#include "geometry/step_io.hpp"

namespace ohs::app {

model::Movement import_movement(const std::filesystem::path& step_file, const ImportOptions& options) {
    const auto parts = geometry::import_step(step_file);
    spdlog::info("read {} parts from {}", parts.size(), step_file.string());

    geometry::FeatureOptions feature_options = options.features;
    if (options.auto_axis) {
        feature_options.dial_axis = geometry::detect_dial_axis(parts);
        spdlog::info("dial axis: {} (found from the geometry)", geometry::axis_name(feature_options.dial_axis));
    }

    std::vector<model::PartFeatures> features;
    features.reserve(parts.size());
    for (const auto& part : parts) {
        const auto start = std::chrono::steady_clock::now();
        features.push_back(geometry::extract_features(part, feature_options));
        spdlog::debug("{}: volume {:.3f} mm3, {} teeth, {:.2f} s", part.name, features.back().volume,
                      features.back().teeth, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    }

    rigging::RigOptions rig = options.rig;
    rig.min_teeth = options.features.min_teeth;
    const std::string name = options.movement_name.empty() ? step_file.stem().string() : options.movement_name;
    model::Movement movement = rigging::auto_rig(name, std::move(features), rig);
    for (const auto& [a, b] : options.mesh_presets) {
        const std::regex ra(a, std::regex::icase), rb(b, std::regex::icase);
        std::string name_a, name_b;
        for (const auto& c : movement.components) {
            if (name_a.empty() && std::regex_match(c.features.name, ra)) name_a = c.features.name;
            if (name_b.empty() && std::regex_match(c.features.name, rb)) name_b = c.features.name;
        }
        if (name_a.empty() || name_b.empty()) {
            spdlog::warn("preset mesh {} / {}: no part matches '{}'", a, b, name_a.empty() ? a : b);
            continue;
        }
        rigging::add_manual_mesh(movement, name_a, name_b, rig);
    }
    movement.source_geometry = step_file.filename().string();
    movement.dial_axis = geometry::axis_name(feature_options.dial_axis);
    return movement;
}

std::vector<rigging::RolePreset> load_role_presets(const std::filesystem::path& path) {
    toml::table root;
    try {
        root = toml::parse_file(path.string());
    } catch (const toml::parse_error& e) {
        throw Error("preset " + path.string() + ": " + std::string(e.description()));
    }
    std::vector<rigging::RolePreset> presets;
    if (const toml::array* rules = root["rule"].as_array()) {
        for (const toml::node& n : *rules) {
            const toml::table* t = n.as_table();
            if (!t) continue;
            const std::string match = (*t)["match"].value_or(std::string{});
            const std::string role = (*t)["role"].value_or(std::string{});
            const auto parsed = model::role_from_string(role);
            if (match.empty() || !parsed) {
                throw Error("preset " + path.string() + ": rule needs 'match' and a valid 'role' (got '" + role + "')");
            }
            presets.push_back({match, *parsed});
        }
    }
    return presets;
}

std::vector<std::pair<std::string, std::string>> load_mesh_presets(const std::filesystem::path& path) {
    toml::table root;
    try {
        root = toml::parse_file(path.string());
    } catch (const toml::parse_error& e) {
        throw Error("preset " + path.string() + ": " + std::string(e.description()));
    }
    std::vector<std::pair<std::string, std::string>> meshes;
    if (const toml::array* rules = root["mesh"].as_array()) {
        for (const toml::node& n : *rules) {
            const toml::table* t = n.as_table();
            if (!t) continue;
            const std::string a = (*t)["a"].value_or(std::string{});
            const std::string b = (*t)["b"].value_or(std::string{});
            if (a.empty() || b.empty()) throw Error("preset " + path.string() + ": a [[mesh]] rule needs 'a' and 'b'");
            meshes.emplace_back(a, b);
        }
    }
    return meshes;
}

}  // namespace ohs::app
