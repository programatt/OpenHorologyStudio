// SPDX-License-Identifier: Apache-2.0
#include "app/import.hpp"

#include <toml++/toml.hpp>

#include <chrono>

#include <spdlog/spdlog.h>

#include "core/error.hpp"
#include "geometry/step_io.hpp"

namespace ohs::app {

model::Movement import_movement(const std::filesystem::path& step_file, const ImportOptions& options) {
    const auto parts = geometry::import_step(step_file);
    spdlog::info("read {} parts from {}", parts.size(), step_file.string());

    std::vector<model::PartFeatures> features;
    features.reserve(parts.size());
    for (const auto& part : parts) {
        const auto start = std::chrono::steady_clock::now();
        features.push_back(geometry::extract_features(part, options.features));
        spdlog::debug("{}: volume {:.3f} mm3, {} teeth, {:.2f} s", part.name, features.back().volume,
                      features.back().teeth, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    }

    rigging::RigOptions rig = options.rig;
    rig.min_teeth = options.features.min_teeth;
    const std::string name = options.movement_name.empty() ? step_file.stem().string() : options.movement_name;
    model::Movement movement = rigging::auto_rig(name, std::move(features), rig);
    movement.source_geometry = step_file.filename().string();
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

}  // namespace ohs::app
