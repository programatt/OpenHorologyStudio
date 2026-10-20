// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "geometry/features.hpp"
#include "model/model.hpp"
#include "rigging/rigging.hpp"

namespace ohs::app {

struct ImportOptions {
    geometry::FeatureOptions features;
    rigging::RigOptions rig;
    std::string movement_name;  ///< defaults to the file stem
    /// Meshes to declare after rigging: pairs of case-insensitive regexes naming the two parts.
    std::vector<std::pair<std::string, std::string>> mesh_presets;
    bool auto_axis{false};      ///< find the dial axis from the geometry instead of features.dial_axis
    /// When non-empty, the geometry cache of this project file is written from the parts just read, so that
    /// opening the project and the geometry commands need not read the CAD file again.
    std::filesystem::path cache_project;
    /// Called after each part's features are extracted (done, total), from any worker thread.
    std::function<void(std::size_t done, std::size_t total)> progress;
    /// When set and true, the import stops between parts and throws ohs::Error("import cancelled").
    const std::atomic<bool>* cancel{nullptr};
};

/// Imports a STEP assembly and auto-rigs it. Throws ohs::Error on unreadable input.
model::Movement import_movement(const std::filesystem::path& step_file, const ImportOptions& options);

/// Reads `[[rule]]` entries (match = regex, role = name) from a preset TOML file.
std::vector<rigging::RolePreset> load_role_presets(const std::filesystem::path& path);

/// Reads `[[mesh]]` entries (a = regex, b = regex) from a preset TOML file.
std::vector<std::pair<std::string, std::string>> load_mesh_presets(const std::filesystem::path& path);

}  // namespace ohs::app
