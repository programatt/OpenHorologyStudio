// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "model/model.hpp"

namespace ohs::model {

/// Serialises the movement (features, roles, mobiles, joints) as the TOML rig/project file.
std::string to_toml(const Movement& movement);

/// Parses a rig/project file. Throws ohs::Error on malformed input.
Movement from_toml(std::string_view text);

void save_rig(const Movement& movement, const std::filesystem::path& path);
Movement load_rig(const std::filesystem::path& path);

}  // namespace ohs::model
