// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "model/model.hpp"

namespace ohs::rigging {

/// Maps part names matching a case-insensitive regex to a role (e.g. an OM10 preset).
struct RolePreset {
    std::string pattern;
    model::Role role{model::Role::Unknown};
};

struct RigOptions {
    double coaxial_tolerance = 0.05;  ///< mm, max centre offset for parts of one mobile
    double centre_tolerance = 0.03;   ///< relative error allowed on mesh centre distance
    double module_tolerance = 0.05;   ///< relative module mismatch allowed between meshing gears
    int min_teeth = 6;                ///< toothed parts need at least this many detected teeth
    double beat_rate_vph = 0.0;
    std::vector<RolePreset> presets;
};

struct Classification {
    model::Role role{model::Role::Unknown};
    double confidence{};
    std::string source;
};

Classification classify(const model::PartFeatures& part, const RigOptions& options);

/// Classifies parts, groups mobiles by coaxiality and infers gear meshes.
model::Movement auto_rig(std::string movement_name, std::vector<model::PartFeatures> parts,
                         const RigOptions& options);

enum class Severity { Info, Warning, Error };

struct Issue {
    Severity severity;
    std::string message;
};

/// Sanity-checks a rig and lists everything a user should confirm or correct.
std::vector<Issue> check_rig(const model::Movement& movement, double low_confidence = 0.6);

}  // namespace ohs::rigging
