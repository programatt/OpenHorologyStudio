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
    // Mesh plausibility, in modules, with the module taken from the centre distance
    // (m = 2C / (z1 + z2)): both gears must have a sensible addendum and dedendum, and the tip of
    // each must clear the root of the other by roughly a quarter module.
    double min_addendum = 0.75;
    double max_addendum = 1.5;
    double min_dedendum = 0.9;
    double max_dedendum = 1.8;
    double min_clearance = -0.1;
    double max_clearance = 0.6;
    double ideal_clearance = 0.25;
    /// A flank fits an involute when its rms deviation is below this many modules.
    double involute_rms_modules = 0.02;
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

/// Recomputes mobiles, meshes and gear fits from the components' roles and features.
void infer_structure(model::Movement& movement, const RigOptions& options);

/// Overrides a component's role (source "user", confidence 1) and re-infers the structure.
/// Throws ohs::Error if there is no component called `name`.
void set_role(model::Movement& movement, const std::string& name, model::Role role, const RigOptions& options);

enum class Severity { Info, Warning, Error };

struct Issue {
    Severity severity;
    std::string message;
};

/// Sanity-checks a rig and lists everything a user should confirm or correct.
std::vector<Issue> check_rig(const model::Movement& movement, double low_confidence = 0.6);

}  // namespace ohs::rigging
