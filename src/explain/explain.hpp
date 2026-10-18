// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "kinematics/kinematics.hpp"
#include "model/model.hpp"

namespace ohs::explain {

struct PartExplanation {
    std::string name;
    std::string role;
    std::string summary;             ///< what this kind of part does
    std::vector<std::string> facts;  ///< numbers from this movement
};

/// Explains one part with figures from the rig and train. Throws ohs::Error for an unknown name.
PartExplanation explain_part(const model::Movement& movement, const kinematics::TrainAnalysis& train,
                             const std::string& name);

struct TrainStep {
    std::string driver;
    std::string driven;
    int driver_teeth{};
    int driven_teeth{};
    std::string driven_mobile;
    double step_ratio{};        ///< driven turns per driver turn (positive; direction reverses every mesh)
    double cumulative_ratio{};  ///< signed, relative to the root mobile
    double revs_per_hour{};     ///< of the driven mobile, 0 when no beat rate is known
};

struct Walkthrough {
    std::string root;
    std::vector<TrainStep> steps;
    std::vector<std::string> conclusion;  ///< e.g. the escape wheel's beat arithmetic
};

/// Follows the gear meshes outward from the root mobile, one step per mesh, breadth first.
Walkthrough walk_train(const model::Movement& movement, const kinematics::TrainAnalysis& train);

std::string to_text(const PartExplanation& explanation);
std::string to_text(const Walkthrough& walkthrough);

}  // namespace ohs::explain
