// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "model/model.hpp"

namespace ohs::kinematics {

struct MobileMotion {
    std::string name;
    bool reached{false};
    double ratio{};            ///< angular speed relative to the root mobile; sign gives direction
    double revs_per_hour{};    ///< 0 unless a beat rate is known
};

struct TrainAnalysis {
    int root{-1};
    int escape_mobile{-1};
    int escape_teeth{};
    double beats_per_root_rev{};  ///< two beats per escape wheel tooth
    double root_period_hours{};   ///< hours per root revolution implied by the beat rate
    std::vector<MobileMotion> mobiles;
    std::vector<std::string> warnings;

    const MobileMotion* find(const std::string& mobile_name) const;
};

/// Level-1 (rigid ratio) analysis of the going train. `root` is a mobile index, or -1 to use the
/// mobile that holds the barrel. A zero `beat_rate_vph` skips the absolute speeds.
TrainAnalysis analyze_train(const model::Movement& movement, double beat_rate_vph, int root = -1);

struct PoseOptions {
    /// false: the train turns smoothly. true: it advances one escapement step per beat, moving
    /// during the last `impulse_fraction` of each beat and resting the remainder.
    bool stepped{false};
    double impulse_fraction{0.2};
};

/// Rotation angle in radians of every mobile at `seconds` of real time (mobile order as in the
/// movement; unreached mobiles stay at 0). Needs a known beat rate, i.e. root_period_hours > 0.
/// A positive angle is counter-clockwise seen from +Z, and the root mobile turns positively.
std::vector<double> mobile_angles(const TrainAnalysis& train, double seconds, const PoseOptions& options = {});

}  // namespace ohs::kinematics
