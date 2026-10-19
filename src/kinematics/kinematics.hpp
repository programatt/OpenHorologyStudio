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
    /// A date ring is not geared to the train. It moves one tooth each time the mobile `index_driver` (the
    /// 24-hour wheel) completes a turn, or each 24 h of clock time when no such mobile is in the train
    /// (`index_driver` -1). `index_teeth` is 0 for every other mobile. For a ring, `ratio` and
    /// `revs_per_hour` are the average over whole days.
    int index_teeth{};
    int index_driver{-1};
};

/// A part that turns about an axis of its own and is not geared to the train: the winding stem, which spins,
/// and the levers and springs that ride on teeth, which swing or bend as the toothed part turns.
struct Articulation {
    enum class Kind {
        Spin,    ///< turns with the root mobile, `gain` turns for each of its own
        Follow,  ///< swings as the toothed part of mobile `driver` turns, once for each of its `teeth`
    };
    int component{-1};
    Kind kind{Kind::Spin};
    double point[3]{};  ///< a point on the axis, mm
    double axis[3]{0, 0, 1};
    double gain{};
    int driver{-1};
    int teeth{};
    double amplitude{};  ///< radians, at the most (signed)
};

struct TrainAnalysis {
    int root{-1};
    int escape_mobile{-1};
    int escape_teeth{};
    double beats_per_root_rev{};  ///< two beats per escape wheel tooth
    double root_period_hours{};   ///< hours per root revolution implied by the beat rate
    std::vector<MobileMotion> mobiles;
    std::vector<std::string> warnings;
    std::vector<Articulation> articulations;  ///< Stem, Lever and Spring parts that move; see `articulation_angles`

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
    /// A date ring is always stepped, in either mode. It moves during the last `date_engage_fraction` of each
    /// turn of its driver (the finger is in contact with a tooth, then the jumper snaps it home), and rests
    /// the remainder. 0.15 of a day is 3.6 h.
    double date_engage_fraction{0.15};
};

/// Turns of the winding stem for each turn of the barrel (the root mobile), as the winding train turns them
/// when the barrel is wound: crown, winding pinion, crown wheel and ratchet wheel, at a typical ratio.
inline constexpr double kWindingRatio = 6.0;

/// Rotation angle in radians of every mobile at `seconds` of real time (mobile order as in the
/// movement; unreached mobiles stay at 0). Needs a known beat rate, i.e. root_period_hours > 0.
/// A positive angle is counter-clockwise seen from +Z, and the root mobile turns positively. A date
/// ring steps one tooth a day (see PoseOptions) rather than following the train.
std::vector<double> mobile_angles(const TrainAnalysis& train, double seconds, const PoseOptions& options = {});

/// Turn angle in radians of every articulation of the train, in order, given the mobile angles at the same
/// instant (`mobile_angles`). The stem is turned as if the barrel were being wound, opposite to the way the
/// train runs. A lever or spring is at rest when the tooth it rides on is at its lowest, and goes over it
/// once for each tooth that passes: slowly up the face, then back down quickly.
std::vector<double> articulation_angles(const TrainAnalysis& train, const std::vector<double>& mobile_angles);

}  // namespace ohs::kinematics
