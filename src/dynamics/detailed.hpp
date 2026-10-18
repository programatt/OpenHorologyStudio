// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "dynamics/oscillator.hpp"

namespace ohs::dynamics {

/// Level-3 ("detailed") simulation. It integrates the balance and the escape wheel as two
/// coupled bodies with a fixed small step and no event location: the wheel presses on a pallet
/// through a stiff, damped, one-sided contact whose position follows the balance (the lever and
/// roller are a kinematic constraint), so the impulse, the inelastic pick-up of the wheel, the
/// drop and the lock arise from the contact instead of being imposed. It shares only the
/// parameters with the level-2 model, which makes it an independent check on that model's
/// reductions. `simulate_chrono` (chrono_adapter.hpp) runs the same model on Project Chrono.
enum class Engine { Builtin, Chrono };
const char* to_string(Engine e);
Engine engine_from_string(const std::string& text);  // throws ohs::Error

struct DetailedConditions {
    Position position{Position::DialUp};
    double wheel_torque_nm{0.0};
    double initial_amplitude_deg{0};  ///< 0: start from estimate_amplitude_deg()
    double duration_s{60.0};
    double step_s{0.0};               ///< 0: 0.1 / (2 pi contact_frequency)
};

RunSummary simulate_detailed(const Calibre& calibre, const DetailedConditions& conditions, const EventSink& sink);

struct Agreement {
    Measurement level2;
    Measurement level3;
    double amplitude_diff_percent{};
    double rate_diff_s_per_day{};
    double beat_error_diff_ms{};
};

/// Runs both models from the level-2 steady state and compares a measurement window of each.
/// `warmup_s` and `window_s` apply to the detailed run.
/// `engine` picks what runs the detailed model: OHS's own integrator or Project Chrono.
Agreement compare_levels(const Calibre& calibre, double wheel_torque_nm, Position position = Position::DialUp,
                         double warmup_s = 30.0, double window_s = 20.0, Engine engine = Engine::Builtin);

}  // namespace ohs::dynamics
