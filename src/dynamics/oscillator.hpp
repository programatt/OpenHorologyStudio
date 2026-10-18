// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <vector>

#include "dynamics/calibre.hpp"

namespace ohs::dynamics {

/// What happens at one instant of the balance's swing. Angles are measured from the lever's
/// centre line (the dead point of the escapement), positive counter-clockwise.
enum class EventType {
    UnlockStart,   ///< the roller jewel enters the fork and starts to unlock the pallet
    ImpulseStart,  ///< the lever begins to push the balance (the "tick" of the timegrapher)
    Drop,          ///< the impulse ends and the escape wheel drops onto the next pallet
    Turn,          ///< the balance stops at the end of its swing and reverses
};

struct Event {
    double t{};
    EventType type{EventType::Turn};
    double theta{};      ///< rad
    double omega{};      ///< rad/s just before the event
    int direction{};     ///< +1 or -1: the direction of the swing the event belongs to
};

using EventSink = std::function<void(const Event&)>;

/// Escapement quantities shared by the models, in SI units.
struct EscapementGeometry {
    double lift{};          ///< balance angle of the impulse, rad
    double unlock{};        ///< balance angle of the unlocking, rad
    double drop{};          ///< escape wheel angle lost to drop, rad
    double wheel_step{};    ///< wheel angle advanced per beat: pi / teeth, rad
    double ratio{};         ///< wheel angle per balance angle during the impulse: (step - drop) / lift
    double impulse_torque{};   ///< torque on the balance during the impulse, N·m
    double unlock_torque{};    ///< resisting torque on the balance while unlocking, N·m
};

EscapementGeometry escapement_geometry(const Calibre& c, double wheel_torque_nm);

struct RunConditions {
    Position position{Position::DialUp};
    double wheel_torque_nm{0.0};     ///< torque at the escape wheel
    double initial_amplitude_deg{0}; ///< 0: start from estimate_amplitude()
    double duration_s{60.0};
    double max_step_fraction{1.0 / 150.0};  ///< largest step as a fraction of the free period
};

struct RunSummary {
    bool stalled{false};  ///< the swing fell below what the lever needs to unlock
    double end_time{};
    double end_amplitude_deg{};
};

/// Level-2 simulation: the balance oscillator integrated with RK4 between escapement events;
/// each event (zone entry, zone exit, turning point) is located by bisection on the step, so the
/// piecewise-constant escapement torques switch at the exact instant. The impulse start is an
/// inelastic hit that gives the escape wheel its speed. Events go to `sink`.
RunSummary simulate_oscillator(const Calibre& calibre, const RunConditions& conditions, const EventSink& sink);

/// Energy-balance amplitude in degrees (harmonic swing assumed): drive per beat against Coulomb,
/// viscous and unlocking losses. A good first guess for the simulation, and its cross-check.
double estimate_amplitude_deg(const Calibre& calibre, double wheel_torque_nm, Position position);

struct Measurement {
    int beats{};
    double amplitude_deg{};   ///< mean of the swing extremes
    double period_s{};        ///< mean full oscillation (two beats)
    double rate_s_per_day{};  ///< positive = fast
    double beat_error_ms{};
};

/// Statistics over the events with t in [t_from, t_to].
Measurement measure(const std::vector<Event>& events, const Calibre& calibre, double t_from, double t_to);

/// Collects every event of a run.
struct EventLog {
    std::vector<Event> events;
    EventSink sink() {
        return [this](const Event& e) { events.push_back(e); };
    }
};

struct SteadyState {
    Measurement m;
    bool stalled{false};
    double settle_time_s{};
};

/// Settles the oscillator at fixed drive torque and measures a 20 s window.
SteadyState steady_state(const Calibre& calibre, double wheel_torque_nm, Position position = Position::DialUp);

}  // namespace ohs::dynamics
