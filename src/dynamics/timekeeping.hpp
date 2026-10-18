// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "dynamics/oscillator.hpp"
#include "dynamics/torque_chain.hpp"

namespace ohs::dynamics {

/// Steady-state behaviour at one state of wind and orientation.
struct WindPoint {
    double wind{};              ///< 1 = fully wound, 0 = run down
    double hours{};             ///< hours since full wind
    Position position{Position::DialUp};
    double barrel_torque_nmm{};
    double wheel_torque_nm{};   ///< at the escape wheel
    double amplitude_deg{};
    double rate_s_per_day{};
    double beat_error_ms{};
    bool stalled{false};
};

/// Total running time of a full wind if the amplitude never limited it.
double full_wind_hours(const Calibre& calibre, const TorqueChain& chain);

/// Steady state at a given wind. Quasi-static: the amplitude settles within a minute while the
/// spring unwinds over hours, so the run-down is a sequence of settled states.
WindPoint wind_point(const Calibre& calibre, const TorqueChain& chain, double wind,
                     Position position = Position::DialUp);

/// Amplitude and rate from fully wound to run down in `samples` equal steps of wind.
std::vector<WindPoint> amplitude_vs_wind(const Calibre& calibre, const TorqueChain& chain, int samples,
                                         Position position = Position::DialUp);

/// Hours until the amplitude falls below min_running_amplitude_deg (or the balance stalls).
double power_reserve_hours(const Calibre& calibre, const TorqueChain& chain, Position position = Position::DialUp);

/// Rate against amplitude: least-squares slope in s/day per degree of the points' rates.
double isochronism_slope(const std::vector<WindPoint>& points);

struct PositionalResult {
    std::vector<WindPoint> points;  ///< one per position
    double spread_s_per_day{};      ///< max rate - min rate
    double flat_to_vertical_amplitude_deg{};  ///< mean flat amplitude minus mean vertical amplitude
};

PositionalResult positional_error(const Calibre& calibre, const TorqueChain& chain, double wind);

/// Where the energy goes in one beat at a state of wind, from the model's own balance.
struct EnergyBudget {
    double wheel_work_j{};       ///< torque at the wheel times the wheel step
    double delivered_j{};        ///< to the balance
    double lost_unlock_j{};
    double lost_wheel_hit_j{};   ///< kinetic energy of the wheel, taken at pick-up and lost at the drop
    double lost_friction_j{};    ///< pivots
    double lost_air_j{};
    double efficiency{};         ///< (delivered - unlock - hit) / wheel_work
};

EnergyBudget energy_budget(const Calibre& calibre, double wheel_torque_nm, double amplitude_deg, Position position);

/// Adjusts the hairspring length (the regulator) so the dial-up rate at `wind` is the target.
Calibre regulate(const Calibre& calibre, const TorqueChain& chain, double target_rate_s_per_day = 0.0,
                 double wind = 0.5);

struct Timekeeping {
    WindPoint full;      ///< dial up, fully wound
    WindPoint half;
    WindPoint low;       ///< dial up, 10 % wound
    double isochronism_s_per_day_per_deg{};
    double power_reserve_h{};
    double full_wind_h{};
    PositionalResult positions;   ///< at half wind
    EnergyBudget budget;          ///< at full wind, dial up
};

Timekeeping evaluate(const Calibre& calibre, const TorqueChain& chain);

}  // namespace ohs::dynamics
