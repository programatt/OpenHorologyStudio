// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "dynamics/torque_chain.hpp"

namespace ohs::dynamics {

/// One line of timegrapher data from a physical movement.
struct BenchReading {
    double hours{};            ///< since the mainspring was fully wound
    Position position{Position::DialUp};
    double amplitude_deg{-1};  ///< negative when not recorded
    double rate_s_per_day{};
    bool has_rate{false};
    double beat_error_ms{-1};
};

/// CSV with the header `hours,position,amplitude_deg,rate_s_per_day,beat_error_ms`. Empty cells
/// mean "not measured". Throws ohs::Error with the line number on bad input.
std::vector<BenchReading> parse_readings_csv(const std::string& text);
std::vector<BenchReading> load_readings_csv(const std::string& path);

struct BenchComparison {
    BenchReading measured;
    double simulated_amplitude_deg{};
    double simulated_rate_s_per_day{};
};

struct Fit {
    std::vector<BenchComparison> rows;
    double rms_amplitude_deg{};
    double rms_rate_s_per_day{};     ///< over rows with a measured rate, after removing the mean offset
    double amplitude_slope_measured{};   ///< degrees of amplitude per hour of running, dial up readings
    double amplitude_slope_simulated{};
};

/// Simulates every reading with the calibre and compares.
Fit compare_with_bench(const Calibre& calibre, const TorqueChain& chain, const std::vector<BenchReading>& readings);

struct Calibration {
    Calibre calibre;
    std::vector<std::string> fitted;  ///< parameters that were adjusted
    Fit before;
    Fit after;
};

/// Adjusts the pivot friction (and, with `fit_damping`, the air damping too) so the simulated
/// amplitudes match the measured ones in the least-squares sense. Rates are reported, not fitted.
Calibration calibrate(const Calibre& calibre, const TorqueChain& chain, const std::vector<BenchReading>& readings,
                      bool fit_damping = false);

}  // namespace ohs::dynamics
