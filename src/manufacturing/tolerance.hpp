// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "model/model.hpp"

namespace ohs::manufacturing {

/// Monte Carlo tolerance analysis of a rigged movement. Each dimension is an independent random
/// variable, normal with +-tol at three standard deviations (or uniform over +-tol); one build is
/// one draw of all of them. Results do not depend on the number of threads.
struct ToleranceConfig {
    int samples = 10000;
    std::uint64_t seed = 42;
    bool uniform = false;               ///< uniform instead of normal distributions

    // +- tolerances in mm
    double position_tol = 0.010;        ///< each coordinate of every pivot position
    double tooth_thickness_tol = 0.006; ///< tooth thickness at the pitch circle, per wheel
    double pivot_diameter_tol = 0.003;
    double hole_diameter_tol = 0.004;
    double arbor_length_tol = 0.020;    ///< shoulder-to-shoulder length between pivot ends
    double bridge_gap_tol = 0.015;      ///< plate-to-bridge distance
    double plate_flatness_tol = 0.008;

    // Nominal design
    double nominal_backlash_factor = 0.08;  ///< circumferential backlash at nominal, in modules
    double nominal_end_shake = 0.030;       ///< mm
    double nominal_side_shake = 0.020;      ///< mm, hole diameter minus pivot diameter

    // Acceptance
    double min_backlash = 0.0;              ///< mm; below this the teeth bind
    double min_contact_ratio = 1.1;         ///< below this the drive is not continuous
    double end_shake_min = 0.010, end_shake_max = 0.080;
    double side_shake_min = 0.004, side_shake_max = 0.040;
};

/// `[tolerance]` table with the field names above; unknown keys are an error.
ToleranceConfig tolerance_from_toml(const std::string& text);
ToleranceConfig load_tolerance(const std::string& path);

struct Stat {
    double mean{}, sd{}, min{}, max{}, p01{}, p99{};
};

struct Sensitivity {
    std::string input;
    double correlation{};  ///< with the output, in [-1, 1]
    double share{};        ///< share of the explained variance, sums to 1 over the inputs
};

struct MeshTolerance {
    std::string a, b;               ///< component names of the wheel and pinion
    double nominal_centre_distance{};
    double nominal_backlash{};      ///< mm, circumferential
    double nominal_contact_ratio{};
    Stat centre_distance;
    Stat backlash;
    Stat contact_ratio;
    double bind_fraction{};
    double under_engage_fraction{};
    std::vector<Sensitivity> backlash_sensitivity;  ///< largest share first
};

struct MobileTolerance {
    std::string mobile;
    Stat end_shake;
    Stat side_shake;
    double end_shake_out_fraction{};
    double side_shake_out_fraction{};
    std::vector<Sensitivity> end_shake_sensitivity;
    std::vector<Sensitivity> side_shake_sensitivity;
};

struct ToleranceResult {
    int samples{};
    std::uint64_t seed{};
    std::vector<MeshTolerance> meshes;
    std::vector<MobileTolerance> mobiles;
    double failing_fraction{};  ///< builds in which anything binds, under-engages or leaves a shake window
};

/// Runs the analysis on the meshes and mobiles of the rig. Throws ohs::Error when the rig has no
/// meshes or the configuration is invalid.
ToleranceResult run_tolerance(const model::Movement& movement, const ToleranceConfig& config);

}  // namespace ohs::manufacturing
