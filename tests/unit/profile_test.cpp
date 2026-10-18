// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "geometry/profile.hpp"

using ohs::geometry::detect_teeth;

namespace {
// Trapezoidal tooth wave: duty is the fraction of each pitch spent at the tip radius.
std::vector<double> toothed(int teeth, double tip, double root, double duty, int bins = 1440) {
    std::vector<double> r(bins);
    for (int i = 0; i < bins; ++i) {
        const double phase = std::fmod(static_cast<double>(i) * teeth / bins, 1.0);
        r[i] = phase < duty ? tip : root;
    }
    return r;
}
}  // namespace

TEST_CASE("tooth counts are recovered across tooth counts and duty cycles") {
    const int teeth = GENERATE(6, 8, 15, 60, 80, 84, 120, 200);
    const double duty = GENERATE(0.25, 0.5, 0.75);
    const auto result = detect_teeth(toothed(teeth, 6.0, 5.7, duty), 6);
    REQUIRE(result.teeth == teeth);
    REQUIRE(result.confidence > 0.8);
}

TEST_CASE("plain outlines are not toothed") {
    REQUIRE(detect_teeth(std::vector<double>(1440, 4.0), 6).teeth == 0);

    std::vector<double> ellipse(1440), square(1440);
    for (int i = 0; i < 1440; ++i) {
        const double a = 2 * std::numbers::pi * i / 1440;
        ellipse[i] = 4.0 + 0.3 * std::cos(2 * a);
        square[i] = 5.0 + 0.5 * std::cos(4 * a);
    }
    REQUIRE(detect_teeth(ellipse, 6).teeth == 0);
    REQUIRE(detect_teeth(square, 6).teeth == 0);
}

TEST_CASE("degenerate input is rejected without crashing") {
    REQUIRE(detect_teeth({}, 6).teeth == 0);
    REQUIRE(detect_teeth(std::vector<double>(4, 1.0), 6).teeth == 0);
    REQUIRE(detect_teeth(std::vector<double>(1440, 0.0), 6).teeth == 0);
}
