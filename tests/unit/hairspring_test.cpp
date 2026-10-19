// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "dynamics/hairspring.hpp"

using namespace ohs;
using namespace ohs::dynamics;
using Catch::Approx;

TEST_CASE("hairspring FE: the small-angle stiffness is E I / L, and the length is the spiral's arc length") {
    HairspringSpec s;
    // Arc length of the Archimedean spiral against a numerical sum of its segments.
    double sum = 0;
    const double dr = s.pitch_mm / (2 * M_PI);
    const int n = 200000;
    const double phi_end = 2 * M_PI * s.turns;
    for (int i = 0; i < n; ++i) {
        const double phi = phi_end * (i + 0.5) / n, r = s.inner_radius_mm + dr * phi;
        sum += std::hypot(r, dr) * phi_end / n;
    }
    CHECK(s.length_mm() == Approx(sum).epsilon(1e-6));

    const auto c = hairspring_curve(s, {0.01});
    CHECK(c[0].torque_nm / 0.01 == Approx(s.euler_stiffness()).epsilon(0.01));  // 1 %: the polygon is a little short of the arc
    CHECK(c[0].side_force_n < 1e-6);  // a concentric spiral barely pushes the staff sideways
}

TEST_CASE("hairspring FE: large collet rotations, symmetric torque, a side force that grows with the angle") {
    HairspringSpec s;
    const auto c = hairspring_curve(s, {1.0, -1.0, 4.0, -4.0});  // 4 rad is over 229 degrees, past pi
    CHECK(c[0].torque_nm == Approx(-c[1].torque_nm).epsilon(1e-3));
    CHECK(c[2].torque_nm == Approx(-c[3].torque_nm).epsilon(1e-3));
    CHECK(c[2].torque_nm / 4.0 == Approx(c[0].torque_nm / 1.0).epsilon(0.01));  // nearly linear
    CHECK(c[2].side_force_n > 2 * c[0].side_force_n);

    // A thicker strip is stiffer as t^3; a longer spring is softer as 1 / L.
    HairspringSpec thick = s;
    thick.thickness_mm *= 1.2;
    CHECK(hairspring_curve(thick, {0.01})[0].torque_nm / hairspring_curve(s, {0.01})[0].torque_nm == Approx(1.2 * 1.2 * 1.2).epsilon(0.01));
    HairspringSpec longer = s;
    longer.turns = 14;
    const double ratio = hairspring_curve(longer, {0.01})[0].torque_nm / hairspring_curve(s, {0.01})[0].torque_nm;
    CHECK(ratio == Approx(s.length_mm() / longer.length_mm()).epsilon(0.02));
}

TEST_CASE("hairspring FE: the fit gives a stiffness and a small cubic term, and bad input is refused") {
    HairspringSpec s;
    const HairspringFit f = fit_hairspring(s, 3.0);
    CHECK(f.stiffness_nm_rad == Approx(s.euler_stiffness()).epsilon(0.01));
    CHECK(std::abs(f.cubic_nm_rad3 * 9.0) < 0.01 * f.stiffness_nm_rad);  // under 1 % at 3 rad: a good hairspring is nearly linear
    CHECK(f.asymmetry < 1e-2);
    CHECK(f.max_side_force_n > 0);
    CHECK_THROWS_AS(fit_hairspring(s, 0.0), Error);
    HairspringSpec bad = s;
    bad.turns = 0;
    CHECK_THROWS_AS(hairspring_curve(bad, {0.1}), Error);
}
