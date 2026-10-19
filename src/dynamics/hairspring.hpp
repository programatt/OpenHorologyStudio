// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

namespace ohs::dynamics {

/// A flat Archimedean hairspring: radius r(phi) = inner_radius + pitch * phi / (2 pi) over `turns` turns, a
/// rectangular strip, stud end clamped, collet end fixed to the staff (which sits at the spiral's centre).
struct HairspringSpec {
    double youngs_gpa = 210.0;
    double width_mm = 0.14;       ///< height of the strip along the staff
    double thickness_mm = 0.025;  ///< radial thickness
    double inner_radius_mm = 0.6;
    double pitch_mm = 0.12;       ///< radial distance between coils
    double turns = 12.0;
    int elements_per_turn = 24;

    double length_mm() const;     ///< length of the strip
    double euler_stiffness() const;  ///< E I / L in N·m/rad: what a spring turned by a pure moment would have
};

struct HairspringPoint {
    double angle_rad{};       ///< collet rotation, positive = winding the spring up
    double torque_nm{};       ///< moment the spring exerts on the staff about its axis, sign as the angle
    double side_force_n{};    ///< magnitude of the force the staff pivots must supply: zero for a perfectly "breathing" spring
};

/// Equilibrium of the spring (geometrically nonlinear beam elements, load stepped from zero) at each collet
/// rotation. Angles may be positive or negative. Throws ohs::Error if the iteration does not converge.
std::vector<HairspringPoint> hairspring_curve(const HairspringSpec& spec, const std::vector<double>& angles_rad);

struct HairspringFit {
    double stiffness_nm_rad{};  ///< k in torque = k a + c a^3 (the part odd in the angle)
    double cubic_nm_rad3{};     ///< c: positive stiffens at large angles, negative softens
    double asymmetry{};         ///< largest even part of the torque over the largest odd part (0 for a symmetric spring)
    double euler_stiffness{};   ///< E I / L for comparison
    double max_side_force_n{};  ///< over the fitted range
};

/// Fits torque against angle over +-amplitude (a few evenly spaced angles each side).
HairspringFit fit_hairspring(const HairspringSpec& spec, double amplitude_rad);

}  // namespace ohs::dynamics
