// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ohs::gear {

/// Polar angle (radians) of the point of a cycloidal tooth flank at radius r, measured from the ray
/// through the pitch point and always >= 0 for r above the pitch radius R (the addendum, an
/// epicycloid traced by a circle of radius rho rolling outside the pitch circle) and for r below it
/// when rho < R / 2 (the dedendum, a hypocycloid traced by a circle rolling inside). A rolling circle
/// of exactly R / 2 gives a radial dedendum flank; a larger one undercuts the root, so the angle is
/// negative there. Valid for R < r <= R + 2 rho (epicycloid) and |R - 2 rho| <= r < R (hypocycloid),
/// with 0 < rho < R; values outside are clamped to the cusp.
inline double cycloid_offset(double R, double rho, double r) {
    if (r >= R) {
        const double c = ((R + rho) * (R + rho) + rho * rho - r * r) / (2.0 * (R + rho) * rho);
        const double beta = std::acos(std::clamp(c, -1.0, 1.0));  // rolling angle R * alpha / rho
        const double alpha = rho * beta / R;
        const double turn = (R + rho) * alpha / rho;
        return std::atan2((R + rho) * std::sin(alpha) - rho * std::sin(turn), (R + rho) * std::cos(alpha) - rho * std::cos(turn));
    }
    const double c = (r * r - (R - rho) * (R - rho) - rho * rho) / (2.0 * (R - rho) * rho);
    const double beta = std::acos(std::clamp(c, -1.0, 1.0));
    const double alpha = rho * beta / R;
    const double turn = (R - rho) * alpha / rho;
    return std::atan2((R - rho) * std::sin(alpha) - rho * std::sin(turn), (R - rho) * std::cos(alpha) + rho * std::cos(turn));
}

/// Half the angular tooth thickness at radius r of a z-tooth wheel whose flanks are cycloids of
/// rolling radius rho on a pitch circle R, with tooth and gap equal at the pitch circle.
inline double cycloidal_half_angle(int teeth, double R, double rho, double r) {
    const double half_pitch = std::numbers::pi / (2.0 * teeth);
    const double d = cycloid_offset(R, rho, r);
    return r >= R ? half_pitch - d : half_pitch + d;
}

}  // namespace ohs::gear
