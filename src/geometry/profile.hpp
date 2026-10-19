// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <span>

namespace ohs::geometry {

struct ToothResult {
    int teeth{};          ///< 0 when the outline is not periodic enough to be toothed
    double confidence{};  ///< share of the outline's spectral energy at multiples of the tooth count
};

/// Counts teeth from an outline sampled as radius vs. angle (uniform angular bins, closed loop)
/// using the dominant Fourier harmonic of at least `min_teeth`.
ToothResult detect_teeth(std::span<const double> radii, int min_teeth);

/// Angle in [0, 2pi/teeth) at which the outline's teeth peak, from the phase of the tooth harmonic.
/// Bin i is taken to span angles [i, i+1) * 2pi/n.
double tooth_phase(std::span<const double> radii, int teeth);

/// One outline sample in polar form about the part centre; theta in [0, 2pi).
struct PolarPoint {
    double r{};
    double theta{};
};

struct FlankFit {
    bool ok{false};
    double base_radius{};  ///< base circle of the best-fit involute
    double rms{};          ///< rms distance of the sampled flank from the fit, in the outline's units
};

/// Fits an involute flank u(r) = s0 - inv(acos(rb / r)) to the middle 60 % of the tooth depth,
/// pooling all teeth and both flanks of a symmetric tooth. `phase` comes from tooth_phase().
FlankFit fit_involute_flank(std::span<const PolarPoint> points, int teeth, double phase,
                            double root_radius, double tip_radius);

struct CycloidFit {
    bool ok{false};
    double pitch_radius{};    ///< pitch circle of the best-fit cycloidal flank
    double rolling_radius{};  ///< radius of the circle that rolls to generate both flanks
    double rms{};             ///< rms distance of the sampled flank from the fit, in the outline's units
};

/// Fits a cycloidal flank (epicycloid above the pitch circle, hypocycloid below, one rolling circle)
/// to the middle 60 % of the tooth depth, with the pitch radius and the rolling radius free. It is
/// meant for flanks the involute fit does not explain; a free pitch radius can make nearly any smooth
/// flank fit, so callers should check the fitted pitch radius against the module they trust.
CycloidFit fit_cycloidal_flank(std::span<const PolarPoint> points, int teeth, double phase,
                               double root_radius, double tip_radius);

}  // namespace ohs::geometry
