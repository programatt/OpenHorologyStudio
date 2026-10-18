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

}  // namespace ohs::geometry
