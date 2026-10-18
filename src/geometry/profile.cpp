// SPDX-License-Identifier: Apache-2.0
#include "geometry/profile.hpp"

#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

namespace ohs::geometry {

namespace {
// A harmonic must reach this fraction of the mean radius to count as toothing.
constexpr double kMinRelativeAmplitude = 0.005;
// A lower harmonic is the true fundamental if it reaches this share of the strongest one.
constexpr double kSubharmonicShare = 0.3;
// Stricter when the lower harmonic would disqualify the part as a gear altogether.
constexpr double kLowFundamentalShare = 0.5;
}  // namespace

ToothResult detect_teeth(std::span<const double> radii, int min_teeth) {
    const int n = static_cast<int>(radii.size());
    if (n < 8 || min_teeth < 2 || min_teeth >= n / 2) return {};

    const double mean = std::accumulate(radii.begin(), radii.end(), 0.0) / n;
    if (mean <= 0.0) return {};

    std::vector<double> cos_table(n);
    std::vector<double> sin_table(n);
    for (int i = 0; i < n; ++i) {
        cos_table[i] = std::cos(2.0 * std::numbers::pi * i / n);
        sin_table[i] = std::sin(2.0 * std::numbers::pi * i / n);
    }

    const int max_k = n / 2 - 1;
    std::vector<double> amp(max_k + 1, 0.0);
    double total_energy = 0.0;
    for (int k = 2; k <= max_k; ++k) {
        double re = 0.0;
        double im = 0.0;
        for (int i = 0; i < n; ++i) {
            const int idx = static_cast<int>((static_cast<long long>(k) * i) % n);
            re += (radii[i] - mean) * cos_table[idx];
            im += (radii[i] - mean) * sin_table[idx];
        }
        amp[k] = 2.0 / n * std::hypot(re, im);
        total_energy += amp[k] * amp[k];
    }

    int best = min_teeth;
    for (int k = min_teeth; k <= max_k; ++k) {
        if (amp[k] > amp[best]) best = k;
    }
    if (amp[best] < kMinRelativeAmplitude * mean) return {};

    // Narrow teeth put as much energy in the harmonics as in the fundamental; walk down to it.
    // A fundamental below min_teeth (e.g. the 4-fold harmonic of a square) means "not a gear".
    for (bool again = true; again;) {
        again = false;
        for (int d = 2; d <= best / 2; ++d) {
            const double share = best / d >= min_teeth ? kSubharmonicShare : kLowFundamentalShare;
            if (best % d == 0 && amp[best / d] >= share * amp[best]) {
                best /= d;
                again = true;
                break;
            }
        }
    }
    if (best < min_teeth) return {};

    double harmonic_energy = 0.0;
    for (int k = best; k <= max_k; k += best) harmonic_energy += amp[k] * amp[k];
    return {best, total_energy > 0.0 ? harmonic_energy / total_energy : 0.0};
}

}  // namespace ohs::geometry
