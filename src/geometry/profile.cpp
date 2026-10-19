// SPDX-License-Identifier: Apache-2.0
#include "geometry/profile.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "gear/cycloid.hpp"

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

double tooth_phase(std::span<const double> radii, int teeth) {
    const int n = static_cast<int>(radii.size());
    if (n == 0 || teeth <= 0) return 0.0;
    const double mean = std::accumulate(radii.begin(), radii.end(), 0.0) / n;
    double re = 0.0;
    double im = 0.0;
    for (int i = 0; i < n; ++i) {
        const double theta = 2.0 * std::numbers::pi * (i + 0.5) / n;
        re += (radii[i] - mean) * std::cos(teeth * theta);
        im -= (radii[i] - mean) * std::sin(teeth * theta);
    }
    const double pitch = 2.0 * std::numbers::pi / teeth;
    double phase = -std::atan2(im, re) / teeth;
    phase = std::fmod(phase, pitch);
    return phase < 0 ? phase + pitch : phase;
}

namespace {
double involute(double alpha) { return std::tan(alpha) - alpha; }
}  // namespace

FlankFit fit_involute_flank(std::span<const PolarPoint> points, int teeth, double phase,
                            double root_radius, double tip_radius) {
    if (teeth < 3 || tip_radius <= root_radius) return {};
    const double pitch = 2.0 * std::numbers::pi / teeth;
    const double depth = tip_radius - root_radius;
    const double lo = root_radius + 0.2 * depth;
    const double hi = tip_radius - 0.2 * depth;

    std::vector<std::pair<double, double>> flank;  // (r, |angle from tooth centre|)
    double rmin = hi;
    for (const PolarPoint& p : points) {
        if (p.r < lo || p.r > hi) continue;
        double u = std::fmod(p.theta - phase, pitch);
        u -= pitch * std::round(u / pitch);
        flank.emplace_back(p.r, std::abs(u));
        rmin = std::min(rmin, p.r);
    }
    if (flank.size() < 30) return {};

    // The fit only needs a few thousand samples.
    constexpr std::size_t kMaxSamples = 2000;
    if (flank.size() > kMaxSamples) {
        std::vector<std::pair<double, double>> thinned;
        const double stride = static_cast<double>(flank.size()) / kMaxSamples;
        for (std::size_t i = 0; i < kMaxSamples; ++i) thinned.push_back(flank[static_cast<std::size_t>(i * stride)]);
        flank = std::move(thinned);
    }

    auto rms_for = [&flank](double rb, double& rms) {
        double s0 = 0.0;
        std::vector<double> x(flank.size());
        for (std::size_t i = 0; i < flank.size(); ++i) {
            x[i] = involute(std::acos(rb / flank[i].first));
            s0 += flank[i].second + x[i];
        }
        s0 /= static_cast<double>(flank.size());
        double sum = 0.0;
        for (std::size_t i = 0; i < flank.size(); ++i) {
            const double d = flank[i].first * (flank[i].second - (s0 - x[i]));
            sum += d * d;
        }
        rms = std::sqrt(sum / static_cast<double>(flank.size()));
    };

    // Coarse scan of the base radius, then repeated zooming around the best value.
    double lo_rb = rmin * 0.3;
    double hi_rb = rmin * 0.9999;
    FlankFit best;
    best.rms = 1e300;
    for (int pass = 0; pass < 6; ++pass) {
        constexpr int kSteps = 40;
        const double width = (hi_rb - lo_rb) / (kSteps - 1);
        for (int step = 0; step < kSteps; ++step) {
            const double rb = lo_rb + width * step;
            double rms = 0.0;
            rms_for(rb, rms);
            if (rms < best.rms) best = {true, rb, rms};
        }
        lo_rb = std::max(rmin * 0.3, best.base_radius - 2 * width);
        hi_rb = std::min(rmin * 0.9999, best.base_radius + 2 * width);
    }
    return best;
}

CycloidFit fit_cycloidal_flank(std::span<const PolarPoint> points, int teeth, double phase,
                               double root_radius, double tip_radius) {
    if (teeth < 3 || tip_radius <= root_radius) return {};
    const double pitch = 2.0 * std::numbers::pi / teeth;
    const double depth = tip_radius - root_radius;
    const double lo = root_radius + 0.2 * depth;
    const double hi = tip_radius - 0.2 * depth;

    std::vector<std::pair<double, double>> flank;  // (r, |angle from tooth centre|)
    for (const PolarPoint& p : points) {
        if (p.r < lo || p.r > hi) continue;
        double u = std::fmod(p.theta - phase, pitch);
        u -= pitch * std::round(u / pitch);
        flank.emplace_back(p.r, std::abs(u));
    }
    if (flank.size() < 30) return {};
    constexpr std::size_t kMaxSamples = 400;
    if (flank.size() > kMaxSamples) {
        std::vector<std::pair<double, double>> thinned;
        const double stride = static_cast<double>(flank.size()) / kMaxSamples;
        for (std::size_t i = 0; i < kMaxSamples; ++i) thinned.push_back(flank[static_cast<std::size_t>(i * stride)]);
        flank = std::move(thinned);
    }

    // rms for a pitch radius R and rolling radius rho; false when the flank leaves the curve's range.
    auto rms_for = [&](double R, double rho, double& rms) {
        if (rho <= 0.0 || rho >= R || lo < std::abs(R - 2 * rho) * 1.01 || hi > (R + 2 * rho) * 0.99) return false;
        std::vector<double> x(flank.size());
        double s0 = 0.0;
        for (std::size_t i = 0; i < flank.size(); ++i) {
            const double r = flank[i].first;
            const double d = gear::cycloid_offset(R, rho, r);
            x[i] = r >= R ? -d : d;  // flank angle relative to the angle at the pitch circle
            s0 += flank[i].second - x[i];
        }
        s0 /= static_cast<double>(flank.size());
        double sum = 0.0;
        for (std::size_t i = 0; i < flank.size(); ++i) {
            const double e = flank[i].first * (flank[i].second - (s0 + x[i]));
            sum += e * e;
        }
        rms = std::sqrt(sum / static_cast<double>(flank.size()));
        return true;
    };

    // Coarse scan of the pitch and rolling radii, then repeated zooming around the best pair.
    double r_lo = root_radius + 0.25 * depth, r_hi = tip_radius - 0.1 * depth;
    double p_lo = 0.1, p_hi = 0.95;  // rolling radius as a fraction of the pitch radius
    CycloidFit best;
    best.rms = 1e300;
    for (int pass = 0; pass < 6; ++pass) {
        constexpr int kSteps = 40;
        const double dr = (r_hi - r_lo) / (kSteps - 1), dp = (p_hi - p_lo) / (kSteps - 1);
        for (int i = 0; i < kSteps; ++i) {
            for (int j = 0; j < kSteps; ++j) {
                const double R = r_lo + dr * i, frac = p_lo + dp * j;
                double rms = 0.0;
                if (rms_for(R, frac * R, rms) && rms < best.rms) best = {true, R, frac * R, rms};
            }
        }
        if (!best.ok) return {};
        r_lo = std::max(root_radius + 0.25 * depth, best.pitch_radius - 2 * dr);
        r_hi = std::min(tip_radius - 0.1 * depth, best.pitch_radius + 2 * dr);
        const double frac = best.rolling_radius / best.pitch_radius;
        p_lo = std::max(0.1, frac - 2 * dp);
        p_hi = std::min(0.95, frac + 2 * dp);
    }
    return best;
}

}  // namespace ohs::geometry
