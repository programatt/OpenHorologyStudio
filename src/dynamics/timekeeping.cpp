// SPDX-License-Identifier: Apache-2.0
#include "dynamics/timekeeping.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "core/error.hpp"

namespace ohs::dynamics {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

}  // namespace

double full_wind_hours(const Calibre& c, const TorqueChain& chain) {
    return c.barrel_turns * chain.barrel_period_hours;
}

WindPoint wind_point(const Calibre& c, const TorqueChain& chain, double wind, Position position) {
    WindPoint p;
    p.wind = wind;
    p.hours = (1.0 - wind) * full_wind_hours(c, chain);
    p.position = position;
    p.barrel_torque_nmm = c.mainspring_torque_nmm(wind);
    p.wheel_torque_nm = escape_torque_nm(chain, c, wind);
    if (p.wheel_torque_nm <= 0) {
        p.stalled = true;
        return p;
    }
    const SteadyState ss = steady_state(c, p.wheel_torque_nm, position);
    p.amplitude_deg = ss.m.amplitude_deg;
    p.rate_s_per_day = ss.m.rate_s_per_day;
    p.beat_error_ms = ss.m.beat_error_ms;
    p.stalled = ss.stalled;
    return p;
}

std::vector<WindPoint> amplitude_vs_wind(const Calibre& c, const TorqueChain& chain, int samples, Position position) {
    if (samples < 2) throw Error("need at least two samples of wind");
    std::vector<WindPoint> out;
    for (int i = 0; i < samples; ++i) {
        out.push_back(wind_point(c, chain, 1.0 - static_cast<double>(i) / (samples - 1), position));
    }
    return out;
}

double power_reserve_hours(const Calibre& c, const TorqueChain& chain, Position position) {
    auto running = [&](double wind) {
        const WindPoint p = wind_point(c, chain, wind, position);
        return !p.stalled && p.amplitude_deg >= c.min_running_amplitude_deg;
    };
    const double total = full_wind_hours(c, chain);
    if (!running(1.0)) return 0.0;
    if (running(0.0)) return total;
    double lo = 0.0, hi = 1.0;  // lo: stopped, hi: running
    for (int i = 0; i < 14; ++i) {
        const double mid = 0.5 * (lo + hi);
        (running(mid) ? hi : lo) = mid;
    }
    return (1.0 - hi) * total;
}

double isochronism_slope(const std::vector<WindPoint>& points) {
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const WindPoint& p : points) {
        if (p.stalled || p.amplitude_deg <= 0) continue;
        ++n;
        sx += p.amplitude_deg;
        sy += p.rate_s_per_day;
        sxx += p.amplitude_deg * p.amplitude_deg;
        sxy += p.amplitude_deg * p.rate_s_per_day;
    }
    const double den = n * sxx - sx * sx;
    if (n < 2 || std::abs(den) < 1e-12) return 0.0;
    return (n * sxy - sx * sy) / den;
}

PositionalResult positional_error(const Calibre& c, const TorqueChain& chain, double wind) {
    PositionalResult r;
    double lo = 1e300, hi = -1e300, flat = 0, vert = 0;
    int nflat = 0, nvert = 0;
    for (Position pos : all_positions()) {
        const WindPoint p = wind_point(c, chain, wind, pos);
        r.points.push_back(p);
        lo = std::min(lo, p.rate_s_per_day);
        hi = std::max(hi, p.rate_s_per_day);
        (is_vertical(pos) ? vert : flat) += p.amplitude_deg;
        (is_vertical(pos) ? nvert : nflat) += 1;
    }
    r.spread_s_per_day = hi - lo;
    r.flat_to_vertical_amplitude_deg = flat / nflat - vert / nvert;
    return r;
}

EnergyBudget energy_budget(const Calibre& c, double wheel_torque_nm, double amplitude_deg, Position position) {
    const EscapementGeometry g = escapement_geometry(c, wheel_torque_nm);
    const double amp = amplitude_deg * kDeg;
    const double w0 = std::sqrt(c.hairspring_stiffness() / c.inertia());
    EnergyBudget b;
    b.wheel_work_j = wheel_torque_nm * g.wheel_step;
    b.delivered_j = g.impulse_torque * g.lift;
    b.lost_unlock_j = g.unlock_torque * g.unlock;
    const double half = 0.5 * g.lift;
    const double hit_speed2 = w0 * w0 * std::max(amp * amp - half * half, 0.0);
    b.lost_wheel_hit_j = c.escape_wheel_inertia() * g.ratio * g.ratio * hit_speed2;  // picked up, then dropped
    b.lost_friction_j = 2.0 * c.friction_torque(position) * amp;
    b.lost_air_j = 0.5 * kPi * c.air_damping_nms * w0 * amp * amp;
    b.efficiency = b.wheel_work_j > 0 ? (b.delivered_j - b.lost_unlock_j - b.lost_wheel_hit_j) / b.wheel_work_j : 0.0;
    return b;
}

Calibre regulate(const Calibre& calibre, const TorqueChain& chain, double target, double wind) {
    Calibre c = calibre;
    auto rate_at = [&](double length_mm) {
        c.hairspring_length_mm = length_mm;
        return wind_point(c, chain, wind).rate_s_per_day;
    };
    // Rate falls as the spring gets longer; f ~ L^-1/2 gives a first slope, then secant steps.
    double l0 = calibre.hairspring_length_mm;
    double r0 = rate_at(l0);
    double l1 = l0 * (1.0 + (r0 - target) / 86400.0 * 2.0);
    double r1 = rate_at(l1);
    for (int i = 0; i < 6 && std::abs(r1 - target) > 0.01; ++i) {
        if (r1 == r0) break;
        const double l2 = l1 + (target - r1) * (l1 - l0) / (r1 - r0);
        l0 = l1;
        r0 = r1;
        l1 = l2;
        r1 = rate_at(l1);
    }
    c.hairspring_length_mm = l1;
    return c;
}

Timekeeping evaluate(const Calibre& c, const TorqueChain& chain) {
    Timekeeping t;
    t.full = wind_point(c, chain, 1.0);
    t.half = wind_point(c, chain, 0.5);
    t.low = wind_point(c, chain, 0.1);
    t.isochronism_s_per_day_per_deg = isochronism_slope({t.full, t.half, t.low});
    t.full_wind_h = full_wind_hours(c, chain);
    t.power_reserve_h = power_reserve_hours(c, chain);
    t.positions = positional_error(c, chain, 0.5);
    t.budget = energy_budget(c, t.full.wheel_torque_nm, t.full.amplitude_deg, Position::DialUp);
    return t;
}

}  // namespace ohs::dynamics
