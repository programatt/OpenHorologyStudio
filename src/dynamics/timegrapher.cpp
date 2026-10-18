// SPDX-License-Identifier: Apache-2.0
#include "dynamics/timegrapher.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <random>
#include <sstream>

namespace ohs::dynamics {

namespace {

/// Standard normal from the raw mt19937_64 stream (Box-Muller), identical on every platform.
class Gaussian {
public:
    explicit Gaussian(std::uint64_t seed) : rng_(seed) {}
    double next() {
        const double u1 = (static_cast<double>(rng_() >> 11) + 0.5) / 9007199254740992.0;
        const double u2 = (static_cast<double>(rng_() >> 11) + 0.5) / 9007199254740992.0;
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
    }

private:
    std::mt19937_64 rng_;
};

}  // namespace

TimegrapherTrace virtual_timegrapher(const std::vector<Event>& events, const Calibre& c, double t_from, double t_to,
                                     double jitter_us, std::uint64_t seed) {
    TimegrapherTrace out;
    Gaussian noise(seed);
    const double jitter = jitter_us * 1e-6;
    const Event* open = nullptr;
    for (const Event& e : events) {
        if (e.t < t_from || e.t > t_to) continue;
        if (e.type == EventType::ImpulseStart) {
            open = &e;
        } else if (e.type == EventType::Drop && open && open->direction == e.direction) {
            TimegrapherBeat b;
            b.time_s = open->t + (jitter > 0 ? jitter * noise.next() : 0.0);
            const double drop = e.t + (jitter > 0 ? jitter * noise.next() : 0.0);
            b.lift_time_ms = (drop - b.time_s) * 1e3;
            out.beats.push_back(b);
            open = nullptr;
        }
    }
    const int n = static_cast<int>(out.beats.size());
    out.reading.beats = n;
    if (n < 3) return out;

    const double nominal = c.beat_period_s();
    // Least-squares line through tick time against beat number: its slope is the beat interval.
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        const double y = out.beats[i].time_s;
        sx += i;
        sy += y;
        sxx += static_cast<double>(i) * i;
        sxy += i * y;
    }
    const double den = n * sxx - sx * sx;
    const double slope = (n * sxy - sx * sy) / den;
    const double offset = (sy - slope * sx) / n;

    double even = 0, odd = 0, lift = 0;
    int ne = 0, no = 0;
    for (int i = 0; i < n; ++i) {
        TimegrapherBeat& b = out.beats[i];
        b.index = i;
        b.tock = (i % 2) == 1;
        b.deviation_ms = (b.time_s - (offset + nominal * i)) * 1e3;
        if (i > 0) {
            b.interval_ms = (b.time_s - out.beats[i - 1].time_s) * 1e3;
            (b.tock ? odd : even) += b.interval_ms;
            ++(b.tock ? no : ne);
        }
        lift += b.lift_time_ms;
    }
    out.reading.rate_s_per_day = (nominal / slope - 1.0) * 86400.0;
    if (ne > 0 && no > 0) out.reading.beat_error_ms = std::abs(even / ne - odd / no) / 2.0;

    const double period = 2.0 * slope;
    const double ratio = std::clamp((lift / n * 1e-3) / period, 1e-6, 0.5);
    out.reading.amplitude_deg = c.lift_angle_deg / (2.0 * std::sin(std::numbers::pi * ratio));
    return out;
}

std::string trace_ascii(const TimegrapherTrace& trace, int width, int height) {
    std::ostringstream os;
    const int n = static_cast<int>(trace.beats.size());
    if (n < 3 || width < 10 || height < 3) return "(not enough beats for a trace)\n";
    double lo = 1e300, hi = -1e300;
    for (const auto& b : trace.beats) {
        lo = std::min(lo, b.deviation_ms);
        hi = std::max(hi, b.deviation_ms);
    }
    if (hi - lo < 1e-6) {
        lo -= 0.5;
        hi += 0.5;
    }
    std::vector<std::string> grid(height, std::string(width, ' '));
    for (const auto& b : trace.beats) {
        const int x = std::min(width - 1, static_cast<int>(static_cast<double>(b.index) / n * width));
        const int y = std::min(height - 1, static_cast<int>((hi - b.deviation_ms) / (hi - lo) * (height - 1) + 0.5));
        grid[y][x] = b.tock ? 'x' : 'o';
    }
    os << std::fixed << std::setprecision(2);
    for (int r = 0; r < height; ++r) {
        const double v = hi - (hi - lo) * r / (height - 1);
        os << std::setw(7) << v << " |" << grid[r] << "\n";
    }
    os << "        +" << std::string(width, '-') << "\n";
    os << "         beats 0.." << n - 1 << "   o tick  x tock   (deviation in ms from the nominal beat)\n";
    os << std::setprecision(1) << "rate " << std::showpos << trace.reading.rate_s_per_day << std::noshowpos
       << " s/d   amplitude " << trace.reading.amplitude_deg << " deg   beat error " << std::setprecision(2)
       << trace.reading.beat_error_ms << " ms\n";
    return os.str();
}

std::string trace_csv(const TimegrapherTrace& trace) {
    std::ostringstream os;
    os << "index,time_s,tock,interval_ms,deviation_ms,lift_time_ms\n" << std::setprecision(9);
    for (const auto& b : trace.beats) {
        os << b.index << ',' << b.time_s << ',' << (b.tock ? 1 : 0) << ',' << b.interval_ms << ',' << b.deviation_ms
           << ',' << b.lift_time_ms << '\n';
    }
    return os.str();
}

}  // namespace ohs::dynamics
