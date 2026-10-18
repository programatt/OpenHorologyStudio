// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "dynamics/oscillator.hpp"

namespace ohs::dynamics {

/// One tick or tock as a microphone on the case would hear it: the lever starting to push the
/// balance (tick time), and the drop at the end of the impulse.
struct TimegrapherBeat {
    int index{};
    double time_s{};          ///< of the tick
    double lift_time_ms{};    ///< from the tick to the drop
    double interval_ms{};     ///< since the previous tick (0 for the first)
    double deviation_ms{};    ///< against an ideal clock beating at the nominal rate
    bool tock{false};         ///< alternate beats
};

/// What a timegrapher prints: derived only from the sounds, never from the simulation's own state.
struct TimegrapherReading {
    int beats{};
    double rate_s_per_day{};
    double amplitude_deg{};   ///< lift / (2 sin(pi * lift_time / period))
    double beat_error_ms{};
};

struct TimegrapherTrace {
    std::vector<TimegrapherBeat> beats;
    TimegrapherReading reading;
};

/// Builds the trace from level-2 or level-3 events in [t_from, t_to]. Timing jitter in
/// microseconds (rms) is added with a seeded generator, so a run is reproducible on any machine.
TimegrapherTrace virtual_timegrapher(const std::vector<Event>& events, const Calibre& calibre, double t_from,
                                     double t_to, double jitter_us = 0.0, std::uint64_t seed = 1);

/// Classic timegrapher plot: the deviation of ticks (o) and tocks (x) from the nominal beat
/// against beat number; a flat line means zero rate, a slope means a rate, a gap means beat error.
std::string trace_ascii(const TimegrapherTrace& trace, int width = 72, int height = 15);

/// index,time_s,tock,interval_ms,deviation_ms,lift_time_ms
std::string trace_csv(const TimegrapherTrace& trace);

}  // namespace ohs::dynamics
