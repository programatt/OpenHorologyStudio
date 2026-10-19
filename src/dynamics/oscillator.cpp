// SPDX-License-Identifier: Apache-2.0
#include "dynamics/oscillator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "core/error.hpp"

namespace ohs::dynamics {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

enum Zone { kFree = 0, kUnlock = 1, kImpulse = 2, kPass = 3 };

double crown_phase(Position p) {
    switch (p) {
        case Position::CrownUp: return 0.0;
        case Position::CrownRight: return kPi / 2;
        case Position::CrownDown: return kPi;
        case Position::CrownLeft: return 3 * kPi / 2;
        default: return 0.0;
    }
}

/// The equations of motion of the balance in SI units, shared by the stepper and the estimate.
struct Model {
    double inertia, wheel_inertia, stiffness, rest, damping, friction;
    double gravity_torque, gravity_phase, lead;
    EscapementType type;
    EscapementGeometry esc;

    Model(const Calibre& c, double wheel_torque, Position p)
        : inertia(c.inertia()),
          wheel_inertia(c.escape_wheel_inertia()),
          stiffness(c.hairspring_stiffness()),
          rest(c.beat_offset_deg * kDeg),
          damping(c.air_damping_nms),
          friction(c.friction_torque(p)),
          gravity_torque(is_vertical(p) ? c.unpoise_torque() : 0.0),
          gravity_phase(c.unpoise_angle_deg * kDeg + crown_phase(p)),
          lead(c.impulse_lead_deg * kDeg),
          type(c.escapement),
          esc(escapement_geometry(c, wheel_torque)) {}

    /// Zone of the escapement for a swing in direction s at angle theta.
    Zone zone(double theta, int s) const {
        const double u = s * theta + lead;
        // Detent: only the swing in the positive direction unlocks and takes an impulse. On the return swing the
        // roller brushes the passing spring aside over the unlocking angle, centred on the dead point.
        if (type == EscapementType::Detent && s < 0) return std::abs(u) <= esc.unlock / 2 ? kPass : kFree;
        if (u >= esc.lift / 2) return kFree;
        if (u >= -esc.lift / 2) return kImpulse;
        if (u >= -(esc.lift / 2 + esc.unlock)) return kUnlock;
        return kFree;
    }

    double effective_inertia(Zone z) const {
        return z == kImpulse ? inertia + wheel_inertia * esc.ratio * esc.ratio : inertia;
    }

    double acceleration(double theta, double omega, Zone z, int s) const {
        double torque = -stiffness * (theta - rest) - damping * omega - friction * s;
        torque -= gravity_torque * std::sin(theta + gravity_phase);
        if (z == kImpulse) torque += s * esc.impulse_torque;
        if (z == kUnlock) torque -= s * esc.unlock_torque;
        if (z == kPass) torque -= s * esc.passing_torque;
        return torque / effective_inertia(z);
    }
};

struct State {
    double theta, omega;
};

State rk4(const Model& m, State y, double h, Zone z, int s) {
    auto f = [&](State q) { return State{q.omega, m.acceleration(q.theta, q.omega, z, s)}; };
    const State k1 = f(y);
    const State k2 = f({y.theta + 0.5 * h * k1.theta, y.omega + 0.5 * h * k1.omega});
    const State k3 = f({y.theta + 0.5 * h * k2.theta, y.omega + 0.5 * h * k2.omega});
    const State k4 = f({y.theta + h * k3.theta, y.omega + h * k3.omega});
    return {y.theta + h / 6 * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta),
            y.omega + h / 6 * (k1.omega + 2 * k2.omega + 2 * k3.omega + k4.omega)};
}

}  // namespace

EscapementGeometry escapement_geometry(const Calibre& c, double wheel_torque_nm) {
    if (c.escape_teeth < 1) throw Error("escape wheel needs at least one tooth");
    EscapementGeometry g;
    g.lift = c.lift_angle_deg * kDeg;
    g.unlock = c.unlock_angle_deg * kDeg;
    g.drop = c.drop_angle_deg * kDeg;
    g.wheel_step = (c.escapement == EscapementType::Detent ? 2.0 : 1.0) * kPi / c.escape_teeth;
    g.ratio = (g.wheel_step - g.drop) / g.lift;
    const double ideal = wheel_torque_nm * g.ratio;
    g.impulse_torque = c.impulse_efficiency * ideal;
    g.unlock_torque = c.unlock_friction_ratio * ideal;
    g.passing_torque = c.escapement == EscapementType::Detent ? c.passing_spring_ratio * ideal : 0.0;
    return g;
}

double estimate_amplitude_deg(const Calibre& c, double wheel_torque_nm, Position position) {
    const Model m(c, wheel_torque_nm, position);
    const double w0 = std::sqrt(m.stiffness / m.inertia);
    const double a2 = 0.5 * kPi * m.damping * w0;  // viscous loss per beat = a2 * A^2
    const double b = 2.0 * m.friction;             // Coulomb loss per beat = b * A
    const double half = 0.5 * m.esc.lift;
    // Net energy per oscillation against amplitude: the impulses (two for the lever, one for the detent)
    // against two swings of Coulomb and viscous loss and, for the detent, the passing spring. The wheel's
    // pick-up grows with speed and the losses with the swing, so it falls monotonically and bisection
    // finds its zero.
    const double impulses = c.impulses_per_oscillation();
    const double passing_work = m.esc.passing_torque * m.esc.unlock;
    auto net = [&](double amp) {
        const double wh2 = w0 * w0 * std::max(amp * amp - half * half, 0.0);
        const double gain = m.esc.impulse_torque * m.esc.lift - m.esc.unlock_torque * m.esc.unlock -
                            m.wheel_inertia * m.esc.ratio * m.esc.ratio * wh2;  // picked up and dropped
        return impulses * gain - 2.0 * (a2 * amp * amp + b * amp) - passing_work;
    };
    double lo = half, hi = 4.0 * kPi;
    if (net(lo) <= 0) return 0.0;
    if (net(hi) > 0) return hi / kDeg;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (net(mid) > 0 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi) / kDeg;
}

RunSummary simulate_oscillator(const Calibre& c, const RunConditions& cond, const EventSink& sink) {
    const Model m(c, cond.wheel_torque_nm, cond.position);
    const double w0 = std::sqrt(m.stiffness / m.inertia);
    const double h_max = 2.0 * kPi / w0 * cond.max_step_fraction;
    const double threshold = m.esc.lift / 2 + m.esc.unlock - m.lead;

    double amp0 = cond.initial_amplitude_deg * kDeg;
    if (amp0 <= 0) amp0 = estimate_amplitude_deg(c, cond.wheel_torque_nm, cond.position) * kDeg;
    if (amp0 < 1.2 * threshold) amp0 = 1.2 * threshold;  // too weak to run: let it decay and stall

    State y{amp0, 0.0};
    int s = -1;  // swing direction; starts at the top of a swing, falling
    Zone zone = kFree;
    double t = 0.0;
    RunSummary out;

    auto emit = [&](EventType type, const State& st) { sink({t, type, st.theta, st.omega, s}); };
    auto event_between = [&](const State& next, Zone z) {
        return next.omega * s < 0 || m.zone(next.theta, s) != z;
    };

    while (t < cond.duration_s) {
        const double h = std::min(h_max, cond.duration_s - t);
        State next = rk4(m, y, h, zone, s);
        double step = h;
        if (event_between(next, zone)) {
            double lo = 0.0, hi = h;
            for (int i = 0; i < 80 && hi - lo > 1e-15; ++i) {
                const double mid = 0.5 * (lo + hi);
                (event_between(rk4(m, y, mid, zone, s), zone) ? hi : lo) = mid;
            }
            step = hi;
            next = rk4(m, y, step, zone, s);
        }
        y = next;
        t += step;

        if (y.omega * s < 0) {
            // Turning point: report the swing that ended, then reverse.
            emit(EventType::Turn, {y.theta, 0.0});
            if (std::abs(y.theta) < threshold) {
                out.stalled = true;
                break;
            }
            s = -s;
            y.omega = 0.0;
            zone = m.zone(y.theta, s);
            continue;
        }
        const Zone now = m.zone(y.theta, s);
        if (now != zone) {
            if (now == kUnlock) {
                emit(EventType::UnlockStart, y);
            } else if (now == kImpulse) {
                emit(EventType::ImpulseStart, y);
                // Inelastic hit: the stationary escape wheel is picked up by the lever.
                y.omega *= m.inertia / m.effective_inertia(kImpulse);
            } else if (zone == kImpulse) {
                emit(EventType::Drop, y);
            }
            zone = now;
        }
    }
    out.end_time = t;
    out.end_amplitude_deg = std::abs(y.theta) / kDeg;
    return out;
}

Measurement measure(const std::vector<Event>& events, const Calibre& c, double t_from, double t_to) {
    Measurement r;
    double amp_sum = 0.0;
    int turns = 0;
    std::vector<double> beats;
    const Event* open = nullptr;
    for (const Event& e : events) {
        if (e.t < t_from || e.t > t_to) continue;
        switch (e.type) {
            case EventType::Turn:
                amp_sum += std::abs(e.theta);
                ++turns;
                break;
            case EventType::ImpulseStart: open = &e; break;
            case EventType::Drop:
                if (open && open->direction == e.direction) beats.push_back(0.5 * (open->t + e.t));
                open = nullptr;
                break;
            default: break;
        }
    }
    r.beats = static_cast<int>(beats.size());
    if (turns > 0) r.amplitude_deg = amp_sum / turns / kDeg;
    if (beats.size() >= 3) {
        // Least-squares slope of beat time against beat number: unlike (last - first) / (n - 1)
        // it is not biased by the endpoints when tick and tock intervals differ.
        const double n = static_cast<double>(beats.size());
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (std::size_t i = 0; i < beats.size(); ++i) {
            const double x = static_cast<double>(i);
            sx += x;
            sy += beats[i];
            sxx += x * x;
            sxy += x * beats[i];
        }
        const double mean = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        r.period_s = c.impulses_per_oscillation() * mean;
        r.rate_s_per_day = (c.nominal_impulse_interval_s() / mean - 1.0) * 86400.0;
        // Tick and tock exist only when every swing gives an impulse (the lever).
        if (c.escapement == EscapementType::SwissLever) {
            double sum = 0.0;
            for (std::size_t i = 0; i + 2 < beats.size(); ++i) {
                sum += std::abs((beats[i + 1] - beats[i]) - (beats[i + 2] - beats[i + 1])) / 2.0;
            }
            r.beat_error_ms = sum / static_cast<double>(beats.size() - 2) * 1e3;
        }
    }
    return r;
}

SteadyState steady_state(const Calibre& c, double wheel_torque_nm, Position position) {
    constexpr double window = 20.0;
    SteadyState out;
    double settle = 120.0;
    for (int attempt = 0; attempt < 4; ++attempt, settle *= 2) {
        RunConditions cond;
        cond.position = position;
        cond.wheel_torque_nm = wheel_torque_nm;
        cond.duration_s = settle + window;
        EventLog log;
        const RunSummary run = simulate_oscillator(c, cond, log.sink());
        out.settle_time_s = settle;
        if (run.stalled) {
            out.stalled = true;
            out.m = measure(log.events, c, 0.0, run.end_time);
            return out;
        }
        const Measurement last = measure(log.events, c, settle, settle + window);
        const Measurement before = measure(log.events, c, settle - window, settle);
        out.m = last;
        if (std::abs(last.amplitude_deg - before.amplitude_deg) <= 1e-3 * last.amplitude_deg) break;
    }
    return out;
}

}  // namespace ohs::dynamics
