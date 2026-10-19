// SPDX-License-Identifier: Apache-2.0
#include "dynamics/detailed.hpp"

#include "core/error.hpp"
#include "dynamics/chrono_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace ohs::dynamics {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

enum Zone { kFree = 0, kUnlock = 1, kImpulse = 2 };

double crown_phase(Position p) {
    switch (p) {
        case Position::CrownUp: return 0.0;
        case Position::CrownRight: return kPi / 2;
        case Position::CrownDown: return kPi;
        case Position::CrownLeft: return 3 * kPi / 2;
        default: return 0.0;
    }
}

struct Bodies {
    double theta, omega;  // balance
    double psi, nu;       // escape wheel angle beyond its current lock, and speed
};

struct World {
    double inertia, wheel_inertia, stiffness, rest, damping, friction, gravity_torque, gravity_phase, lead;
    double wheel_torque, contact_k, contact_c, efficiency;
    EscapementGeometry esc;

    World(const Calibre& c, double wheel_torque_nm, Position p)
        : inertia(c.inertia()),
          wheel_inertia(c.escape_wheel_inertia()),
          stiffness(c.hairspring_stiffness()),
          rest(c.beat_offset_deg * kDeg),
          damping(c.air_damping_nms),
          friction(c.friction_torque(p)),
          gravity_torque(is_vertical(p) ? c.unpoise_torque() : 0.0),
          gravity_phase(c.unpoise_angle_deg * kDeg + crown_phase(p)),
          lead(c.impulse_lead_deg * kDeg),
          wheel_torque(wheel_torque_nm),
          efficiency(c.impulse_efficiency),
          esc(escapement_geometry(c, wheel_torque_nm)) {
        const double wc = 2.0 * kPi * c.contact_frequency_hz;
        contact_k = wheel_inertia * wc * wc;
        contact_c = 2.0 * c.contact_damping_ratio * std::sqrt(contact_k * wheel_inertia);
    }

    double position_in_swing(double theta, int s) const { return s * theta + lead; }

    Zone zone(double theta, int s) const {
        const double u = position_in_swing(theta, s);
        if (u >= esc.lift / 2) return kFree;
        if (u >= -esc.lift / 2) return kImpulse;
        if (u >= -(esc.lift / 2 + esc.unlock)) return kUnlock;
        return kFree;
    }

    /// Position of the pallet in wheel angle beyond the current lock, and its slope per balance angle.
    void pallet(double theta, int s, double& psi_c, double& slope) const {
        const double u = position_in_swing(theta, s);
        const double travel = esc.wheel_step - esc.drop;
        if (u >= esc.lift / 2) {
            psi_c = esc.wheel_step;  // pallet has gone: the wheel runs on to the next lock
            slope = 0.0;
        } else if (u <= -esc.lift / 2) {
            psi_c = 0.0;
            slope = 0.0;
        } else {
            psi_c = travel * (u + esc.lift / 2) / esc.lift;
            slope = s * travel / esc.lift;
        }
    }

    std::array<double, 4> derivative(const Bodies& b, int s) const {
        double psi_c, slope;
        pallet(b.theta, s, psi_c, slope);
        const double gap = b.psi - psi_c;
        double force = 0.0;
        if (gap > 0.0) force = std::max(contact_k * gap + contact_c * (b.nu - slope * b.omega), 0.0);

        const Zone z = zone(b.theta, s);
        double torque = -stiffness * (b.theta - rest) - damping * b.omega - friction * s;
        torque -= gravity_torque * std::sin(b.theta + gravity_phase);
        torque += efficiency * force * slope;
        if (z == kUnlock) torque -= s * esc.unlock_torque;
        return {b.omega, torque / inertia, b.nu, (wheel_torque - force) / wheel_inertia};
    }
};

Bodies advance(const World& w, const Bodies& y, double h, int s) {
    auto add = [](const Bodies& b, const std::array<double, 4>& d, double k) {
        return Bodies{b.theta + k * d[0], b.omega + k * d[1], b.psi + k * d[2], b.nu + k * d[3]};
    };
    const auto k1 = w.derivative(y, s);
    const auto k2 = w.derivative(add(y, k1, 0.5 * h), s);
    const auto k3 = w.derivative(add(y, k2, 0.5 * h), s);
    const auto k4 = w.derivative(add(y, k3, h), s);
    Bodies out = y;
    out.theta += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    out.omega += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    out.psi += h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    out.nu += h / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
    return out;
}

}  // namespace

RunSummary simulate_detailed(const Calibre& c, const DetailedConditions& cond, const EventSink& sink) {
    if (c.escapement != EscapementType::SwissLever) {
        throw Error("the detailed (contact) model covers the Swiss lever only; use the reduced model for the detent escapement");
    }
    const World w(c, cond.wheel_torque_nm, cond.position);
    const double h = cond.step_s > 0 ? cond.step_s : 0.1 / (2.0 * kPi * c.contact_frequency_hz);
    const double threshold = w.esc.lift / 2 + w.esc.unlock - w.lead;

    double amp0 = cond.initial_amplitude_deg * kDeg;
    if (amp0 <= 0) amp0 = estimate_amplitude_deg(c, cond.wheel_torque_nm, cond.position) * kDeg;
    if (amp0 < 1.2 * threshold) amp0 = 1.2 * threshold;

    int s = -1;
    Bodies y{amp0, 0.0, cond.wheel_torque_nm / w.contact_k, 0.0};
    Zone zone = kFree;
    double t = 0.0;
    RunSummary out;
    const long steps = static_cast<long>(cond.duration_s / h);

    for (long i = 0; i < steps; ++i) {
        const Bodies next = advance(w, y, h, s);
        const double t_next = t + h;
        if (next.omega * s < 0) {
            // Turning point: interpolate its instant, report it and reverse; the wheel has been
            // caught by the next pallet, so its reference moves on by one step.
            const double frac = y.omega / (y.omega - next.omega);
            const double t_turn = t + frac * h;
            sink({t_turn, EventType::Turn, y.theta + frac * (next.theta - y.theta), 0.0, s});
            if (std::abs(next.theta) < threshold) {
                out.stalled = true;
                y = next;
                t = t_next;
                break;
            }
            y = next;
            y.psi -= w.esc.wheel_step;
            s = -s;
            zone = w.zone(y.theta, s);
            t = t_next;
            continue;
        }
        const Zone now = w.zone(next.theta, s);
        if (now != zone) {
            const double boundary = now == kUnlock ? -(w.esc.lift / 2 + w.esc.unlock)
                                                   : (now == kImpulse ? -w.esc.lift / 2 : w.esc.lift / 2);
            const double u0 = w.position_in_swing(y.theta, s), u1 = w.position_in_swing(next.theta, s);
            const double frac = u1 != u0 ? std::clamp((boundary - u0) / (u1 - u0), 0.0, 1.0) : 0.5;
            const double t_ev = t + frac * h;
            if (now == kUnlock) {
                sink({t_ev, EventType::UnlockStart, next.theta, next.omega, s});
            } else if (now == kImpulse) {
                sink({t_ev, EventType::ImpulseStart, next.theta, next.omega, s});
            } else if (zone == kImpulse) {
                sink({t_ev, EventType::Drop, next.theta, next.omega, s});
            }
            zone = now;
        }
        y = next;
        t = t_next;
    }
    out.end_time = t;
    out.end_amplitude_deg = std::abs(y.theta) / kDeg;
    return out;
}

const char* to_string(Engine e) { return e == Engine::Chrono ? "chrono" : "builtin"; }

Engine engine_from_string(const std::string& text) {
    if (text == "builtin") return Engine::Builtin;
    if (text == "chrono") return Engine::Chrono;
    throw Error("unknown engine '" + text + "' (builtin or chrono)");
}

Agreement compare_levels(const Calibre& c, double wheel_torque_nm, Position position, double warmup_s, double window_s,
                         Engine engine) {
    Agreement a;
    const SteadyState ss = steady_state(c, wheel_torque_nm, position);
    a.level2 = ss.m;

    DetailedConditions cond;
    cond.position = position;
    cond.wheel_torque_nm = wheel_torque_nm;
    cond.initial_amplitude_deg = ss.m.amplitude_deg;
    cond.duration_s = warmup_s + window_s;
    EventLog log;
    if (engine == Engine::Chrono) {
        simulate_chrono(c, cond, log.sink());
    } else {
        simulate_detailed(c, cond, log.sink());
    }
    a.level3 = measure(log.events, c, warmup_s, warmup_s + window_s);
    if (a.level2.amplitude_deg > 0) {
        a.amplitude_diff_percent = 100.0 * (a.level3.amplitude_deg - a.level2.amplitude_deg) / a.level2.amplitude_deg;
    }
    a.rate_diff_s_per_day = a.level3.rate_s_per_day - a.level2.rate_s_per_day;
    a.beat_error_diff_ms = a.level3.beat_error_ms - a.level2.beat_error_ms;
    return a;
}

}  // namespace ohs::dynamics
