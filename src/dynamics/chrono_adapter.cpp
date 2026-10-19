// SPDX-License-Identifier: Apache-2.0
#include "dynamics/chrono_adapter.hpp"

#include "core/error.hpp"

#ifdef OHS_WITH_CHRONO

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numbers>

#include "chrono/ChVersion.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsAppliedTorque.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChSystemSMC.h"

namespace ohs::dynamics {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

enum Zone { kFree = 0, kUnlock = 1, kImpulse = 2 };

double crown_phase(Position p) {
    switch (p) {
        case Position::CrownRight: return kPi / 2;
        case Position::CrownDown: return kPi;
        case Position::CrownLeft: return 3 * kPi / 2;
        default: return 0.0;
    }
}

/// A torque between two shafts given by a callable, applied to the first shaft and reacted on the second.
class FunctionTorque final : public chrono::ChShaftsTorque {
  public:
    explicit FunctionTorque(std::function<double()> f) : f_(std::move(f)) {}
    FunctionTorque* Clone() const override { return new FunctionTorque(*this); }

  private:
    double ComputeTorque() override { return f_(); }
    std::function<double()> f_;
};

/// The escapement geometry and contact law, with the state it reads from the two shafts.
struct Escapement {
    EscapementGeometry esc;
    double lead{}, contact_k{}, contact_c{}, efficiency{}, friction{}, gravity_torque{}, gravity_phase{}, rest{};
    const chrono::ChShaft* balance{};
    const chrono::ChShaft* wheel{};
    int s{-1};  ///< direction of the current swing

    /// The balance shaft is measured from the hairspring's rest position.
    double theta() const { return balance->GetPos() + rest; }
    double position_in_swing(double th) const { return s * th + lead; }

    Zone zone(double th) const {
        const double u = position_in_swing(th);
        if (u >= esc.lift / 2) return kFree;
        if (u >= -esc.lift / 2) return kImpulse;
        if (u >= -(esc.lift / 2 + esc.unlock)) return kUnlock;
        return kFree;
    }

    /// Position of the pallet in wheel angle beyond the current lock, and its slope per balance angle.
    void pallet(double th, double& psi_c, double& slope) const {
        const double u = position_in_swing(th);
        const double travel = esc.wheel_step - esc.drop;
        if (u >= esc.lift / 2) {
            psi_c = esc.wheel_step;
            slope = 0.0;
        } else if (u <= -esc.lift / 2) {
            psi_c = 0.0;
            slope = 0.0;
        } else {
            psi_c = travel * (u + esc.lift / 2) / esc.lift;
            slope = s * travel / esc.lift;
        }
    }

    double contact_force(double& slope) const {
        double psi_c;
        pallet(theta(), psi_c, slope);
        const double gap = wheel->GetPos() - psi_c;
        if (gap <= 0.0) return 0.0;
        return std::max(contact_k * gap + contact_c * (wheel->GetPosDt() - slope * balance->GetPosDt()), 0.0);
    }

    /// Everything on the balance except the hairspring's spring and the air damping.
    double balance_torque() const {
        double slope;
        const double force = contact_force(slope);
        const double th = theta();
        double torque = -friction * s - gravity_torque * std::sin(th + gravity_phase) + efficiency * force * slope;
        if (zone(th) == kUnlock) torque -= s * esc.unlock_torque;
        return torque;
    }

    double wheel_reaction() const {
        double slope;
        return -contact_force(slope);
    }
};

}  // namespace

bool chrono_available() { return true; }
const char* chrono_version() { return CHRONO_VERSION; }

RunSummary simulate_chrono(const Calibre& c, const DetailedConditions& cond, const EventSink& sink) {
    if (c.escapement != EscapementType::SwissLever) throw Error("the Chrono model covers the Swiss lever only; use the reduced model for the detent escapement");
    Escapement e;
    e.esc = escapement_geometry(c, cond.wheel_torque_nm);
    e.lead = c.impulse_lead_deg * kDeg;
    e.rest = c.beat_offset_deg * kDeg;
    e.efficiency = c.impulse_efficiency;
    e.friction = c.friction_torque(cond.position);
    e.gravity_torque = is_vertical(cond.position) ? c.unpoise_torque() : 0.0;
    e.gravity_phase = c.unpoise_angle_deg * kDeg + crown_phase(cond.position);
    const double wheel_inertia = c.escape_wheel_inertia();
    const double wc = 2.0 * kPi * c.contact_frequency_hz;
    e.contact_k = wheel_inertia * wc * wc;
    e.contact_c = 2.0 * c.contact_damping_ratio * std::sqrt(e.contact_k * wheel_inertia);

    const double h = cond.step_s > 0 ? cond.step_s : 0.4 / wc;  // Chrono costs far more per step than the built-in loop
    const double threshold = e.esc.lift / 2 + e.esc.unlock - e.lead;

    double amp0 = cond.initial_amplitude_deg * kDeg;
    if (amp0 <= 0) amp0 = estimate_amplitude_deg(c, cond.wheel_torque_nm, cond.position) * kDeg;
    if (amp0 < 1.2 * threshold) amp0 = 1.2 * threshold;

    chrono::ChSystemSMC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, 0));
    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto ground = std::make_shared<chrono::ChShaft>();
    ground->SetFixed(true);
    auto spring_ref = std::make_shared<chrono::ChShaft>();  // the hairspring's rest position
    spring_ref->SetFixed(true);
    auto balance = std::make_shared<chrono::ChShaft>();
    balance->SetInertia(c.inertia());
    balance->SetPos(amp0 - e.rest);
    balance->SetPosDt(0.0);
    auto wheel = std::make_shared<chrono::ChShaft>();
    wheel->SetInertia(wheel_inertia);
    wheel->SetPos(cond.wheel_torque_nm / e.contact_k);
    wheel->SetPosDt(0.0);
    for (const auto& shaft : {ground, spring_ref, balance, wheel}) sys.AddShaft(shaft);
    e.balance = balance.get();
    e.wheel = wheel.get();

    auto hairspring = std::make_shared<chrono::ChShaftsTorsionSpring>();
    hairspring->Initialize(balance, spring_ref);
    hairspring->SetTorsionalStiffness(c.hairspring_stiffness());
    hairspring->SetTorsionalDamping(c.air_damping_nms);
    sys.Add(hairspring);

    auto escapement_on_balance = std::make_shared<FunctionTorque>([&e] { return e.balance_torque(); });
    escapement_on_balance->Initialize(balance, ground);
    sys.Add(escapement_on_balance);

    auto drive = std::make_shared<chrono::ChShaftsAppliedTorque>();
    drive->Initialize(wheel, ground);
    drive->SetTorque(cond.wheel_torque_nm);
    sys.Add(drive);

    auto contact_on_wheel = std::make_shared<FunctionTorque>([&e] { return e.wheel_reaction(); });
    contact_on_wheel->Initialize(wheel, ground);
    sys.Add(contact_on_wheel);

    Zone zone = kFree;
    double t = 0.0;
    RunSummary out;
    const long steps = static_cast<long>(cond.duration_s / h);
    double th = e.theta(), om = 0.0;

    for (long i = 0; i < steps; ++i) {
        const double th0 = th, om0 = om;
        sys.DoStepDynamics(h);
        th = e.theta();
        om = balance->GetPosDt();
        const double t_next = t + h;
        if (om * e.s < 0) {
            // Turning point: interpolate its instant, report it and reverse; the wheel has been
            // caught by the next pallet, so its reference moves on by one step.
            const double frac = om0 / (om0 - om);
            sink({t + frac * h, EventType::Turn, th0 + frac * (th - th0), 0.0, e.s});
            if (std::abs(th) < threshold) {
                out.stalled = true;
                t = t_next;
                break;
            }
            wheel->SetPos(wheel->GetPos() - e.esc.wheel_step);
            e.s = -e.s;
            zone = e.zone(th);
            t = t_next;
            continue;
        }
        const Zone now = e.zone(th);
        if (now != zone) {
            const double boundary = now == kUnlock ? -(e.esc.lift / 2 + e.esc.unlock)
                                                   : (now == kImpulse ? -e.esc.lift / 2 : e.esc.lift / 2);
            const double u0 = e.position_in_swing(th0), u1 = e.position_in_swing(th);
            const double frac = u1 != u0 ? std::clamp((boundary - u0) / (u1 - u0), 0.0, 1.0) : 0.5;
            const double t_ev = t + frac * h;
            if (now == kUnlock) {
                sink({t_ev, EventType::UnlockStart, th, om, e.s});
            } else if (now == kImpulse) {
                sink({t_ev, EventType::ImpulseStart, th, om, e.s});
            } else if (zone == kImpulse) {
                sink({t_ev, EventType::Drop, th, om, e.s});
            }
            zone = now;
        }
        t = t_next;
    }
    out.end_time = t;
    out.end_amplitude_deg = std::abs(th) / kDeg;
    return out;
}

}  // namespace ohs::dynamics

#else  // !OHS_WITH_CHRONO

namespace ohs::dynamics {

bool chrono_available() { return false; }
const char* chrono_version() { return ""; }

RunSummary simulate_chrono(const Calibre&, const DetailedConditions&, const EventSink&) {
    throw Error("this build of OHS does not include Project Chrono (configure with -DOHS_WITH_CHRONO=ON)");
}

}  // namespace ohs::dynamics

#endif
