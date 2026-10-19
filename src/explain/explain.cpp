// SPDX-License-Identifier: Apache-2.0
#include "explain/explain.hpp"

#include <cmath>
#include <iomanip>
#include <queue>
#include <set>
#include <sstream>

#include "core/error.hpp"

namespace ohs::explain {

using model::Movement;
using model::Role;

namespace {

std::string num(double v, int precision = 7) {
    std::ostringstream out;
    out << std::setprecision(precision) << v;
    return out.str();
}

std::string_view summary_for(Role role) {
    switch (role) {
        case Role::Plate: return "The main plate is the ground frame: every pivot and bridge references it.";
        case Role::Bridge: return "A bridge holds the upper pivots of the train, positioning the wheels against the plate.";
        case Role::Barrel: return "The barrel holds the mainspring and is the driving root of the going train; as the spring unwinds it turns the first pinion.";
        case Role::Mainspring: return "The mainspring stores the energy that drives the movement.";
        case Role::Arbor: return "An arbor is the axle of a mobile: it carries wheels and pinions that must turn together and ends in pivots.";
        case Role::Wheel: return "A wheel meshes with the pinion of the next mobile; the tooth ratio steps the speed up on the way to the escapement.";
        case Role::Pinion: return "A pinion is a small gear driven by the previous wheel; few leaves give a large speed-up.";
        case Role::EscapeWheel: return "The escape wheel releases one tooth per half oscillation of the balance, so it sets the train's speed and receives the impulse that keeps the balance swinging.";
        case Role::PalletFork: return "The pallet fork locks and releases the escape wheel and passes impulse to the balance through the roller.";
        case Role::Balance: return "The balance is the oscillator; with the hairspring it sets the beat rate (f = 1/(2 pi) sqrt(k/I)).";
        case Role::Hairspring: return "The hairspring supplies the restoring torque of the balance oscillator.";
        case Role::Roller: return "The roller carries the impulse jewel that couples the balance to the pallet fork.";
        case Role::Jewel: return "A jewel is a low-friction bearing for a pivot.";
        case Role::Screw: return "A screw fixes a plate or bridge.";
        case Role::Rotor: return "The rotor (oscillating weight) winds the mainspring of an automatic movement: it swings about the movement's centre as the wrist moves, and the reverser and reduction wheels turn that into winding of the barrel. Its winding is not simulated yet, so it is drawn but does not move.";
        case Role::Unknown: break;
    }
    return "This part has not been classified; set its role with `ohs rig set`.";
}

}  // namespace

PartExplanation explain_part(const Movement& m, const kinematics::TrainAnalysis& train, const std::string& name) {
    const int idx = m.find_component(name);
    if (idx < 0) throw Error("no component named '" + name + "'");
    const model::Component& c = m.components[idx];

    PartExplanation out;
    out.name = name;
    out.role = std::string(to_string(c.role));
    out.summary = std::string(summary_for(c.role));
    auto fact = [&out](std::string text) { out.facts.push_back(std::move(text)); };

    fact("role " + out.role + " (" + c.role_source + ", confidence " + num(c.role_confidence, 2) + ")");
    if (c.features.teeth > 0) fact(std::to_string(c.features.teeth) + " teeth");
    if (c.gear) {
        const model::GearFit& g = *c.gear;
        fact("module " + num(g.module) + " mm (" + g.module_source + "), pitch diameter " +
             num(g.module * c.features.teeth) + " mm");
        fact("addendum " + num(g.addendum, 3) + " m, dedendum " + num(g.dedendum, 3) + " m");
        if (g.profile == "involute") {
            fact("involute flank, pressure angle " + num(g.pressure_angle_deg, 3) + " deg");
        } else if (g.profile == "cycloidal") {
            fact("cycloidal flank, rolling circle " + num(g.rolling_radius, 2) + " m (a wheel meshing with it needs the same one)");
        } else {
            fact("flank is neither a recognisable involute nor a cycloid (unfitted)");
        }
    }

    const int mob = m.mobile_of(idx);
    if (mob >= 0) {
        std::string peers;
        for (int other : m.mobiles[mob].components) {
            if (other != idx) peers += (peers.empty() ? "" : ", ") + m.components[other].features.name;
        }
        fact("belongs to mobile " + m.mobiles[mob].name + (peers.empty() ? "" : ", turning with " + peers));
        const kinematics::MobileMotion& mm = train.mobiles[mob];
        if (mm.reached) {
            fact("turns " + num(std::abs(mm.ratio)) + " x the root speed, " +
                 (mm.ratio < 0 ? "opposite" : "same") + " direction as the root");
            if (mm.revs_per_hour > 0) {
                fact(num(mm.revs_per_hour) + " rev/h = " + num(mm.revs_per_hour / 60.0) + " rpm");
            }
        }
    } else if (!model::is_rotating_role(c.role)) {
        fact("fixed relative to the plate");
    }

    for (const model::MeshJoint& j : m.meshes) {
        if (j.a != idx && j.b != idx) continue;
        const int other = j.a == idx ? j.b : j.a;
        const auto& partner = m.components[other];
        fact("meshes with " + partner.features.name + " (" + std::to_string(partner.features.teeth) + " teeth), " +
             std::to_string(c.features.teeth) + ":" + std::to_string(partner.features.teeth) + " gives a ratio of " +
             num(static_cast<double>(c.features.teeth) / partner.features.teeth) + ", centre distance " +
             num(j.centre_distance) + " mm");
    }
    if (c.role == Role::EscapeWheel && train.beats_per_root_rev > 0 && train.root_period_hours > 0) {
        fact("passes " + std::to_string(c.features.teeth) + " teeth x 2 beats per tooth = " +
             std::to_string(2 * c.features.teeth) + " beats per turn");
    }
    return out;
}

Walkthrough walk_train(const Movement& m, const kinematics::TrainAnalysis& train) {
    Walkthrough w;
    w.root = m.mobiles.at(train.root).name;

    std::set<int> seen{train.root};
    std::queue<int> pending;
    pending.push(train.root);
    while (!pending.empty()) {
        const int cur = pending.front();
        pending.pop();
        for (const model::MeshJoint& j : m.meshes) {
            const int ma = m.mobile_of(j.a);
            const int mb = m.mobile_of(j.b);
            int driver_idx, driven_idx;
            if (ma == cur && !seen.count(mb)) {
                driver_idx = j.a;
                driven_idx = j.b;
            } else if (mb == cur && !seen.count(ma)) {
                driver_idx = j.b;
                driven_idx = j.a;
            } else {
                continue;
            }
            const int next = m.mobile_of(driven_idx);
            seen.insert(next);
            pending.push(next);
            const auto& driver = m.components[driver_idx];
            const auto& driven = m.components[driven_idx];
            TrainStep step;
            step.driver = driver.features.name;
            step.driven = driven.features.name;
            step.driver_teeth = driver.features.teeth;
            step.driven_teeth = driven.features.teeth;
            step.driven_mobile = m.mobiles[next].name;
            step.step_ratio = static_cast<double>(driver.features.teeth) / driven.features.teeth;
            step.cumulative_ratio = train.mobiles[next].ratio;
            step.revs_per_hour = train.mobiles[next].revs_per_hour;
            w.steps.push_back(std::move(step));
        }
    }

    if (train.escape_mobile >= 0 && train.beats_per_root_rev > 0) {
        w.conclusion.push_back("Escape wheel: " + std::to_string(train.escape_teeth) + " teeth x 2 beats per tooth = " +
                               std::to_string(2 * train.escape_teeth) + " beats per turn, and it turns " +
                               num(std::abs(train.mobiles[train.escape_mobile].ratio)) + " x per root turn: " +
                               num(train.beats_per_root_rev) + " beats per root turn.");
        if (train.root_period_hours > 0) {
            w.conclusion.push_back("With the movement's beat rate the root turns once per " +
                                   num(train.root_period_hours) + " h, and the escape wheel makes " +
                                   num(train.beats_per_root_rev / train.root_period_hours) + " beats/h.");
        }
    }
    return w;
}

std::string to_text(const PartExplanation& e) {
    std::ostringstream out;
    out << e.name << " (" << e.role << ")\n" << e.summary << "\n";
    for (const std::string& f : e.facts) out << "  - " << f << "\n";
    return out.str();
}

std::string to_text(const Walkthrough& w) {
    std::ostringstream out;
    out << "Going train, starting from mobile " << w.root << ":\n";
    int n = 1;
    for (const TrainStep& s : w.steps) {
        out << n++ << ". " << s.driver << " (" << s.driver_teeth << " teeth) drives " << s.driven << " ("
            << s.driven_teeth << " teeth): " << s.driver_teeth << ":" << s.driven_teeth << " = x"
            << num(s.step_ratio) << ", direction reverses. Mobile " << s.driven_mobile << " now turns x"
            << num(std::abs(s.cumulative_ratio)) << " the root speed";
        if (s.revs_per_hour > 0) out << " = " << num(s.revs_per_hour) << " rev/h (" << num(s.revs_per_hour / 60.0) << " rpm)";
        out << ".\n";
    }
    for (const std::string& c : w.conclusion) out << c << "\n";
    return out.str();
}

}  // namespace ohs::explain
