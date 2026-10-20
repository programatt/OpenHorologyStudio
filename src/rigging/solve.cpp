// SPDX-License-Identifier: Apache-2.0
// Closing the going train: what drives the gear nothing drives, and which wheels turn the hands.
#include <algorithm>
#include <cmath>
#include <queue>

#include "core/error.hpp"
#include "rigging/rigging.hpp"

namespace ohs::rigging {

using model::Component;
using model::Movement;
using model::Role;

namespace {

constexpr double kMaxResidual = 0.12;  // how far a pair's module may sit from its gears' own, as a fraction
constexpr int kMinEstimatedTeeth = 12;

double module_of(const model::PartFeatures& f) { return f.teeth > 0 ? 2.0 * f.outer_radius / (f.teeth + 2.0) : 0.0; }

// Turns per hour of every mobile that the meshes of the train join to the escape wheel (-1 for the others). A mesh
// turns its two gears so that teeth_a * rate_a = teeth_b * rate_b.
std::vector<double> rates_from_escape(const Movement& m, int escape_mobile, double escape_rate) {
    std::vector<double> rate(m.mobiles.size(), -1.0);
    if (escape_mobile < 0) return rate;
    rate[static_cast<std::size_t>(escape_mobile)] = escape_rate;
    std::queue<int> queue;
    queue.push(escape_mobile);
    while (!queue.empty()) {
        const int at = queue.front();
        queue.pop();
        for (const model::MeshJoint& j : m.meshes) {
            if (!m.in_train(j)) continue;
            const int ma = m.mobile_of(j.a), mb = m.mobile_of(j.b);
            const double ta = m.components[static_cast<std::size_t>(j.a)].features.teeth;
            const double tb = m.components[static_cast<std::size_t>(j.b)].features.teeth;
            if (ma < 0 || mb < 0 || ma == mb || ta <= 0 || tb <= 0) continue;
            if (ma == at && rate[static_cast<std::size_t>(mb)] < 0) {
                rate[static_cast<std::size_t>(mb)] = rate[static_cast<std::size_t>(ma)] * ta / tb;
                queue.push(mb);
            } else if (mb == at && rate[static_cast<std::size_t>(ma)] < 0) {
                rate[static_cast<std::size_t>(ma)] = rate[static_cast<std::size_t>(mb)] * tb / ta;
                queue.push(ma);
            }
        }
    }
    return rate;
}

}  // namespace

TrainSolution solve_train(const Movement& m, double beat_rate_vph, const RigOptions& options) {
    TrainSolution out;
    out.beat_rate_vph = beat_rate_vph;
    const TrainChain chain = train_chain(m);
    out.connected = chain.connected;
    if (chain.escape < 0) {
        out.notes.push_back("no escape wheel found: give one the role escape_wheel first");
        return out;
    }
    if (beat_rate_vph <= 0.0) {
        out.notes.push_back("no beat rate: the rates of the wheels cannot be worked out (pass one)");
        return out;
    }
    int escape_teeth = 0;
    for (int idx : m.mobiles[static_cast<std::size_t>(chain.escape)].components) {
        if (m.components[static_cast<std::size_t>(idx)].role == Role::EscapeWheel) escape_teeth = m.components[static_cast<std::size_t>(idx)].features.teeth;
    }
    if (escape_teeth <= 0) {
        out.notes.push_back("the escape wheel has no tooth count, so the rates cannot be worked out");
        return out;
    }
    const std::vector<double> rate = rates_from_escape(m, chain.escape, beat_rate_vph / (2.0 * escape_teeth));

    // The hands: the wheels whose rate is one turn in a minute, an hour and twelve hours, and so on.
    const auto near = [](double a, double b) { return b > 0 && std::abs(a - b) <= 0.02 * b; };
    for (std::size_t i = 0; i < rate.size(); ++i) {
        if (rate[i] < 0) continue;
        const char* hand = near(rate[i], 60.0) ? "seconds" : near(rate[i], 1.0) ? "minute" : near(rate[i], 1.0 / 12.0) ? "hour" : nullptr;
        if (!hand) continue;
        out.hands.push_back({hand, static_cast<int>(i), describe_mobile(m, static_cast<int>(i)), rate[i]});
    }
    const auto has = [&out](const char* word) {
        return std::any_of(out.hands.begin(), out.hands.end(), [word](const HandWheel& h) { return h.hand.find(word) != std::string::npos; });
    };
    if (!has("minute")) out.notes.push_back("no wheel of the train turns once an hour: the minute wheel is not joined to it, or a tooth count is wrong");
    if (!has("seconds")) out.notes.push_back("no wheel of the train turns once a minute (the seconds wheel)");
    if (!has("hour")) out.notes.push_back("no wheel turns once in twelve hours: the motion work (cannon pinion, minute wheel, hour wheel) is not rigged or its teeth are not counted");

    if (chain.connected) return out;

    // The gear nothing drives: a toothed part of the mobile at the loose end of the escape side that meshes with nothing.
    const int edge = chain.from_escape.empty() ? chain.escape : chain.from_escape.front();
    std::vector<int> inputs;
    for (int idx : m.mobiles[static_cast<std::size_t>(edge)].components) {
        const Component& c = m.components[static_cast<std::size_t>(idx)];
        if (c.role == Role::EscapeWheel || !model::is_toothed_role(c.role) || c.features.teeth < options.min_teeth) continue;
        const bool meshed = std::any_of(m.meshes.begin(), m.meshes.end(), [&](const model::MeshJoint& j) {
            return (j.a == idx || j.b == idx) && m.in_train(j);
        });
        if (!meshed) inputs.push_back(idx);
    }
    if (inputs.empty()) {
        out.notes.push_back("the loose end of the train (" + describe_mobile(m, edge) + ") has no toothed part that needs a driver");
        return out;
    }
    out.input = inputs.front();
    const double input_rate = rate[static_cast<std::size_t>(edge)];

    for (int input : inputs) {
        const model::PartFeatures& fi = m.components[static_cast<std::size_t>(input)].features;
        const double mi = module_of(fi);
        if (mi <= 0.0) continue;
        for (std::size_t d = 0; d < m.components.size(); ++d) {
            const Component& cd = m.components[d];
            const int mobile = m.mobile_of(static_cast<int>(d));
            if (mobile < 0 || mobile == edge || rate[static_cast<std::size_t>(mobile)] >= 0 || model::is_winding_part(cd)) continue;
            const model::PartFeatures& fd = cd.features;
            if (std::min(fi.z_max, fd.z_max) - std::max(fi.z_min, fd.z_min) <= 0.0) continue;  // not in the same layer
            const double centre = std::hypot(fi.center.x - fd.center.x, fi.center.y - fd.center.y);
            if (centre <= 0.0) continue;
            SolveCandidate c;
            c.driver = static_cast<int>(d);
            c.input = input;
            c.centre = centre;
            if (fd.teeth >= options.min_teeth) {
                c.teeth = fd.teeth;
                c.module = 2.0 * centre / (fd.teeth + fi.teeth);
                c.residual = std::max(std::abs(c.module - mi) / mi, std::abs(c.module - module_of(fd)) / module_of(fd));
            } else {
                if (fd.outer_radius < 3.0 * mi) continue;  // an arbor or a pinion is no driver of a wheel's size
                c.estimated = true;
                c.teeth = static_cast<int>(std::lround(2.0 * centre / mi - fi.teeth));
                if (c.teeth < kMinEstimatedTeeth) continue;
                c.module = 2.0 * centre / (c.teeth + fi.teeth);
                const double tip = c.module * (c.teeth + 2.0) / 2.0;  // where the tips of that many teeth would be
                c.residual = std::max(std::abs(c.module - mi) / mi, std::abs(tip - fd.outer_radius) / fd.outer_radius);
            }
            if (c.residual > kMaxResidual) continue;
            c.driver_rate = input_rate * fi.teeth / c.teeth;
            out.candidates.push_back(c);
        }
    }
    // The barrel first when it fits, then by how well the gears fit.
    std::stable_sort(out.candidates.begin(), out.candidates.end(), [&m](const SolveCandidate& a, const SolveCandidate& b) {
        const bool ba = m.components[static_cast<std::size_t>(a.driver)].role == Role::Barrel;
        const bool bb = m.components[static_cast<std::size_t>(b.driver)].role == Role::Barrel;
        if (ba != bb) return ba;
        return a.residual < b.residual;
    });
    if (out.candidates.size() > 8) out.candidates.resize(8);
    if (out.candidates.empty()) {
        out.notes.push_back("nothing fits as the driver of " + m.components[static_cast<std::size_t>(out.input)].display_name() +
                            ": declare the mesh by hand (the part may be missing or its teeth not found)");
    }
    return out;
}

void apply_solution(Movement& m, const SolveCandidate& candidate, const RigOptions& options) {
    if (candidate.driver < 0 || candidate.input < 0) throw Error("solve: no candidate to apply");
    Component& driver = m.components[static_cast<std::size_t>(candidate.driver)];
    if (candidate.estimated) {
        driver.features.teeth = candidate.teeth;
        driver.features.tooth_confidence = 0.5;
    }
    add_manual_mesh(m, driver.features.name, m.components[static_cast<std::size_t>(candidate.input)].features.name, options);
}

}  // namespace ohs::rigging
