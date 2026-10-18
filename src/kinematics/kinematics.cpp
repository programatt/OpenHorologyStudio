// SPDX-License-Identifier: Apache-2.0
#include "kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>

#include "core/error.hpp"

namespace ohs::kinematics {

using model::Movement;
using model::Role;

const MobileMotion* TrainAnalysis::find(const std::string& mobile_name) const {
    for (const MobileMotion& m : mobiles) {
        if (m.name == mobile_name) return &m;
    }
    return nullptr;
}

TrainAnalysis analyze_train(const Movement& m, double beat_rate_vph, int root) {
    TrainAnalysis out;
    const int nm = static_cast<int>(m.mobiles.size());
    out.mobiles.resize(nm);
    for (int i = 0; i < nm; ++i) out.mobiles[i].name = m.mobiles[i].name;

    if (root < 0) {
        for (int i = 0; i < nm && root < 0; ++i) {
            for (int c : m.mobiles[i].components) {
                if (m.components[c].role == Role::Barrel) root = i;
            }
        }
    }
    // Adjacency over gear meshes: omega_b = -omega_a * teeth_a / teeth_b.
    struct Edge { int to; double factor; };
    std::vector<std::vector<Edge>> adj(nm);
    for (const model::MeshJoint& j : m.meshes) {
        const int ma = m.mobile_of(j.a);
        const int mb = m.mobile_of(j.b);
        const double za = m.components[j.a].features.teeth;
        const double zb = m.components[j.b].features.teeth;
        if (ma < 0 || mb < 0 || za <= 0 || zb <= 0) continue;
        adj[ma].push_back({mb, -za / zb});
        adj[mb].push_back({ma, -zb / za});
    }

    // No barrel (a smooth barrel drum has no teeth to find, so it is often missing from CAD rigs):
    // start the train at the far end of the gear chain that leads to the escape wheel.
    if (root < 0) {
        int escape = -1;
        for (int i = 0; i < nm && escape < 0; ++i) {
            for (int c : m.mobiles[i].components) {
                if (m.components[c].role == Role::EscapeWheel) escape = i;
            }
        }
        if (escape >= 0) {
            std::vector<int> hops(nm, -1);
            std::queue<int> q;
            hops[escape] = 0;
            q.push(escape);
            int far = escape;
            while (!q.empty()) {
                const int cur = q.front();
                q.pop();
                if (hops[cur] > hops[far]) far = cur;
                for (const Edge& e : adj[cur]) {
                    if (hops[e.to] < 0) {
                        hops[e.to] = hops[cur] + 1;
                        q.push(e.to);
                    }
                }
            }
            if (far != escape) {
                root = far;
                out.warnings.push_back("no barrel in the rig: the train starts at " + m.mobiles[root].name + ", the far end of the gear chain from the escape wheel (set a barrel or pass a root to change this)");
            }
        }
    }
    if (root < 0 || root >= nm) throw Error("no root mobile: the rig has no barrel and none was given");
    out.root = root;

    out.mobiles[root].reached = true;
    out.mobiles[root].ratio = 1.0;
    std::queue<int> pending;
    pending.push(root);
    while (!pending.empty()) {
        const int cur = pending.front();
        pending.pop();
        for (const Edge& e : adj[cur]) {
            const double ratio = out.mobiles[cur].ratio * e.factor;
            MobileMotion& next = out.mobiles[e.to];
            if (!next.reached) {
                next.reached = true;
                next.ratio = ratio;
                pending.push(e.to);
            } else if (std::abs(next.ratio - ratio) > 1e-9 * std::max(1.0, std::abs(ratio))) {
                out.warnings.push_back("inconsistent train: mobile " + next.name +
                                       " is reached with two different ratios (a gear loop)");
            }
        }
    }
    for (const MobileMotion& mm : out.mobiles) {
        if (!mm.reached) out.warnings.push_back("mobile " + mm.name + " is not connected to the train");
    }

    for (int i = 0; i < nm; ++i) {
        for (int c : m.mobiles[i].components) {
            if (m.components[c].role == Role::EscapeWheel) {
                out.escape_mobile = i;
                out.escape_teeth = m.components[c].features.teeth;
            }
        }
    }
    if (out.escape_mobile >= 0 && out.mobiles[out.escape_mobile].reached && out.escape_teeth > 0) {
        out.beats_per_root_rev = 2.0 * out.escape_teeth * std::abs(out.mobiles[out.escape_mobile].ratio);
        if (beat_rate_vph > 0) {
            out.root_period_hours = out.beats_per_root_rev / beat_rate_vph;
            for (MobileMotion& mm : out.mobiles) {
                if (mm.reached) mm.revs_per_hour = std::abs(mm.ratio) / out.root_period_hours;
            }
        }
    } else {
        out.warnings.push_back("the escape wheel is missing or not connected to the train");
    }
    return out;
}

std::vector<double> mobile_angles(const TrainAnalysis& train, double seconds, const PoseOptions& options) {
    if (train.root_period_hours <= 0.0 || train.beats_per_root_rev <= 0.0) {
        throw Error("poses need a beat rate and an escape wheel connected to the train");
    }
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double root_revs;
    if (options.stepped) {
        const double beats = seconds / 3600.0 * train.beats_per_root_rev / train.root_period_hours;
        const double whole = std::floor(beats);
        const double frac = beats - whole;
        const double w = std::clamp(options.impulse_fraction, 1e-3, 1.0);
        const double x = std::clamp((frac - (1.0 - w)) / w, 0.0, 1.0);
        const double eased = x * x * (3.0 - 2.0 * x);  // smoothstep
        root_revs = (whole + eased) / train.beats_per_root_rev;
    } else {
        root_revs = seconds / 3600.0 / train.root_period_hours;
    }
    std::vector<double> angles(train.mobiles.size(), 0.0);
    for (std::size_t i = 0; i < angles.size(); ++i) {
        if (train.mobiles[i].reached) angles[i] = two_pi * root_revs * train.mobiles[i].ratio;
    }
    return angles;
}

}  // namespace ohs::kinematics
