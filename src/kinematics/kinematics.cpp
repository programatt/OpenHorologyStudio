// SPDX-License-Identifier: Apache-2.0
#include "kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>
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
    if (root < 0 || root >= nm) throw Error("no root mobile: the rig has no barrel and none was given");
    out.root = root;

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

}  // namespace ohs::kinematics
