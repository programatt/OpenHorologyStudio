// SPDX-License-Identifier: Apache-2.0
#include "dynamics/torque_chain.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>

#include "core/error.hpp"

namespace ohs::dynamics {

TorqueChain build_chain(const model::Movement& m, const kinematics::TrainAnalysis& train, const Calibre& calibre) {
    const int nm = static_cast<int>(m.mobiles.size());
    if (train.escape_mobile < 0 || !train.mobiles[train.escape_mobile].reached) {
        throw Error("the escape wheel is not connected to the barrel, so there is no torque path");
    }
    std::vector<std::vector<int>> adj(nm);
    for (const model::MeshJoint& j : m.meshes) {
        const int a = m.mobile_of(j.a), b = m.mobile_of(j.b);
        if (a < 0 || b < 0) continue;
        adj[a].push_back(b);
        adj[b].push_back(a);
    }
    std::vector<int> parent(nm, -2);
    std::queue<int> pending;
    parent[train.root] = -1;
    pending.push(train.root);
    while (!pending.empty()) {
        const int cur = pending.front();
        pending.pop();
        for (int next : adj[cur]) {
            if (parent[next] == -2) {
                parent[next] = cur;
                pending.push(next);
            }
        }
    }
    std::vector<int> path;
    for (int i = train.escape_mobile; i != -1; i = parent[i]) path.push_back(i);
    std::reverse(path.begin(), path.end());

    TorqueChain chain;
    chain.escape_teeth = train.escape_teeth;
    chain.barrel_period_hours = train.root_period_hours;
    chain.meshes = static_cast<int>(path.size()) - 1;
    chain.speed_ratio = std::abs(train.mobiles[train.escape_mobile].ratio);
    chain.efficiency = std::pow(calibre.mesh_efficiency, chain.meshes);
    for (std::size_t i = 0; i < path.size(); ++i) {
        TorqueStage s;
        s.mobile = m.mobiles[path[i]].name;
        s.meshes_from_barrel = static_cast<int>(i);
        s.speed_ratio = std::abs(train.mobiles[path[i]].ratio);
        s.fraction = std::pow(calibre.mesh_efficiency, static_cast<double>(i)) / s.speed_ratio;
        chain.stages.push_back(s);
    }
    return chain;
}

TorqueChain direct_chain(double speed_ratio, int meshes, const Calibre& calibre, int escape_teeth,
                         double barrel_period_hours) {
    TorqueChain chain;
    chain.meshes = meshes;
    chain.speed_ratio = speed_ratio;
    chain.efficiency = std::pow(calibre.mesh_efficiency, meshes);
    chain.escape_teeth = escape_teeth;
    chain.barrel_period_hours = barrel_period_hours;
    chain.stages.push_back({"barrel", 0, 1.0, 1.0});
    chain.stages.push_back({"escape", meshes, speed_ratio, chain.efficiency / speed_ratio});
    return chain;
}

double escape_torque_nm(const TorqueChain& chain, const Calibre& calibre, double wind) {
    const double barrel = calibre.mainspring_torque_nmm(wind) * 1e-3;
    const double arriving = barrel * chain.transmission() - calibre.escape_pivot_loss_nmm * 1e-3;
    return std::max(arriving, 0.0);
}

std::vector<TorqueRow> torque_table(const TorqueChain& chain, const Calibre& calibre, double wind) {
    const double barrel = calibre.mainspring_torque_nmm(wind);
    std::vector<TorqueRow> rows;
    double previous = barrel;
    for (std::size_t i = 0; i < chain.stages.size(); ++i) {
        const TorqueStage& s = chain.stages[i];
        TorqueRow r;
        r.mobile = s.mobile;
        r.torque_nmm = barrel * s.fraction;
        if (i > 0) {
            // Torque that would arrive through a lossless mesh, minus what does.
            const double lossless = previous * chain.stages[i - 1].speed_ratio / s.speed_ratio;
            r.lost_nmm = lossless - r.torque_nmm;
        }
        previous = r.torque_nmm;
        rows.push_back(r);
    }
    if (!rows.empty()) {
        rows.back().torque_nmm = std::max(rows.back().torque_nmm - calibre.escape_pivot_loss_nmm, 0.0);
    }
    return rows;
}

double escape_power_w(const TorqueChain& chain, const Calibre& calibre, double wind) {
    // The escape wheel advances 2*pi/escape_teeth per two beats.
    const double wheel_rad_per_s = 2.0 * std::numbers::pi / chain.escape_teeth * calibre.beat_rate_vph / 3600.0 / 2.0;
    return escape_torque_nm(chain, calibre, wind) * wheel_rad_per_s;
}

void adopt_train(Calibre& calibre, const TorqueChain& chain, double beat_rate_vph) {
    if (chain.escape_teeth > 0) calibre.escape_teeth = chain.escape_teeth;
    if (beat_rate_vph > 0) calibre.beat_rate_vph = beat_rate_vph;
}

}  // namespace ohs::dynamics
