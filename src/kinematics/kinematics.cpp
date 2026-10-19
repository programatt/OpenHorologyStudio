// SPDX-License-Identifier: Apache-2.0
#include "kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>

#include "core/error.hpp"
#include "model/followers.hpp"

namespace ohs::kinematics {

using model::Movement;
using model::Role;

const MobileMotion* TrainAnalysis::find(const std::string& mobile_name) const {
    for (const MobileMotion& m : mobiles) {
        if (m.name == mobile_name) return &m;
    }
    return nullptr;
}

namespace {

// A lever or spring that turns more than this on one tooth is not one the part features can place: its pivot or
// its free end is a poor estimate (radians, about 17 degrees).
constexpr double kMaxSwing = 0.3;

void build_articulations(const Movement& m, TrainAnalysis& train) {
    const auto mid_height = [](const model::PartFeatures& f) { return 0.5 * (f.z_min + f.z_max); };
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const model::Component& c = m.components[i];
        const model::PartFeatures& f = c.features;
        Articulation a;
        a.component = static_cast<int>(i);
        // The mobile of a toothed part, if the train reaches it.
        const auto driver_mobile = [&](int component) {
            const int mobile = m.mobile_of(component);
            return mobile >= 0 && train.mobiles[static_cast<std::size_t>(mobile)].reached ? mobile : -1;
        };
        const auto teeth_of = [&](int component) {
            const model::PartFeatures& d = m.components[static_cast<std::size_t>(component)].features;
            return d.inner_teeth >= 6 ? d.inner_teeth : d.teeth;
        };
        const model::Usable reached = [&](int component) { return driver_mobile(component) >= 0; };
        switch (c.role) {
            case Role::Stem: {
                const double along = std::hypot(f.axis[0], std::hypot(f.axis[1], f.axis[2]));
                if (f.length <= 0.0 || along <= 0.0) continue;
                a.kind = Articulation::Kind::Spin;
                a.point[0] = f.center.x;
                a.point[1] = f.center.y;
                a.point[2] = mid_height(f);
                for (int k = 0; k < 3; ++k) a.axis[k] = f.axis[k] / along;
                a.gain = -kWindingRatio;
                break;
            }
            case Role::Lever: {
                const auto contact = model::lever_contact(m.components, static_cast<int>(i), reached);
                if (!contact) continue;
                a.driver = driver_mobile(contact->driver);
                if (a.driver < 0) continue;
                a.kind = Articulation::Kind::Follow;
                a.teeth = teeth_of(contact->driver);
                a.point[0] = contact->pivot.x;
                a.point[1] = contact->pivot.y;
                a.point[2] = mid_height(f);
                a.amplitude = model::lever_swing(m.components, *contact);
                break;
            }
            case Role::Spring: {
                const auto flex = model::spring_flex(m.components, static_cast<int>(i), reached);
                if (!flex) continue;
                a.driver = driver_mobile(flex->driver);
                if (a.driver < 0) continue;
                a.kind = Articulation::Kind::Follow;
                a.teeth = teeth_of(flex->driver);
                a.point[0] = flex->fixed.x;
                a.point[1] = flex->fixed.y;
                a.point[2] = mid_height(f);
                a.amplitude = flex->swing;
                break;
            }
            default: continue;
        }
        if (a.kind == Articulation::Kind::Follow && (a.teeth < 6 || std::abs(a.amplitude) < 1e-4 || std::abs(a.amplitude) > kMaxSwing)) continue;
        train.articulations.push_back(a);
    }
}

}  // namespace

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
    auto is_date_ring = [&m](int mobile) {
        for (int c : m.mobiles[mobile].components) {
            if (m.components[c].role == Role::DateRing) return true;
        }
        return false;
    };
    for (int i = 0; i < nm; ++i) {
        // A date ring is stepped by a wheel of the train, not geared to it: see below.
        if (!out.mobiles[i].reached && !is_date_ring(i)) {
            out.warnings.push_back("mobile " + out.mobiles[i].name + " is not connected to the train");
        }
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

    // Date rings. Their speed needs absolute speeds to find the 24-hour wheel, so a train analysed for
    // ratios only leaves them out.
    if (out.root_period_hours > 0) {
        constexpr double day_rev_per_hour = 1.0 / 24.0;
        // The driver is the wheel of the train that turns once a day; the hour wheel, once in 12 h, gives the
        // direction when there is none.
        auto nearest = [&](int skip, double revs_per_hour) {
            int best = -1;
            double error = 0.03;
            for (int i = 0; i < nm; ++i) {
                if (i == skip || !out.mobiles[i].reached || out.mobiles[i].index_teeth > 0) continue;
                const double e = std::abs(out.mobiles[i].revs_per_hour / revs_per_hour - 1.0);
                if (e < error) {
                    error = e;
                    best = i;
                }
            }
            return best;
        };
        for (int i = 0; i < nm; ++i) {
            if (!is_date_ring(i)) continue;
            int teeth = 0;
            for (int c : m.mobiles[i].components) {
                if (m.components[c].role != Role::DateRing) continue;
                const model::PartFeatures& f = m.components[c].features;
                teeth = f.inner_teeth > 0 ? f.inner_teeth : f.teeth;
            }
            if (teeth <= 0) {
                teeth = 31;
                out.warnings.push_back("date ring " + out.mobiles[i].name + " shows no teeth: assuming 31, one for each day of the longest month");
            }
            MobileMotion& ring = out.mobiles[i];
            ring.reached = true;
            ring.index_teeth = teeth;
            ring.index_driver = nearest(i, day_rev_per_hour);
            double driver_ratio;  // the root turns this many times for each turn of the driver, with sign
            if (ring.index_driver >= 0) {
                driver_ratio = out.mobiles[ring.index_driver].ratio;
            } else {
                const int hour = nearest(i, 2.0 * day_rev_per_hour);
                const double sense = hour >= 0 && out.mobiles[hour].ratio < 0 ? -1.0 : 1.0;
                driver_ratio = sense * out.root_period_hours / 24.0;
                out.warnings.push_back("no wheel of the train turns once in 24 h: date ring " + ring.name +
                                       " advances one tooth for each 24 h of clock time");
            }
            ring.ratio = driver_ratio / teeth;
            ring.revs_per_hour = std::abs(ring.ratio) / out.root_period_hours;
        }
    }
    build_articulations(m, out);
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
        const MobileMotion& mm = train.mobiles[i];
        if (!mm.reached) continue;
        if (mm.index_teeth > 0) {
            // One tooth per turn of the driver, however the train itself is being posed; the driver
            // turns this many times in `seconds`, whole turns of it being whole days.
            const double driver_ratio = mm.ratio * mm.index_teeth;
            const double turns = seconds / 3600.0 / train.root_period_hours * std::abs(driver_ratio);
            const double whole = std::floor(turns);
            const double w = std::clamp(options.date_engage_fraction, 1e-3, 1.0);
            const double x = std::clamp((turns - whole - (1.0 - w)) / w, 0.0, 1.0);
            const double eased = x * x * (3.0 - 2.0 * x);
            angles[i] = (driver_ratio < 0 ? -1.0 : 1.0) * two_pi / mm.index_teeth * (whole + eased);
            continue;
        }
        angles[i] = two_pi * root_revs * mm.ratio;
    }
    return angles;
}

std::vector<double> articulation_angles(const TrainAnalysis& train, const std::vector<double>& mobile_angles) {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    std::vector<double> out(train.articulations.size(), 0.0);
    for (std::size_t i = 0; i < out.size(); ++i) {
        const Articulation& a = train.articulations[i];
        if (a.kind == Articulation::Kind::Spin) {
            const auto root = static_cast<std::size_t>(train.root);
            out[i] = root < mobile_angles.size() ? a.gain * mobile_angles[root] : 0.0;
            continue;
        }
        const auto driver = static_cast<std::size_t>(a.driver);
        if (driver >= mobile_angles.size()) continue;
        // Teeth gone past, whichever way the part turns; the part is at rest at the start of a tooth.
        const double teeth_gone = a.teeth * std::abs(mobile_angles[driver]) / two_pi;
        const double f = teeth_gone - std::floor(teeth_gone);
        const auto smooth = [](double x) { return x * x * (3.0 - 2.0 * x); };
        // Up the face of the tooth over 70 % of the pitch, back down over the rest.
        constexpr double up = 0.7;
        out[i] = a.amplitude * (f < up ? smooth(f / up) : 1.0 - smooth((f - up) / (1.0 - up)));
    }
    return out;
}

}  // namespace ohs::kinematics
