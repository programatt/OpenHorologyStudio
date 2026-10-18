// SPDX-License-Identifier: Apache-2.0
#include "manufacturing/collision.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <unordered_map>

#include "core/error.hpp"
#include "core/parallel.hpp"
#include "geometry/section.hpp"

namespace ohs::manufacturing {

namespace {

using mesh::P2;
using mesh::Region;
using mesh::Ring;
constexpr double kPi = std::numbers::pi;

struct Seg {
    P2 a, b;
};

std::vector<Seg> segments(const Region& r) {
    std::vector<Seg> s;
    for (const Ring& ring : r.rings) {
        for (std::size_t i = 0; i < ring.size(); ++i) s.push_back({ring[i], ring[(i + 1) % ring.size()]});
    }
    return s;
}

bool inside_solid(const Region& r, const P2& p) {
    bool in = false;
    for (const Ring& ring : r.rings) {
        for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
            const P2& a = ring[i];
            const P2& b = ring[j];
            if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
        }
    }
    return in;
}

double point_seg(const P2& p, const Seg& s) {
    const double dx = s.b.x - s.a.x, dy = s.b.y - s.a.y, l2 = dx * dx + dy * dy;
    const double t = l2 > 0 ? std::clamp(((p.x - s.a.x) * dx + (p.y - s.a.y) * dy) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (s.a.x + t * dx), p.y - (s.a.y + t * dy));
}

double orient(const P2& a, const P2& b, const P2& c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

bool cross(const Seg& s, const Seg& t) {
    const double d1 = orient(t.a, t.b, s.a), d2 = orient(t.a, t.b, s.b), d3 = orient(s.a, s.b, t.a), d4 = orient(s.a, s.b, t.b);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

double seg_seg(const Seg& s, const Seg& t) {
    return std::min({point_seg(s.a, t), point_seg(s.b, t), point_seg(t.a, s), point_seg(t.b, s)});
}

/// Uniform grid over a set of segments, for distance queries limited to a range.
class Grid {
public:
    Grid(const std::vector<Seg>& segs, double cell) : segs_(segs), cell_(cell) {
        for (std::size_t i = 0; i < segs.size(); ++i) {
            const Seg& s = segs[i];
            for (long long x = cx(std::min(s.a.x, s.b.x)); x <= cx(std::max(s.a.x, s.b.x)); ++x) {
                for (long long y = cx(std::min(s.a.y, s.b.y)); y <= cx(std::max(s.a.y, s.b.y)); ++y) cells_[key(x, y)].push_back(static_cast<int>(i));
            }
        }
    }
    template <class F>
    void near(const Seg& q, double range, F f) const {
        std::vector<int> seen;
        for (long long x = cx(std::min(q.a.x, q.b.x) - range); x <= cx(std::max(q.a.x, q.b.x) + range); ++x) {
            for (long long y = cx(std::min(q.a.y, q.b.y) - range); y <= cx(std::max(q.a.y, q.b.y) + range); ++y) {
                auto it = cells_.find(key(x, y));
                if (it == cells_.end()) continue;
                for (int i : it->second) {
                    if (std::find(seen.begin(), seen.end(), i) == seen.end()) {
                        seen.push_back(i);
                        f(segs_[static_cast<std::size_t>(i)]);
                    }
                }
            }
        }
    }

private:
    long long cx(double v) const { return static_cast<long long>(std::floor(v / cell_)); }
    static long long key(long long x, long long y) { return x * 2000003LL + y; }
    const std::vector<Seg>& segs_;
    double cell_;
    std::unordered_map<long long, std::vector<int>> cells_;
};

struct Relation {
    bool overlap{false};
    double clearance{1e30};  ///< min distance when separate; negative depth estimate when overlapping
};

Relation relate(const Region& A, const Region& B, double range) {
    const auto sa = segments(A), sb = segments(B);
    const Grid grid(sb, std::max(range, 1e-3));
    Relation r;
    double best = 1e30;
    bool crossing = false;
    for (const Seg& s : sa) {
        grid.near(s, range, [&](const Seg& t) {
            if (cross(s, t)) crossing = true;
            best = std::min(best, seg_seg(s, t));
        });
    }
    // Containment without crossing: a vertex of one inside the solid of the other.
    bool a_in_b = false, b_in_a = false;
    if (!crossing && !A.rings.empty() && !A.rings[0].empty()) a_in_b = inside_solid(B, A.rings[0][0]);
    if (!crossing && !B.rings.empty() && !B.rings[0].empty()) b_in_a = inside_solid(A, B.rings[0][0]);
    r.overlap = crossing || a_in_b || b_in_a;
    if (!r.overlap) {
        r.clearance = best;
        return r;
    }
    // Penetration estimate: the deepest vertex of either outline inside the other's solid, looked
    // for only among the vertices within `range` of the other outline.
    double depth = 0;
    auto deepest = [&](const Region& P, const Region& Q, const Grid& gq) {
        for (const Ring& ring : P.rings) {
            for (const P2& p : ring) {
                double nearest = 1e30;
                gq.near({p, p}, range, [&](const Seg& t) { nearest = std::min(nearest, point_seg(p, t)); });
                if (nearest > range) continue;
                if (inside_solid(Q, p)) depth = std::max(depth, nearest);
            }
        }
    };
    const Grid ga(sa, std::max(range, 1e-3));
    deepest(A, B, grid);
    deepest(B, A, ga);
    r.clearance = -depth;
    return r;
}

Region rotated(const Region& r, P2 centre, double angle) {
    if (angle == 0.0) return r;
    const double c = std::cos(angle), s = std::sin(angle);
    Region out;
    for (const Ring& ring : r.rings) {
        Ring q;
        q.reserve(ring.size());
        for (const P2& p : ring) q.push_back({centre.x + (p.x - centre.x) * c - (p.y - centre.y) * s, centre.y + (p.x - centre.x) * s + (p.y - centre.y) * c});
        out.rings.push_back(std::move(q));
    }
    return out;
}

double radius_about(const Region& r, P2 c) {
    double m = 0;
    for (const Ring& ring : r.rings) {
        for (const P2& p : ring) m = std::max(m, std::hypot(p.x - c.x, p.y - c.y));
    }
    return m;
}

}  // namespace

CollisionReport sweep_collisions(const model::Movement& m, const kinematics::TrainAnalysis& train, const std::vector<CollisionPart>& parts,
                                 const CollisionConfig& cfg) {
    CollisionReport report;
    auto ratio = [&](int mobile) { return mobile >= 0 && train.mobiles[static_cast<std::size_t>(mobile)].reached ? train.mobiles[static_cast<std::size_t>(mobile)].ratio : 0.0; };
    auto centre = [&](int mobile) { return mobile >= 0 ? P2{m.mobiles[static_cast<std::size_t>(mobile)].center.x, m.mobiles[static_cast<std::size_t>(mobile)].center.y} : P2{0, 0}; };

    // Meshing component pairs and the driver's tooth pitch.
    std::map<std::pair<std::string, std::string>, double> mesh_pitch;
    for (const model::MeshJoint& j : m.meshes) {
        const std::string& a = m.components[static_cast<std::size_t>(j.a)].features.name;
        const std::string& b = m.components[static_cast<std::size_t>(j.b)].features.name;
        const int za = m.components[static_cast<std::size_t>(j.a)].features.teeth;
        if (za <= 0) continue;
        mesh_pitch[{a, b}] = 2 * kPi / za;
        mesh_pitch[{b, a}] = 2 * kPi / m.components[static_cast<std::size_t>(j.b)].features.teeth;
    }

    std::vector<double> reach(parts.size());
    std::vector<P2> mid(parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const CollisionPart& p = parts[i];
        if (p.mobile >= 0) {
            mid[i] = centre(p.mobile);
            reach[i] = radius_about(p.region, mid[i]);
        } else {
            double lx = 1e30, ly = 1e30, hx = -1e30, hy = -1e30;
            for (const Ring& ring : p.region.rings) {
                for (const P2& q : ring) {
                    lx = std::min(lx, q.x);
                    hx = std::max(hx, q.x);
                    ly = std::min(ly, q.y);
                    hy = std::max(hy, q.y);
                }
            }
            mid[i] = {0.5 * (lx + hx), 0.5 * (ly + hy)};
            reach[i] = 0.5 * std::hypot(hx - lx, hy - ly);
        }
    }

    // Candidate pairs first, then the sweeps on threads: each pair is independent, and results are
    // collected in pair order so the report does not depend on scheduling.
    struct Candidate {
        std::size_t i, j;
        double ra, rb, sweep_phi, faster;
        int steps;
        bool meshing;
    };
    std::vector<Candidate> candidates;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        for (std::size_t j = i + 1; j < parts.size(); ++j) {
            const CollisionPart &A = parts[i], &B = parts[j];
            if (A.mobile == B.mobile) continue;  // fixed to each other
            const double ra = ratio(A.mobile), rb = ratio(B.mobile);
            if (ra == 0.0 && rb == 0.0) continue;
            if (!(A.z0 < B.z1 - 1e-6 && B.z0 < A.z1 - 1e-6)) continue;
            // Static parts and mobiles with ratio 0 do not move; a moving part sweeps a disc about its axis.
            const double dist = std::hypot(mid[i].x - mid[j].x, mid[i].y - mid[j].y);
            if (dist > reach[i] + reach[j] + cfg.search_range) continue;
            Candidate c{i, j, ra, rb, 0, 0, 0, false};
            const auto pitch = mesh_pitch.find({A.name, B.name});
            if (pitch != mesh_pitch.end() && ra != 0.0) {
                c.meshing = true;
                c.steps = cfg.steps_per_pitch;
                c.sweep_phi = pitch->second / std::abs(ra);
                c.faster = std::abs(ra);
            } else {
                c.steps = cfg.steps_per_revolution;
                c.faster = std::max(std::abs(ra), std::abs(rb));
                c.sweep_phi = 2 * kPi / c.faster;
            }
            candidates.push_back(c);
        }
    }
    report.pairs.resize(candidates.size());
    ohs::parallel_for(candidates.size(), [&](std::size_t k) {
        const Candidate& c = candidates[k];
        const CollisionPart &A = parts[c.i], &B = parts[c.j];
        CollisionPair& cp = report.pairs[k];
        cp.a = A.name;
        cp.b = B.name;
        cp.meshing = c.meshing;
        cp.steps = c.steps + 1;
        cp.min_clearance = 1e30;
        for (int s = 0; s <= c.steps; ++s) {
            const double phi = c.sweep_phi * s / c.steps;
            const Region pa = rotated(A.region, centre(A.mobile), c.ra * phi);
            const Region pb = rotated(B.region, centre(B.mobile), c.rb * phi);
            const Relation rel = relate(pa, pb, cfg.search_range);
            if (rel.overlap && rel.clearance < -cfg.overlap_tolerance) ++cp.overlapping_steps;
            if (rel.clearance < cp.min_clearance) {
                cp.min_clearance = rel.clearance;
                cp.at_degrees = c.faster * phi * 180.0 / kPi;
            }
        }
    });
    for (const CollisionPair& cp : report.pairs) {
        ++report.pairs_checked;
        if (cp.overlapping_steps > 0) ++report.overlaps;
        else if (cp.min_clearance < cfg.warn_clearance) ++report.close_calls;
    }
    return report;
}

std::vector<CollisionPart> collision_parts(const model::Movement& m, const std::vector<geometry::ImportedPart>& step_parts, geometry::Axis axis,
                                           double tol) {
    std::vector<CollisionPart> out;
    for (std::size_t ci = 0; ci < m.components.size(); ++ci) {
        const auto& c = m.components[ci];
        if (c.features.volume <= 0) continue;
        const geometry::ImportedPart* found = nullptr;
        for (const auto& p : step_parts) {
            if (p.name == c.features.name) found = &p;
        }
        if (!found) continue;
        CollisionPart cp;
        cp.name = c.features.name;
        cp.z0 = c.features.z_min;
        cp.z1 = c.features.z_max;
        cp.mobile = m.mobile_of(static_cast<int>(ci));
        const TopoDS_Shape shape = geometry::orient_dial_axis_to_z(found->shape, axis);
        cp.region = geometry::section_region(shape, 0.5 * (cp.z0 + cp.z1), tol * 0.5, tol);
        out.push_back(std::move(cp));
    }
    return out;
}

}  // namespace ohs::manufacturing
