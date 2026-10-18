// SPDX-License-Identifier: Apache-2.0
#include "mesh/mesh2d.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "core/error.hpp"

namespace ohs::mesh {

namespace {

using LD = long double;

double orient(const P2& a, const P2& b, const P2& c) {
    return static_cast<double>((static_cast<LD>(b.x) - a.x) * (static_cast<LD>(c.y) - a.y) -
                               (static_cast<LD>(b.y) - a.y) * (static_cast<LD>(c.x) - a.x));
}

// > 0 when p is strictly inside the circumcircle of the counter-clockwise triangle abc.
LD incircle(const P2& a, const P2& b, const P2& c, const P2& p) {
    const LD adx = static_cast<LD>(a.x) - p.x, ady = static_cast<LD>(a.y) - p.y;
    const LD bdx = static_cast<LD>(b.x) - p.x, bdy = static_cast<LD>(b.y) - p.y;
    const LD cdx = static_cast<LD>(c.x) - p.x, cdy = static_cast<LD>(c.y) - p.y;
    const LD ad = adx * adx + ady * ady, bd = bdx * bdx + bdy * bdy, cd = cdx * cdx + cdy * cdy;
    return adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx);
}

double dist(const P2& a, const P2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

struct Tri {
    int v[3];
    int n[3];  // neighbour opposite v[i]; -1 on the outer hull
    bool alive = true;
    signed char inside = -1;  // lazily evaluated: -1 unknown, 0 outside the region, 1 inside
};

class Delaunay {
public:
    std::vector<P2> pts;
    std::vector<Tri> tris;
    std::vector<int> vtri;
    std::vector<int> created;  // triangles made by the last insert

    Delaunay(double cx, double cy, double d) {
        add_point({cx - 20 * d, cy - d});
        add_point({cx + 20 * d, cy - d});
        add_point({cx, cy + 20 * d});
        Tri t{{0, 1, 2}, {-1, -1, -1}};
        tris.push_back(t);
        vtri = {0, 0, 0};
        hint_ = 0;
    }

    int add_point(P2 p) {
        pts.push_back(p);
        vtri.push_back(-1);
        return static_cast<int>(pts.size()) - 1;
    }

    int locate(const P2& p) const {
        int t = alive_hint();
        for (std::size_t steps = 0; steps < tris.size() * 2 + 16; ++steps) {
            const Tri& tr = tris[t];
            bool moved = false;
            for (int i = 0; i < 3; ++i) {
                if (orient(pts[tr.v[(i + 1) % 3]], pts[tr.v[(i + 2) % 3]], p) < 0 && tr.n[i] >= 0) {
                    t = tr.n[i];
                    moved = true;
                    break;
                }
            }
            if (!moved) return t;
        }
        for (std::size_t i = 0; i < tris.size(); ++i) {  // the walk cycled (degenerate input): scan
            if (!tris[i].alive) continue;
            const Tri& tr = tris[i];
            if (orient(pts[tr.v[0]], pts[tr.v[1]], p) >= 0 && orient(pts[tr.v[1]], pts[tr.v[2]], p) >= 0 &&
                orient(pts[tr.v[2]], pts[tr.v[0]], p) >= 0) {
                return static_cast<int>(i);
            }
        }
        return alive_hint();
    }

    /// Inserts p; returns the vertex id (an existing one if p coincides with it).
    int insert(const P2& p, double merge_tol) {
        created.clear();
        const int t0 = locate(p);
        for (int k = 0; k < 3; ++k) {
            if (dist(pts[tris[t0].v[k]], p) <= merge_tol) return tris[t0].v[k];
        }
        std::vector<char> in_cavity(tris.size(), 0);
        std::vector<int> cavity{t0};
        in_cavity[t0] = 1;
        struct Edge {
            int a, b, outside;
        };
        std::vector<Edge> boundary;
        for (int round = 0; round < 64; ++round) {
            boundary.clear();
            for (std::size_t q = 0; q < cavity.size(); ++q) {
                const Tri& tr = tris[cavity[q]];
                for (int i = 0; i < 3; ++i) {
                    const int nb = tr.n[i];
                    if (nb >= 0 && !in_cavity[nb] && incircle(pts[tris[nb].v[0]], pts[tris[nb].v[1]], pts[tris[nb].v[2]], p) > 0) {
                        in_cavity[nb] = 1;
                        cavity.push_back(nb);
                    }
                }
            }
            // The cavity must be star-shaped around p; if p sits on a boundary edge, absorb the
            // neighbour across it and look again.
            bool grew = false;
            for (int ti : cavity) {
                const Tri& tr = tris[ti];
                for (int i = 0; i < 3; ++i) {
                    const int nb = tr.n[i];
                    if (nb >= 0 && in_cavity[nb]) continue;
                    const int a = tr.v[(i + 1) % 3], b = tr.v[(i + 2) % 3];
                    if (orient(pts[a], pts[b], p) <= 1e-14 * (dist(pts[a], pts[b]) * dist(pts[a], pts[b]) + 1e-300) && nb >= 0) {
                        in_cavity[nb] = 1;
                        cavity.push_back(nb);
                        grew = true;
                    }
                }
            }
            if (!grew) break;
        }
        for (int ti : cavity) {
            const Tri& tr = tris[ti];
            for (int i = 0; i < 3; ++i) {
                const int nb = tr.n[i];
                if (nb < 0 || !in_cavity[nb]) boundary.push_back({tr.v[(i + 1) % 3], tr.v[(i + 2) % 3], nb});
            }
        }
        const int pv = add_point(p);
        std::vector<int> fresh;
        fresh.reserve(boundary.size());
        for (const Edge& e : boundary) {
            Tri nt{{e.a, e.b, pv}, {-1, -1, e.outside}};
            tris.push_back(nt);
            const int id = static_cast<int>(tris.size()) - 1;
            fresh.push_back(id);
            if (e.outside >= 0) {
                Tri& o = tris[e.outside];
                for (int i = 0; i < 3; ++i) {
                    const int oa = o.v[(i + 1) % 3], ob = o.v[(i + 2) % 3];
                    if (oa == e.b && ob == e.a) o.n[i] = id;
                }
            }
        }
        for (std::size_t i = 0; i < fresh.size(); ++i) {
            for (std::size_t j = 0; j < fresh.size(); ++j) {
                if (tris[fresh[j]].v[0] == tris[fresh[i]].v[1]) {
                    tris[fresh[i]].n[0] = fresh[j];  // edge (b,p) is shared with the triangle starting at b
                    tris[fresh[j]].n[1] = fresh[i];  // its edge (p,a)
                }
            }
        }
        for (int ti : cavity) tris[ti].alive = false;
        for (int id : fresh) {
            for (int k = 0; k < 3; ++k) vtri[tris[id].v[k]] = id;
        }
        created = fresh;
        hint_ = fresh.empty() ? hint_ : fresh.front();
        return pv;
    }

    bool has_edge(int a, int b) const {
        int start = vtri[a];
        if (start < 0 || !tris[start].alive) start = find_incident(a);
        if (start < 0) return false;
        for (int dir : {1, 2}) {
            int t = start;
            for (std::size_t guard = 0; guard < tris.size() + 8 && t >= 0; ++guard) {
                const Tri& tr = tris[t];
                int k = 0;
                while (tr.v[k] != a) ++k;
                if (tr.v[(k + 1) % 3] == b || tr.v[(k + 2) % 3] == b) return true;
                t = tr.n[(k + dir) % 3];
                if (t == start) break;
            }
        }
        return false;
    }

private:
    int hint_ = 0;

    int alive_hint() const {
        if (tris[hint_].alive) return hint_;
        for (std::size_t i = tris.size(); i-- > 0;) {
            if (tris[i].alive) return static_cast<int>(i);
        }
        return 0;
    }

    int find_incident(int a) const {
        for (std::size_t i = 0; i < tris.size(); ++i) {
            if (!tris[i].alive) continue;
            for (int k = 0; k < 3; ++k) {
                if (tris[i].v[k] == a) return static_cast<int>(i);
            }
        }
        return -1;
    }
};

bool point_in_region(const Region& r, const P2& p) {
    bool inside = false;
    for (const Ring& ring : r.rings) {
        const std::size_t n = ring.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const P2& a = ring[i];
            const P2& b = ring[j];
            if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
        }
    }
    return inside;
}

struct Segment {
    int a, b;
    bool alive = true;
};

P2 circumcenter(const P2& a, const P2& b, const P2& c) {
    const double bx = b.x - a.x, by = b.y - a.y, cx = c.x - a.x, cy = c.y - a.y;
    const double d = 2 * (bx * cy - by * cx);
    const double ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / d;
    const double uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / d;
    return {a.x + ux, a.y + uy};
}

double min_angle_deg(const P2& a, const P2& b, const P2& c) {
    const double la = dist(b, c), lb = dist(a, c), lc = dist(a, b);
    auto ang = [](double opp, double s1, double s2) {
        return std::acos(std::clamp((s1 * s1 + s2 * s2 - opp * opp) / (2 * s1 * s2), -1.0, 1.0));
    };
    return std::min({ang(la, lb, lc), ang(lb, la, lc), ang(lc, la, lb)}) * 180.0 / std::numbers::pi;
}

}  // namespace

Mesh2d triangulate(const Region& region, const Mesh2dOptions& opt) {
    if (region.rings.empty()) throw Error("mesh: the region has no outline");
    if (opt.size <= 0) throw Error("mesh: the element size must be positive");

    // Clean rings: drop repeated points, need three or more corners.
    Region clean;
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const Ring& ring : region.rings) {
        Ring r;
        for (const P2& p : ring) {
            if (r.empty() || dist(r.back(), p) > 1e-12) r.push_back(p);
        }
        while (r.size() > 1 && dist(r.front(), r.back()) <= 1e-12) r.pop_back();
        if (r.size() < 3) throw Error("mesh: an outline needs at least three distinct corners");
        for (const P2& p : r) {
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }
        clean.rings.push_back(std::move(r));
    }
    auto size_at = [&](const P2& p) { return opt.size_at ? opt.size_at(p) : opt.size; };
    const double extent = std::max(maxx - minx, maxy - miny);
    double hmin = opt.size;
    if (opt.size_at) {
        for (const Ring& r : clean.rings) {
            for (const P2& p : r) hmin = std::min(hmin, size_at(p));
        }
    }
    const double merge_tol = 1e-9 * extent;
    const double min_len = std::max(1e-6 * extent, 0.02 * hmin);

    Delaunay dt(0.5 * (minx + maxx), 0.5 * (miny + maxy), extent);
    std::vector<Segment> segs;
    auto check_budget = [&] {
        if (static_cast<int>(dt.pts.size()) > opt.max_nodes) {
            throw Error("mesh: more than " + std::to_string(opt.max_nodes) + " nodes would be needed; use a larger element size");
        }
    };

    for (const Ring& ring : clean.rings) {
        std::vector<int> ids;
        const std::size_t n = ring.size();
        for (std::size_t i = 0; i < n; ++i) {
            const P2& p = ring[i];
            const P2& q = ring[(i + 1) % n];
            const double len = dist(p, q);
            const P2 mid{0.5 * (p.x + q.x), 0.5 * (p.y + q.y)};
            const int pieces = std::max(1, static_cast<int>(std::ceil(len / size_at(mid) - 1e-9)));
            for (int k = 0; k < pieces; ++k) {
                const double s = static_cast<double>(k) / pieces;
                ids.push_back(dt.insert({p.x + s * (q.x - p.x), p.y + s * (q.y - p.y)}, merge_tol));
                check_budget();
            }
        }
        for (std::size_t i = 0; i < ids.size(); ++i) {
            const int a = ids[i], b = ids[(i + 1) % ids.size()];
            if (a != b) segs.push_back({a, b});
        }
    }

    // Spatial hash of the segments' diametral circles, for encroachment tests.
    const double cell = std::max(hmin, extent * 1e-4);
    std::unordered_map<long long, std::vector<int>> grid;
    auto cell_key = [&](long long i, long long j) { return i * 1000003LL + j; };
    auto grid_add = [&](int si) {
        const Segment& s = segs[si];
        const P2 m{0.5 * (dt.pts[s.a].x + dt.pts[s.b].x), 0.5 * (dt.pts[s.a].y + dt.pts[s.b].y)};
        const double r = 0.5 * dist(dt.pts[s.a], dt.pts[s.b]);
        for (long long i = static_cast<long long>(std::floor((m.x - r - minx) / cell)); i <= static_cast<long long>(std::floor((m.x + r - minx) / cell)); ++i) {
            for (long long j = static_cast<long long>(std::floor((m.y - r - miny) / cell)); j <= static_cast<long long>(std::floor((m.y + r - miny) / cell)); ++j) {
                grid[cell_key(i, j)].push_back(si);
            }
        }
    };

    // Conforming step: split any boundary segment that is not yet an edge of the triangulation.
    for (int pass = 0; pass < 200; ++pass) {
        bool changed = false;
        for (std::size_t si = 0; si < segs.size(); ++si) {
            if (!segs[si].alive || dt.has_edge(segs[si].a, segs[si].b)) continue;
            const int a = segs[si].a, b = segs[si].b;
            if (dist(dt.pts[a], dt.pts[b]) < min_len) continue;
            const P2 m{0.5 * (dt.pts[a].x + dt.pts[b].x), 0.5 * (dt.pts[a].y + dt.pts[b].y)};
            const int mid = dt.insert(m, merge_tol);
            check_budget();
            segs[si].alive = false;
            segs.push_back({a, mid});
            segs.push_back({mid, b});
            changed = true;
        }
        if (!changed) break;
    }
    for (std::size_t si = 0; si < segs.size(); ++si) {
        if (segs[si].alive) grid_add(static_cast<int>(si));
    }

    auto inside_tri = [&](int ti) {
        Tri& t = dt.tris[ti];
        if (t.inside < 0) {
            bool super = false;
            for (int k = 0; k < 3; ++k) super |= t.v[k] < 3;
            if (super) {
                t.inside = 0;
            } else {
                const P2& a = dt.pts[t.v[0]];
                const P2& b = dt.pts[t.v[1]];
                const P2& c = dt.pts[t.v[2]];
                t.inside = point_in_region(clean, {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3}) ? 1 : 0;
            }
        }
        return t.inside == 1;
    };

    auto encroached_by = [&](const P2& p, int skip_a, int skip_b) {
        std::vector<int> out;
        const long long i = static_cast<long long>(std::floor((p.x - minx) / cell));
        const long long j = static_cast<long long>(std::floor((p.y - miny) / cell));
        auto it = grid.find(cell_key(i, j));
        if (it == grid.end()) return out;
        for (int si : it->second) {
            const Segment& s = segs[si];
            if (!s.alive || s.a == skip_a || s.b == skip_a || s.a == skip_b || s.b == skip_b) continue;
            const P2 m{0.5 * (dt.pts[s.a].x + dt.pts[s.b].x), 0.5 * (dt.pts[s.a].y + dt.pts[s.b].y)};
            const double r = 0.5 * dist(dt.pts[s.a], dt.pts[s.b]);
            if (dist(m, p) < r * (1 - 1e-9) && std::find(out.begin(), out.end(), si) == out.end()) out.push_back(si);
        }
        return out;
    };

    std::vector<int> work;  // triangles to examine
    auto queue_created = [&] { work.insert(work.end(), dt.created.begin(), dt.created.end()); };
    for (std::size_t i = 0; i < dt.tris.size(); ++i) {
        if (dt.tris[i].alive) work.push_back(static_cast<int>(i));
    }

    // Splits a segment at its midpoint, then splits whatever the new vertex encroaches.
    auto split_segment = [&](int first) {
        std::vector<int> stack{first};
        while (!stack.empty()) {
            const int si = stack.back();
            stack.pop_back();
            if (!segs[si].alive) continue;
            const int a = segs[si].a, b = segs[si].b;
            if (dist(dt.pts[a], dt.pts[b]) < 2 * min_len) continue;
            const P2 m{0.5 * (dt.pts[a].x + dt.pts[b].x), 0.5 * (dt.pts[a].y + dt.pts[b].y)};
            const int mid = dt.insert(m, 0.0);
            check_budget();
            queue_created();
            segs[si].alive = false;
            segs.push_back({a, mid});
            grid_add(static_cast<int>(segs.size()) - 1);
            segs.push_back({mid, b});
            grid_add(static_cast<int>(segs.size()) - 1);
            for (int e : encroached_by(dt.pts[mid], a, b)) stack.push_back(e);
        }
    };

    const double max_ratio = opt.max_radius_edge_ratio;
    while (!work.empty()) {
        const int ti = work.back();
        work.pop_back();
        if (!dt.tris[ti].alive || !inside_tri(ti)) continue;
        const Tri t = dt.tris[ti];
        const P2& a = dt.pts[t.v[0]];
        const P2& b = dt.pts[t.v[1]];
        const P2& c = dt.pts[t.v[2]];
        const double la = dist(b, c), lb = dist(a, c), lc = dist(a, b);
        const double shortest = std::min({la, lb, lc});
        const double tri_area = 0.5 * std::abs(orient(a, b, c));
        if (tri_area <= 0 || shortest < min_len) continue;
        const double radius = la * lb * lc / (4 * tri_area);
        const P2 centroid{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        const double h = size_at(centroid);
        const bool skinny = radius / shortest > max_ratio;
        const bool big = tri_area > 0.4330127 * h * h;
        if (!skinny && !big) continue;

        const P2 cc = circumcenter(a, b, c);
        const auto enc = encroached_by(cc, -1, -1);
        if (!enc.empty()) {
            for (int si : enc) split_segment(si);
            work.push_back(ti);
            continue;
        }
        if (!point_in_region(clean, cc)) {
            // Outside the domain without encroaching a segment cannot happen in exact arithmetic;
            // fall back to splitting the longest edge of the triangle when it is on the boundary.
            continue;
        }
        dt.insert(cc, 0.0);
        check_budget();
        queue_created();
    }

    // Extract the interior.
    Mesh2d out;
    std::vector<int> remap(dt.pts.size(), -1);
    auto use = [&](int v) {
        if (remap[v] < 0) {
            remap[v] = static_cast<int>(out.nodes.size());
            out.nodes.push_back(dt.pts[v]);
        }
        return remap[v];
    };
    for (std::size_t i = 0; i < dt.tris.size(); ++i) {
        if (!dt.tris[i].alive || !inside_tri(static_cast<int>(i))) continue;
        out.triangles.push_back({use(dt.tris[i].v[0]), use(dt.tris[i].v[1]), use(dt.tris[i].v[2])});
    }
    for (const Segment& s : segs) {
        if (s.alive && remap[s.a] >= 0 && remap[s.b] >= 0) out.boundary.push_back({remap[s.a], remap[s.b]});
    }

    // Laplacian smoothing of the interior nodes, kept only where the worst angle does not drop.
    std::vector<char> fixed(out.nodes.size(), 0);
    for (const auto& s : out.boundary) fixed[s[0]] = fixed[s[1]] = 1;
    std::vector<std::vector<int>> around(out.nodes.size());
    for (std::size_t i = 0; i < out.triangles.size(); ++i) {
        for (int v : out.triangles[i]) around[v].push_back(static_cast<int>(i));
    }
    auto worst = [&](int v) {
        double w = 180;
        for (int t : around[v]) {
            const auto& tr = out.triangles[t];
            if (orient(out.nodes[tr[0]], out.nodes[tr[1]], out.nodes[tr[2]]) <= 0) return -1.0;
            w = std::min(w, min_angle_deg(out.nodes[tr[0]], out.nodes[tr[1]], out.nodes[tr[2]]));
        }
        return w;
    };
    for (int pass = 0; pass < opt.smoothing_passes; ++pass) {
        for (std::size_t v = 0; v < out.nodes.size(); ++v) {
            if (fixed[v] || around[v].empty()) continue;
            double sx = 0, sy = 0;
            int cnt = 0;
            for (int t : around[v]) {
                for (int u : out.triangles[t]) {
                    if (u == static_cast<int>(v)) continue;
                    sx += out.nodes[u].x;
                    sy += out.nodes[u].y;
                    ++cnt;
                }
            }
            const P2 old = out.nodes[v];
            const double before = worst(static_cast<int>(v));
            out.nodes[v] = {sx / cnt, sy / cnt};
            if (worst(static_cast<int>(v)) < before) out.nodes[v] = old;
        }
    }
    return out;
}

std::array<double, 2> angle_range(const Mesh2d& m) {
    double lo = 180, hi = 0;
    for (const auto& t : m.triangles) {
        const P2& a = m.nodes[t[0]];
        const P2& b = m.nodes[t[1]];
        const P2& c = m.nodes[t[2]];
        lo = std::min(lo, min_angle_deg(a, b, c));
        const double la = dist(b, c), lb = dist(a, c), lc = dist(a, b);
        auto ang = [](double opp, double s1, double s2) {
            return std::acos(std::clamp((s1 * s1 + s2 * s2 - opp * opp) / (2 * s1 * s2), -1.0, 1.0)) * 180.0 / std::numbers::pi;
        };
        hi = std::max({hi, ang(la, lb, lc), ang(lb, la, lc), ang(lc, la, lb)});
    }
    return {lo, hi};
}

double area(const Mesh2d& m) {
    double s = 0;
    for (const auto& t : m.triangles) s += 0.5 * orient(m.nodes[t[0]], m.nodes[t[1]], m.nodes[t[2]]);
    return s;
}

}  // namespace ohs::mesh
