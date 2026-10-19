// SPDX-License-Identifier: Apache-2.0
#include "manufacturing/dfm.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <unordered_map>

#include "core/error.hpp"

namespace ohs::manufacturing {

extern const char* const kDfmRulesToml;  // generated from data/dfm/rules.toml

namespace {

using mesh::P2;
using mesh::Ring;
constexpr double kPi = std::numbers::pi;
constexpr double kHuge = 1e30;

const char* const kKnownRules[] = {"internal_radius", "depth_to_diameter", "min_wall", "module_cutter", "min_teeth", "undercut", "pinion_leaves", "jewel_fit"};

double signed_area(const Ring& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
    return 0.5 * a;
}

/// Outer ring counter-clockwise, holes clockwise: material is always on the left.
std::vector<Ring> oriented(const mesh::Region& region) {
    std::vector<Ring> out;
    for (std::size_t i = 0; i < region.rings.size(); ++i) {
        Ring r = region.rings[i];
        const bool ccw = signed_area(r) > 0;
        if (ccw != (i == 0)) std::reverse(r.begin(), r.end());
        out.push_back(std::move(r));
    }
    return out;
}

std::string num(double v, int prec = 3) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

}  // namespace

const char* to_string(Severity s) {
    switch (s) {
        case Severity::Info: return "ok";
        case Severity::Warning: return "warning";
        case Severity::Error: return "error";
    }
    return "?";
}

double ProcessRules::limit(const std::string& key, double fallback) const {
    const auto it = limits.find(key);
    return it == limits.end() ? fallback : it->second;
}

const std::vector<double>& ProcessRules::list(const std::string& key) const {
    static const std::vector<double> empty;
    const auto it = lists.find(key);
    return it == lists.end() ? empty : it->second;
}

std::vector<ProcessRules> rule_sets_from_toml(const std::string& text) {
    toml::table root;
    try {
        root = toml::parse(text);
    } catch (const toml::parse_error& e) {
        throw Error(std::string("rules file: ") + std::string(e.description()));
    }
    const toml::table* procs = root["process"].as_table();
    if (!procs) throw Error("rules file: no [process.<name>] tables");
    std::vector<ProcessRules> out;
    for (const auto& [name, node] : *procs) {
        const toml::table* t = node.as_table();
        if (!t) throw Error("rules file: process '" + std::string(name.str()) + "' must be a table");
        ProcessRules p;
        p.name = std::string(name.str());
        for (const auto& [key, value] : *t) {
            const std::string k(key.str());
            if (k == "description") {
                p.description = value.value_or(std::string());
            } else if (k == "rules") {
                const toml::array* a = value.as_array();
                if (!a) throw Error("rules file: 'rules' of " + p.name + " must be an array");
                for (const auto& el : *a) {
                    const std::string r = el.value_or(std::string());
                    if (std::none_of(std::begin(kKnownRules), std::end(kKnownRules), [&](const char* known) { return r == known; })) {
                        throw Error("rules file: unknown rule '" + r + "' in " + p.name);
                    }
                    p.rules.push_back(r);
                }
            } else if (const toml::array* a = value.as_array()) {
                std::vector<double> v;
                for (const auto& el : *a) v.push_back(el.value_or(0.0));
                p.lists[k] = std::move(v);
            } else if (auto d = value.value<double>()) {
                p.limits[k] = *d;
            } else {
                throw Error("rules file: '" + k + "' of " + p.name + " must be a number or an array of numbers");
            }
        }
        out.push_back(std::move(p));
    }
    return out;
}

std::vector<ProcessRules> load_rule_sets(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();
    return rule_sets_from_toml(buf.str());
}

const std::vector<ProcessRules>& builtin_rule_sets() {
    static const std::vector<ProcessRules> v = rule_sets_from_toml(kDfmRulesToml);
    return v;
}

const ProcessRules& find_process(std::string_view name, const std::vector<ProcessRules>& extra) {
    for (const ProcessRules& p : extra) {
        if (p.name == name) return p;
    }
    return find_process(name);
}

const ProcessRules& find_process(std::string_view name) {
    for (const ProcessRules& p : builtin_rule_sets()) {
        if (p.name == name) return p;
    }
    throw Error("unknown process '" + std::string(name) + "' (ohs dfm --list shows them)");
}

double min_internal_radius(const mesh::Region& region, P2* where) {
    double best = kHuge;
    for (const Ring& r : oriented(region)) {
        const std::size_t n = r.size();
        for (std::size_t i = 0; i < n; ++i) {
            const P2& p = r[(i + n - 1) % n];
            const P2& q = r[i];
            const P2& s = r[(i + 1) % n];
            const double e1x = q.x - p.x, e1y = q.y - p.y, e2x = s.x - q.x, e2y = s.y - q.y;
            const double cross = e1x * e2y - e1y * e2x, dot = e1x * e2x + e1y * e2y;
            if (cross >= -1e-12) continue;  // convex or straight
            const double l1 = std::hypot(e1x, e1y), l2 = std::hypot(e2x, e2y);
            const double turn = std::abs(std::atan2(cross, dot));
            const double ratio = l1 / l2;
            double radius;
            if (turn < 60 * kPi / 180 && ratio > 0.4 && ratio < 2.5) {
                // circumradius of the three points: a sampled arc
                const double chord = std::hypot(s.x - p.x, s.y - p.y);
                radius = l1 * l2 * chord / (2 * std::abs(cross));
            } else if (turn > 15 * kPi / 180) {
                radius = 0.0;  // a sharp corner
            } else {
                continue;
            }
            if (radius < best) {
                best = radius;
                if (where) *where = q;
            }
        }
    }
    return best;
}

double min_wall_thickness(const mesh::Region& region, P2* where) {
    struct Seg {
        P2 a, b;
        std::size_t ring, index;
    };
    const auto rings = oriented(region);
    std::vector<Seg> segs;
    for (std::size_t k = 0; k < rings.size(); ++k) {
        for (std::size_t i = 0; i < rings[k].size(); ++i) segs.push_back({rings[k][i], rings[k][(i + 1) % rings[k].size()], k, i});
    }
    if (segs.empty()) return kHuge;

    // Uniform grid of the segments; each ray only looks at the cells along its stretch up to the
    // best wall found so far, which shrinks quickly on parts with thin features.
    double lx = 1e300, ly = 1e300, hx = -1e300, hy = -1e300;
    for (const Seg& s : segs) {
        lx = std::min({lx, s.a.x, s.b.x});
        hx = std::max({hx, s.a.x, s.b.x});
        ly = std::min({ly, s.a.y, s.b.y});
        hy = std::max({hy, s.a.y, s.b.y});
    }
    const double extent = std::max(hx - lx, hy - ly);
    const double cell = std::max(extent / 150.0, 1e-4);
    auto cx = [&](double v) { return static_cast<long long>(std::floor((v - lx) / cell)); };
    auto cy = [&](double v) { return static_cast<long long>(std::floor((v - ly) / cell)); };
    std::unordered_map<long long, std::vector<int>> grid;
    auto key = [](long long i, long long j) { return i * 1000003LL + j; };
    for (std::size_t i = 0; i < segs.size(); ++i) {
        for (long long x = cx(std::min(segs[i].a.x, segs[i].b.x)); x <= cx(std::max(segs[i].a.x, segs[i].b.x)); ++x) {
            for (long long y = cy(std::min(segs[i].a.y, segs[i].b.y)); y <= cy(std::max(segs[i].a.y, segs[i].b.y)); ++y) grid[key(x, y)].push_back(static_cast<int>(i));
        }
    }
    std::vector<std::size_t> stamp(segs.size(), static_cast<std::size_t>(-1));

    double best = kHuge;
    for (std::size_t si = 0; si < segs.size(); ++si) {
        const Seg& s = segs[si];
        const double dx = s.b.x - s.a.x, dy = s.b.y - s.a.y, len = std::hypot(dx, dy);
        if (len < 1e-12) continue;
        const P2 mid{0.5 * (s.a.x + s.b.x), 0.5 * (s.a.y + s.b.y)};
        const double nx = -dy / len, ny = dx / len;  // left of the direction: into the material
        const double reach = std::min(best, extent * 1.5);
        const P2 far{mid.x + nx * reach, mid.y + ny * reach};
        double hit = kHuge;
        for (long long x = cx(std::min(mid.x, far.x)); x <= cx(std::max(mid.x, far.x)); ++x) {
            for (long long y = cy(std::min(mid.y, far.y)); y <= cy(std::max(mid.y, far.y)); ++y) {
                const auto it = grid.find(key(x, y));
                if (it == grid.end()) continue;
                for (int sj : it->second) {
                    if (stamp[static_cast<std::size_t>(sj)] == si || static_cast<std::size_t>(sj) == si) continue;
                    stamp[static_cast<std::size_t>(sj)] = si;
                    const Seg& o = segs[static_cast<std::size_t>(sj)];
                    if (o.ring == s.ring) {
                        const std::size_t n = rings[o.ring].size();
                        const std::size_t d = (o.index + n - s.index) % n;
                        if (d <= 1 || d >= n - 1) continue;  // neighbours
                    }
                    // ray mid + t n against segment o
                    const double ex = o.b.x - o.a.x, ey = o.b.y - o.a.y;
                    const double den = nx * ey - ny * ex;
                    if (std::abs(den) < 1e-14) continue;
                    const double t = ((o.a.x - mid.x) * ey - (o.a.y - mid.y) * ex) / den;
                    const double u = ((o.a.x - mid.x) * ny - (o.a.y - mid.y) * nx) / den;
                    if (t > 1e-9 && u >= 0 && u <= 1) hit = std::min(hit, t);
                }
            }
        }
        if (hit < best) {
            best = hit;
            if (where) *where = mid;
        }
    }
    return best;
}

namespace {

Finding make(const ProcessRules& p, const PartGeometry& g, const char* rule, Severity sev, std::string msg, std::string fix, double value, double limit) {
    Finding f;
    f.process = p.name;
    f.rule = rule;
    f.part = g.name;
    f.severity = sev;
    f.message = std::move(msg);
    f.fix = std::move(fix);
    f.value = value;
    f.limit = limit;
    return f;
}

double smallest_cutter(const ProcessRules& p) {
    const auto& c = p.list("cutters");
    return c.empty() ? 0.0 : *std::min_element(c.begin(), c.end());
}

}  // namespace

std::vector<Finding> run_dfm(const ProcessRules& p, const PartGeometry& g) {
    std::vector<Finding> out;
    auto has = [&](const char* r) { return std::find(p.rules.begin(), p.rules.end(), r) != p.rules.end(); };

    if (g.teeth > 0 && p.limit("skip_toothed", 0.0) > 0) {
        out.push_back(make(p, g, "applicability", Severity::Info, "toothed part: it is cut on a wheel-cutting engine, not by this process", "", 0, 0));
        return out;
    }

    if (has("internal_radius") && !g.region.rings.empty()) {
        const double tool = smallest_cutter(p);
        const double r_min = tool / 2;
        P2 at{};
        const double r = min_internal_radius(g.region, &at);
        if (r >= kHuge / 2) {
            out.push_back(make(p, g, "internal_radius", Severity::Info, "no internal corners", "", 0, r_min));
        } else if (r < r_min * (1 - 1e-6)) {
            Finding f = make(p, g, "internal_radius", Severity::Error,
                             "internal corner radius " + num(r, 3) + " mm is smaller than the smallest tool radius " + num(r_min, 3) + " mm",
                             "round internal corners to at least " + num(r_min, 3) + " mm, or open holes to at least " + num(tool, 3) + " mm diameter", r, r_min);
            f.has_location = true;
            f.location = at;
            out.push_back(f);
        } else {
            Finding f = make(p, g, "internal_radius", Severity::Info, "smallest internal radius " + num(r, 3) + " mm", "", r, r_min);
            f.has_location = true;
            f.location = at;
            out.push_back(f);
        }

        if (has("depth_to_diameter") && g.thickness > 0) {
            // The largest cutter that still fits the tightest internal feature does the deep cutting.
            double tool_used = tool;
            for (double c : p.list("cutters")) {
                if ((r >= kHuge / 2 || c <= 2 * r + 1e-9) && c > tool_used) tool_used = c;
            }
            if (tool_used > 0) {
                const double ratio = g.thickness / tool_used;
                const double limit = p.limit("max_depth_to_diameter", 4.0);
                if (ratio > limit) {
                    out.push_back(make(p, g, "depth_to_diameter", Severity::Warning,
                                       "depth " + num(g.thickness, 3) + " mm over a " + num(tool_used, 3) + " mm cutter is " + num(ratio, 1) + ":1 (limit " + num(limit, 1) + ":1)",
                                       "use a larger cutter, widen the tightest feature, or cut from both sides", ratio, limit));
                } else {
                    out.push_back(make(p, g, "depth_to_diameter", Severity::Info, "depth to cutter diameter " + num(ratio, 1) + ":1", "", ratio, limit));
                }
            }
        }
    }

    if (has("min_wall") && !g.region.rings.empty()) {
        const double limit = p.limit("min_wall", 0.25);
        P2 at{};
        const double w = min_wall_thickness(g.region, &at);
        if (w < limit * (1 - 1e-6)) {
            Finding f = make(p, g, "min_wall", Severity::Error, "wall of " + num(w, 3) + " mm is thinner than the " + num(limit, 3) + " mm this process holds",
                             "thicken the wall to " + num(limit, 3) + " mm or more", w, limit);
            f.has_location = true;
            f.location = at;
            out.push_back(f);
        } else {
            Finding f = make(p, g, "min_wall", Severity::Info, "thinnest wall " + (w >= kHuge / 2 ? std::string("none") : num(w, 3) + " mm"), "", w >= kHuge / 2 ? 0 : w, limit);
            f.has_location = w < kHuge / 2;
            f.location = at;
            out.push_back(f);
        }
    }

    if (g.teeth > 0) {
        if (has("module_cutter") && g.module > 0) {
            const double tol = p.limit("module_tolerance", 0.02);
            double best = 0, gap = kHuge;
            for (double m : p.list("cutter_modules")) {
                if (std::abs(m - g.module) / g.module < gap) {
                    gap = std::abs(m - g.module) / g.module;
                    best = m;
                }
            }
            if (gap > tol) {
                out.push_back(make(p, g, "module_cutter", Severity::Error,
                                   "module " + num(g.module, 4) + " has no cutter within " + num(tol * 100, 1) + " % (nearest " + num(best, 3) + ", " + num(gap * 100, 1) + " % away)",
                                   "change the module to " + num(best, 3) + " or buy a cutter for " + num(g.module, 4), gap, tol));
            } else {
                out.push_back(make(p, g, "module_cutter", Severity::Info, "cutter module " + num(best, 3) + " fits module " + num(g.module, 4), "", gap, tol));
            }
        }
        if (has("min_teeth")) {
            const double limit = p.limit("min_teeth", 8);
            if (g.teeth < limit) {
                out.push_back(make(p, g, "min_teeth", Severity::Error, std::to_string(g.teeth) + " teeth is fewer than the " + num(limit, 0) + " this process cuts",
                                   "use at least " + num(limit, 0) + " teeth", g.teeth, limit));
            } else {
                out.push_back(make(p, g, "min_teeth", Severity::Info, std::to_string(g.teeth) + " teeth", "", g.teeth, limit));
            }
        }
        if (has("undercut") && g.rolling_radius > 0) {
            // A hypocycloid dedendum undercuts when its rolling circle is larger than half the pitch radius.
            const double zmin = 4.0 * g.rolling_radius;
            if (g.teeth < zmin) {
                out.push_back(make(p, g, "undercut", Severity::Warning,
                                   std::to_string(g.teeth) + " cycloidal teeth undercut at the root below " + num(std::ceil(zmin), 0) + " teeth for a rolling circle of " + num(g.rolling_radius, 2) + " modules",
                                   "use a smaller rolling circle (the meshing wheel must match), raise the tooth count, or accept the weaker root", g.teeth, zmin));
            } else {
                out.push_back(make(p, g, "undercut", Severity::Info, "no undercut above " + num(std::ceil(zmin), 0) + " teeth", "", g.teeth, zmin));
            }
        } else if (has("undercut")) {
            const double a = g.pressure_angle_deg * kPi / 180;
            const double zmin = 2.0 / (std::sin(a) * std::sin(a));
            if (g.teeth < zmin) {
                out.push_back(make(p, g, "undercut", Severity::Warning,
                                   std::to_string(g.teeth) + " involute teeth undercut at the root below " + num(std::ceil(zmin), 0) + " teeth for a " + num(g.pressure_angle_deg, 1) + " degree profile",
                                   "profile-shift the wheel, raise the tooth count, or accept the weaker root", g.teeth, zmin));
            } else {
                out.push_back(make(p, g, "undercut", Severity::Info, "no undercut above " + num(std::ceil(zmin), 0) + " teeth", "", g.teeth, zmin));
            }
        }
        if (has("pinion_leaves") && g.role == model::Role::Pinion) {
            const double limit = p.limit("min_pinion_leaves", 6);
            if (g.teeth < limit) {
                out.push_back(make(p, g, "pinion_leaves", Severity::Error, std::to_string(g.teeth) + " leaves is fewer than the " + num(limit, 0) + " a pinion needs",
                                   "use at least " + num(limit, 0) + " leaves", g.teeth, limit));
            } else {
                out.push_back(make(p, g, "pinion_leaves", Severity::Info, std::to_string(g.teeth) + " leaves", "", g.teeth, limit));
            }
        }
    }

    if (has("jewel_fit") && g.region.rings.size() > 1) {
        const double window = p.limit("match_window", 0.15), lo = p.limit("interference_min", 0.004), hi = p.limit("interference_max", 0.020);
        const auto rings = oriented(g.region);
        for (std::size_t k = 1; k < rings.size(); ++k) {
            const double d_hole = 2 * std::sqrt(std::abs(signed_area(rings[k])) / kPi);
            double nearest = 0, gap = kHuge;
            for (double j : p.list("jewel_diameters")) {
                if (std::abs(j - d_hole) < gap) {
                    gap = std::abs(j - d_hole);
                    nearest = j;
                }
            }
            if (gap > window) continue;  // not a jewel seat
            const double interference = nearest - d_hole;
            double cx = 0, cy = 0;
            for (const P2& q : rings[k]) {
                cx += q.x;
                cy += q.y;
            }
            const P2 centre{cx / static_cast<double>(rings[k].size()), cy / static_cast<double>(rings[k].size())};
            Finding f;
            if (interference < lo - 1e-9 || interference > hi + 1e-9) {  // a hair of slack: the window edges are inclusive
                f = make(p, g, "jewel_fit", Severity::Error,
                         "hole of " + num(d_hole, 4) + " mm for a " + num(nearest, 2) + " mm jewel gives " + num(interference * 1000, 1) + " um interference (want " + num(lo * 1000, 0) + " to " + num(hi * 1000, 0) + " um)",
                         "ream the seat to " + num(nearest - 0.5 * (lo + hi), 4) + " mm", interference, interference < lo ? lo : hi);
            } else {
                f = make(p, g, "jewel_fit", Severity::Info, "jewel seat " + num(d_hole, 4) + " mm for a " + num(nearest, 2) + " mm jewel: " + num(interference * 1000, 1) + " um interference", "", interference, lo);
            }
            f.has_location = true;
            f.location = centre;
            out.push_back(f);
        }
    }
    return out;
}

}  // namespace ohs::manufacturing
