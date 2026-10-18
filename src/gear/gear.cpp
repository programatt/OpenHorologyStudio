// SPDX-License-Identifier: Apache-2.0
#include "gear/gear.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "core/error.hpp"

namespace ohs::gear {

namespace {

using mesh::P2;
constexpr double kPi = std::numbers::pi;

double inv(double a) { return std::tan(a) - a; }

P2 rotate(const P2& p, double a) { return {p.x * std::cos(a) - p.y * std::sin(a), p.x * std::sin(a) + p.y * std::cos(a)}; }

P2 polar(double r, double a) { return {r * std::cos(a), r * std::sin(a)}; }

void validate(const GearSpec& s) {
    if (s.teeth < 6) throw Error("a gear needs at least 6 teeth");
    if (s.module <= 0) throw Error("the module must be positive");
    if (s.pressure_angle_deg <= 0 || s.pressure_angle_deg >= 45) throw Error("the pressure angle must be between 0 and 45 degrees");
    if (s.root_radius() <= s.bore_radius) throw Error("the bore reaches the tooth roots");
}

/// One tooth, centred on angle 0: right-hand root tangent point, fillet, flank to the tip.
std::vector<P2> right_side(const GearSpec& s) {
    const double rp = s.pitch_radius(), rb = s.base_radius(), rt = s.tip_radius(), rr = s.root_radius();
    if (rt <= rb) throw Error("the tip circle lies inside the base circle");
    auto phi = [&](double r) { return kPi / (2 * s.teeth) + inv(s.pressure_angle_deg * kPi / 180) - inv(std::acos(std::min(rb / r, 1.0))); };
    if (phi(rt) <= 0) throw Error("the teeth are pointed: reduce the addendum or the pressure angle");
    // Below the base circle the "flank" runs radially at the angle where the involute starts.
    auto flank = [&](double r) { return polar(r, -phi(std::max(r, rb))); };

    const double lo = rr + 1e-6 * s.module;
    std::vector<P2> pts;
    double r_start = lo;
    bool filleted = false;
    const double rho = s.root_fillet * s.module;
    if (rho > 0) {
        auto centre_gap = [&](double r, P2& C) {
            const double d = 1e-5 * s.module;
            const P2 a = flank(r - d), b = flank(r + d);
            const double tx = b.x - a.x, ty = b.y - a.y, tl = std::hypot(tx, ty);
            const P2 T = flank(r);
            C = {T.x + rho * ty / tl, T.y - rho * tx / tl};
            return std::hypot(C.x, C.y) - (rr + rho);
        };
        P2 C{};
        const int n = 80;
        double prev_r = lo + 1e-6 * s.module, prev_g = centre_gap(prev_r, C);
        for (int i = 1; i <= n && !filleted; ++i) {
            const double r = lo + (rp - lo) * i / n;
            const double g = centre_gap(r, C);
            if (prev_g * g <= 0) {
                double a = prev_r, b = r, ga = prev_g;
                for (int k = 0; k < 60; ++k) {
                    const double m = 0.5 * (a + b);
                    const double gm = centre_gap(m, C);
                    if (ga * gm <= 0) b = m;
                    else {
                        a = m;
                        ga = gm;
                    }
                }
                r_start = 0.5 * (a + b);
                centre_gap(r_start, C);
                const P2 T = flank(r_start);
                const double clen = std::hypot(C.x, C.y);
                const P2 R{C.x * rr / clen, C.y * rr / clen};
                double a0 = std::atan2(R.y - C.y, R.x - C.x), a1 = std::atan2(T.y - C.y, T.x - C.x);
                while (a1 - a0 > kPi) a1 -= 2 * kPi;
                while (a1 - a0 < -kPi) a1 += 2 * kPi;
                for (int j = 0; j <= s.points_per_fillet; ++j) {
                    const double a_j = a0 + (a1 - a0) * j / s.points_per_fillet;
                    pts.push_back({C.x + rho * std::cos(a_j), C.y + rho * std::sin(a_j)});
                }
                filleted = true;
            }
            prev_r = r;
            prev_g = g;
        }
    }
    if (!filleted) {
        r_start = rr;
        pts.push_back(flank(rr));
    }
    // The corner where the radial run-out meets the involute at the base circle.
    if (r_start < rb - 1e-9) pts.push_back(flank(rb));
    const double r_from = std::max(r_start, rb);
    for (int j = 1; j <= s.points_per_flank; ++j) {
        // Denser near the root where the curvature is highest.
        const double t = static_cast<double>(j) / s.points_per_flank;
        pts.push_back(flank(r_from + (rt - r_from) * (t * t * (3 - 2 * t) * 0.5 + t * 0.5)));
    }
    return pts;
}

std::vector<P2> tooth(const GearSpec& s, const std::vector<P2>& right) {
    std::vector<P2> pts = right;
    const P2 tip = right.back();
    const double a_tip = std::atan2(tip.y, tip.x);
    for (int k = 1; k <= 3; ++k) pts.push_back(polar(s.tip_radius(), a_tip * (1 - 2.0 * k / 4)));
    for (std::size_t i = right.size(); i-- > 0;) pts.push_back({right[i].x, -right[i].y});
    return pts;
}

}  // namespace

double GearSpec::base_radius() const { return pitch_radius() * std::cos(pressure_angle_deg * kPi / 180); }

double half_tooth_angle(const GearSpec& s, double r) {
    const double rb = s.base_radius();
    return kPi / (2 * s.teeth) + inv(s.pressure_angle_deg * kPi / 180) - inv(std::acos(std::min(rb / r, 1.0)));
}

namespace {

std::vector<P2> teeth_run(const GearSpec& s, int first, int last) {
    const auto right = right_side(s);
    const auto one = tooth(s, right);
    const double pitch = 2 * kPi / s.teeth;
    std::vector<P2> pts;
    for (int k = first; k <= last; ++k) {
        for (const P2& p : one) pts.push_back(rotate(p, k * pitch));
        if (k != last) {
            // root arc to the next tooth
            const P2 from = rotate(one.back(), k * pitch), to = rotate(one.front(), (k + 1) * pitch);
            const double a0 = std::atan2(from.y, from.x);
            double a1 = std::atan2(to.y, to.x);
            while (a1 < a0) a1 += 2 * kPi;
            const int steps = std::max(1, static_cast<int>(std::ceil((a1 - a0) / (pitch * 0.2))));
            for (int j = 1; j < steps; ++j) pts.push_back(polar(s.root_radius(), a0 + (a1 - a0) * j / steps));
        }
    }
    return pts;
}

}  // namespace

mesh::Ring outline(const GearSpec& s) {
    validate(s);
    mesh::Ring ring = teeth_run(s, 0, s.teeth - 1);
    // close with the root arc from the last tooth to the first
    const auto right = right_side(s);
    const auto one = tooth(s, right);
    const double pitch = 2 * kPi / s.teeth;
    const P2 from = rotate(one.back(), (s.teeth - 1) * pitch), to = rotate(one.front(), s.teeth * pitch);
    const double a0 = std::atan2(from.y, from.x);
    double a1 = std::atan2(to.y, to.x);
    while (a1 < a0) a1 += 2 * kPi;
    const int steps = std::max(1, static_cast<int>(std::ceil((a1 - a0) / (pitch * 0.2))));
    for (int j = 1; j < steps; ++j) ring.push_back(polar(s.root_radius(), a0 + (a1 - a0) * j / steps));
    return ring;
}

mesh::Region wheel_region(const GearSpec& s) {
    mesh::Region region;
    region.rings.push_back(outline(s));
    if (s.bore_radius > 0) {
        mesh::Ring hole;
        const int n = std::max(48, s.teeth * 4);
        for (int i = 0; i < n; ++i) hole.push_back(polar(s.bore_radius, 2 * kPi * i / n));
        region.rings.push_back(hole);
    }
    return region;
}

mesh::Region sector_region(const GearSpec& s, int tooth_count, double rim_radius) {
    validate(s);
    if (tooth_count < 1 || tooth_count % 2 == 0) throw Error("a tooth sector needs an odd number of teeth");
    if (rim_radius <= 0 || rim_radius >= s.root_radius()) throw Error("the sector rim must lie inside the root circle");
    const int half = (tooth_count - 1) / 2;
    mesh::Ring ring = teeth_run(s, -half, half);
    const P2 start = ring.front(), end = ring.back();
    const double a_end = std::atan2(end.y, end.x), a_start = std::atan2(start.y, start.x);
    ring.push_back(polar(rim_radius, a_end));
    const int steps = 12;
    for (int j = 1; j < steps; ++j) ring.push_back(polar(rim_radius, a_end + (a_start - a_end) * j / steps));
    ring.push_back(polar(rim_radius, a_start));
    return {{ring}};
}

double lewis_form_factor(int teeth) {
    static const double table[][2] = {{12, 0.245}, {13, 0.261}, {14, 0.277}, {15, 0.290}, {16, 0.296}, {17, 0.303},
                                      {18, 0.309}, {19, 0.314}, {20, 0.322}, {21, 0.328}, {22, 0.331}, {24, 0.337},
                                      {26, 0.346}, {28, 0.353}, {30, 0.359}, {34, 0.371}, {38, 0.384}, {45, 0.399},
                                      {50, 0.408}, {60, 0.421}, {75, 0.434}, {100, 0.446}, {150, 0.459}, {300, 0.471},
                                      {1000, 0.484}};
    const int n = static_cast<int>(sizeof(table) / sizeof(table[0]));
    if (teeth <= table[0][0]) return table[0][1];
    for (int i = 1; i < n; ++i) {
        if (teeth <= table[i][0]) {
            const double x = (teeth - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
            return table[i - 1][1] + x * (table[i][1] - table[i - 1][1]);
        }
    }
    return table[n - 1][1];
}

double lewis_stress(const GearSpec& s, double wt, double face_width) {
    return wt / (face_width * s.module * lewis_form_factor(s.teeth));
}

}  // namespace ohs::gear
