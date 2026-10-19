// SPDX-License-Identifier: Apache-2.0
#include "model/followers.hpp"

#include <algorithm>
#include <cmath>

namespace ohs::model {

namespace {

// How far the estimated end of a part may be from the band of the teeth and still be on them, mm.
constexpr double kReach = 0.35;
// The worst fit (see band_fit) that still counts as resting on the teeth: a tip right at the edge of the reach
// could as well be next to them.
constexpr double kMaxFit = 0.85;
// Parts closer than this in height touch (a gap of a few hundredths of a millimetre is a clearance).
constexpr double kHeightGap = 0.05;
// A hole that another part is centred in, to this tolerance, has a pin through it.
constexpr double kPinTolerance = 0.15;
constexpr double kMinHole = 0.1;
constexpr double kMaxHole = 1.5;

double distance(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

double height_overlap(const PartFeatures& a, const PartFeatures& b) {
    return std::min(a.z_max, b.z_max) - std::max(a.z_min, b.z_min);
}

// Distance from a point to the segment a-b; `nearest` is the point of the segment closest to it.
double segment_distance(Vec2 p, Vec2 a, Vec2 b, Vec2& nearest) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double t = len2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    nearest = {a.x + t * dx, a.y + t * dy};
    return distance(p, nearest);
}

// How well a point at `radius` from a toothed part's centre sits in its band: 0 in the middle, 1 at the
// limit of the reach, more than 1 off the teeth.
double band_fit(const ToothBand& band, double radius) {
    const double half = 0.5 * band.depth() + kReach;
    return std::abs(radius - 0.5 * (band.lo + band.hi)) / half;
}

// +1 where the teeth push a follower away from the centre of their part, -1 where they push it towards it.
double push_direction(const Component& c) { return c.features.inner_teeth >= 6 && c.features.inner_root_radius > c.features.inner_tip_radius ? -1.0 : 1.0; }

bool occupied(const std::vector<Component>& components, int self, int driver, Vec2 hole) {
    for (std::size_t i = 0; i < components.size(); ++i) {
        const int j = static_cast<int>(i);
        if (j == self || j == driver) continue;
        if (distance(components[i].features.center, hole) < kPinTolerance) return true;
    }
    return false;
}

}  // namespace

bool is_follow_target(const Component& c) {
    if (!is_rotating_role(c.role)) return false;
    const PartFeatures& f = c.features;
    if (f.inner_teeth >= 6 && f.inner_root_radius > f.inner_tip_radius) return true;
    return f.teeth >= 6 && f.tooth_confidence >= 0.4 && f.outer_radius > f.root_radius;
}

ToothBand tooth_band(const Component& c) {
    const PartFeatures& f = c.features;
    if (f.inner_teeth >= 6 && f.inner_root_radius > f.inner_tip_radius) return {f.inner_tip_radius, f.inner_root_radius};
    return {f.root_radius, f.outer_radius};
}

std::optional<LeverContact> lever_contact(const std::vector<Component>& components, int index, const Usable& usable) {
    const PartFeatures& f = components[static_cast<std::size_t>(index)].features;
    if (f.length <= 0.0) return std::nullopt;
    std::optional<LeverContact> best;
    double best_score = 1e300;
    for (const Hole& h : f.holes) {
        if (h.radius < kMinHole || h.radius > kMaxHole) continue;
        const Vec2 pivot{h.x, h.y};
        // The part reaches from the pivot through its centre to its far end, as far past the centre again as
        // the centre is from the middle of the part: taken as half the length.
        const double d = distance(f.center, pivot);
        if (d < 0.1) continue;
        const double arm = d + 0.5 * f.length;
        const Vec2 tip{pivot.x + (f.center.x - pivot.x) / d * arm, pivot.y + (f.center.y - pivot.y) / d * arm};
        for (std::size_t i = 0; i < components.size(); ++i) {
            const int j = static_cast<int>(i);
            if (j == index || !is_follow_target(components[i]) || (usable && !usable(j))) continue;
            if (height_overlap(f, components[i].features) < -kHeightGap) continue;
            const ToothBand band = tooth_band(components[i]);
            const double fit = band_fit(band, distance(tip, components[i].features.center));
            if (fit > kMaxFit) continue;
            // The pivot cannot be in the middle of the wheel it rides on.
            if (distance(pivot, components[i].features.center) < 0.5 * band.lo) continue;
            const double score = occupied(components, index, j, pivot) ? 0.5 * fit : fit;
            if (score < best_score) {
                best_score = score;
                best = LeverContact{j, pivot, tip, arm, fit};
            }
        }
    }
    return best;
}

double lever_swing(const std::vector<Component>& components, const LeverContact& contact) {
    const Component& driver = components[static_cast<std::size_t>(contact.driver)];
    const Vec2 c = driver.features.center;
    const double dist = distance(contact.tip, c);
    if (dist <= 0.0 || contact.arm <= 0.0) return 0.0;
    // The tip moves along the perpendicular to the arm when the lever turns counter-clockwise.
    const double rx = contact.tip.x - contact.pivot.x, ry = contact.tip.y - contact.pivot.y;
    const double along = (-ry * (contact.tip.x - c.x) + rx * (contact.tip.y - c.y)) / (contact.arm * dist);
    const double push = tooth_band(driver).depth() / contact.arm;
    return push_direction(driver) * (along >= 0.0 ? push : -push);
}

std::optional<SpringFlex> spring_flex(const std::vector<Component>& components, int index, const Usable& usable) {
    const PartFeatures& f = components[static_cast<std::size_t>(index)].features;
    const double along = std::hypot(f.axis[0], f.axis[1]);
    // Bent wire standing up from the plate has no end in the plane that can be told from its bend.
    if (f.length <= 0.0 || along < 0.7) return std::nullopt;
    const double half = 0.5 * f.length * along;
    const Vec2 u{f.axis[0] / along, f.axis[1] / along};
    const Vec2 ends[2] = {{f.center.x - u.x * half, f.center.y - u.y * half}, {f.center.x + u.x * half, f.center.y + u.y * half}};

    std::optional<SpringFlex> best;
    double best_score = 1e300;
    for (int e = 0; e < 2; ++e) {
        const Vec2 free_end = ends[e], fixed = ends[1 - e];
        const double length = distance(free_end, fixed);
        if (length < 0.5) continue;
        const double sx = free_end.x - fixed.x, sy = free_end.y - fixed.y;
        // Pressing on a lever that moves: the end is on the lever, between its pivot and its tip.
        for (std::size_t i = 0; i < components.size(); ++i) {
            const int j = static_cast<int>(i);
            if (j == index || components[i].role != Role::Lever) continue;
            const auto contact = lever_contact(components, j, usable);
            if (!contact) continue;
            const PartFeatures& lf = components[i].features;
            if (height_overlap(f, lf) < -kHeightGap) continue;
            Vec2 on_lever;
            const double gap = segment_distance(free_end, contact->pivot, contact->tip, on_lever);
            const double reach = 0.5 * lf.width + 0.5;
            if (gap > reach || gap / reach >= best_score) continue;
            best_score = gap / reach;
            // The lever turns by depth / arm about its pivot and carries the point of the spring with it.
            const double angle = lever_swing(components, *contact);
            const Vec2 moved{-angle * (on_lever.y - contact->pivot.y), angle * (on_lever.x - contact->pivot.x)};
            SpringFlex flex;
            flex.driver = contact->driver;
            flex.lever = j;
            flex.fixed = fixed;
            flex.free_end = free_end;
            flex.length = length;
            flex.depth = std::hypot(moved.x, moved.y);
            flex.swing = (-sy * moved.x + sx * moved.y) / (length * length);
            best = flex;
        }
        // Pressing on the teeth itself: the end is the tip.
        for (std::size_t i = 0; i < components.size(); ++i) {
            const int j = static_cast<int>(i);
            if (j == index || !is_follow_target(components[i]) || (usable && !usable(j))) continue;
            if (height_overlap(f, components[i].features) < -kHeightGap) continue;
            const ToothBand band = tooth_band(components[i]);
            const Vec2 c = components[i].features.center;
            const double dist = distance(free_end, c);
            const double fit = band_fit(band, dist);
            if (fit > kMaxFit || fit >= best_score || dist <= 0.0) continue;
            best_score = fit;
            SpringFlex flex;
            flex.driver = j;
            flex.fixed = fixed;
            flex.free_end = free_end;
            flex.length = length;
            flex.depth = band.depth();
            // Pushed away from the centre of the part it rides on.
            const double push = push_direction(components[i]);
            const Vec2 out{push * (free_end.x - c.x) / dist, push * (free_end.y - c.y) / dist};
            flex.swing = band.depth() * (-sy * out.x + sx * out.y) / (length * length);
            best = flex;
        }
    }
    return best;
}

}  // namespace ohs::model
