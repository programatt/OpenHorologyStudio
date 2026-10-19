// SPDX-License-Identifier: Apache-2.0
#include "geometry/features.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Bnd_OBB.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Cylinder.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include "geometry/profile.hpp"

namespace ohs::geometry {

TopoDS_Shape orient_dial_axis_to_z(const TopoDS_Shape& shape, Axis axis) {
    gp_Trsf t;
    switch (axis) {
        case Axis::Z: return shape;
        case Axis::Y: t.SetRotation(gp_Ax1(gp::Origin(), gp::DX()), std::numbers::pi / 2); break;
        case Axis::X: t.SetRotation(gp_Ax1(gp::Origin(), gp::DY()), -std::numbers::pi / 2); break;
    }
    return shape.Moved(TopLoc_Location(t));
}

namespace {

std::uint64_t fnv1a(std::uint64_t h, long long v) {
    for (int i = 0; i < 8; ++i) {
        h ^= static_cast<std::uint64_t>((v >> (8 * i)) & 0xff);
        h *= 1099511628211ULL;
    }
    return h;
}

// Quantised to 1 um so the fingerprint survives a CAD round trip.
std::uint64_t fingerprint(double volume, const double (&box)[6]) {
    std::uint64_t h = 14695981039346656037ULL;
    h = fnv1a(h, std::llround(volume * 1e3));
    for (double v : box) h = fnv1a(h, std::llround(v * 1e3));
    return h;
}

struct Outline {
    std::vector<PolarPoint> points;
    std::vector<double> outer;  ///< max radius per angular bin, gaps filled
    std::vector<double> inner;  ///< min radius per angular bin, gaps filled (the inside of a ring)
    double min_radius{};        ///< smallest radius of any sampled point (bore if present)
    bool ok{false};
};

Outline sample_outline(const TopoDS_Shape& shape, double z, double cx, double cy, const FeatureOptions& o) {
    Outline out;
    BRepAlgoAPI_Section section(shape, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)));
    if (!section.IsDone()) return out;

    const int n = o.angular_bins;
    std::vector<double> bins(n, -1.0);
    std::vector<double> lows(n, -1.0);
    double min_r = 1e300;
    auto add = [&](const gp_Pnt& p) {
        const double dx = p.X() - cx;
        const double dy = p.Y() - cy;
        const double r = std::hypot(dx, dy);
        const double a = std::atan2(dy, dx) + std::numbers::pi;
        const int bin = std::min(n - 1, static_cast<int>(a / (2 * std::numbers::pi) * n));
        bins[bin] = std::max(bins[bin], r);
        lows[bin] = lows[bin] < 0 ? r : std::min(lows[bin], r);
        min_r = std::min(min_r, r);
        out.points.push_back({r, a});
    };

    for (TopExp_Explorer ex(section.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) {
        BRepAdaptor_Curve curve(TopoDS::Edge(ex.Current()));
        // Parameter-uniform samples; the count follows a coarse length estimate so that small
        // closed edges (an arbor's circle) still cover every angular bin.
        const double u0 = curve.FirstParameter();
        const double u1 = curve.LastParameter();
        double length = 0.0;
        gp_Pnt previous = curve.Value(u0);
        constexpr int kProbe = 16;
        for (int i = 1; i <= kProbe; ++i) {
            const gp_Pnt next = curve.Value(u0 + (u1 - u0) * i / kProbe);
            length += previous.Distance(next);
            previous = next;
        }
        const int count = std::clamp(static_cast<int>(std::ceil(length / std::min(o.sample_step, length / (2.0 * n)))), 2, 200000);
        for (int i = 0; i <= count; ++i) add(curve.Value(u0 + (u1 - u0) * i / count));
    }

    // Fill empty bins from the nearest filled neighbours (circular, linear in index).
    const int filled = static_cast<int>(std::count_if(bins.begin(), bins.end(), [](double r) { return r >= 0; }));
    if (filled < n / 2) return out;
    auto fill = [n](const std::vector<double>& sparse) {
        std::vector<double> full = sparse;
        for (int i = 0; i < n; ++i) {
            if (sparse[i] >= 0) continue;
            int lo = 1;
            while (sparse[(i - lo + n) % n] < 0) ++lo;
            int hi = 1;
            while (sparse[(i + hi) % n] < 0) ++hi;
            const double a = sparse[(i - lo + n) % n];
            const double b = sparse[(i + hi) % n];
            full[i] = a + (b - a) * lo / (lo + hi);
        }
        return full;
    };
    out.outer = fill(bins);
    out.inner = fill(lows);
    out.min_radius = min_r;
    out.ok = true;
    return out;
}

// The oriented bounding box: the direction of its longest side and the three sides, longest first.
void fit_box(const TopoDS_Shape& shape, model::PartFeatures& f) {
    Bnd_OBB obb;
    BRepBndLib::AddOBB(shape, obb, true, false, false);
    if (obb.IsVoid()) return;
    const gp_Dir dirs[3] = {obb.XDirection(), obb.YDirection(), obb.ZDirection()};
    const double sides[3] = {2 * obb.XHSize(), 2 * obb.YHSize(), 2 * obb.ZHSize()};
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int a, int b) { return sides[a] > sides[b]; });
    const gp_Dir& d = dirs[order[0]];
    // Either way along the axis is the same line: point it towards +X (then +Y) so a rig is reproducible.
    const double sign = d.X() > 1e-9 ? 1.0 : d.X() < -1e-9 ? -1.0 : (d.Y() >= 0 ? 1.0 : -1.0);
    f.axis[0] = sign * d.X();
    f.axis[1] = sign * d.Y();
    f.axis[2] = sign * d.Z();
    f.length = sides[order[0]];
    f.width = sides[order[1]];
    f.height = sides[order[2]];
}

// Round holes along Z: the cylindrical faces whose material is on the outside, merged where a hole is
// split over several faces or runs through a stack of them.
void find_holes(const TopoDS_Shape& shape, model::PartFeatures& f) {
    constexpr double kMerge = 1e-4;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Cylinder) continue;
        const gp_Cylinder cyl = surface.Cylinder();
        if (std::abs(cyl.Axis().Direction().Z()) < 0.9999) continue;
        const double r = cyl.Radius();
        const gp_Pnt a = cyl.Location();
        // A point of the face and one a little towards the axis: a hole has no material there.
        const gp_Pnt p = surface.Value(0.5 * (surface.FirstUParameter() + surface.LastUParameter()),
                                       0.5 * (surface.FirstVParameter() + surface.LastVParameter()));
        const gp_Vec radial(gp_Pnt(a.X(), a.Y(), p.Z()), p);
        if (radial.Magnitude() < 1e-12) continue;
        const gp_Pnt inside = p.Translated(-0.2 * r * radial.Normalized());
        BRepClass3d_SolidClassifier classifier(shape, inside, 1e-7);
        if (classifier.State() != TopAbs_OUT) continue;
        bool known = false;
        for (const model::Hole& h : f.holes) {
            known |= std::hypot(h.x - a.X(), h.y - a.Y()) < kMerge && std::abs(h.radius - r) < kMerge;
        }
        if (!known) f.holes.push_back({a.X(), a.Y(), r});
    }
    std::sort(f.holes.begin(), f.holes.end(),
              [](const model::Hole& a, const model::Hole& b) { return a.radius > b.radius; });
    if (f.holes.size() > 8) f.holes.resize(8);
}

}  // namespace

model::PartFeatures extract_features(const ImportedPart& part, const FeatureOptions& options) {
    model::PartFeatures f;
    f.name = part.name;
    const TopoDS_Shape shape = orient_dial_axis_to_z(part.shape, options.dial_axis);

    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    f.volume = std::abs(props.Mass());
    if (f.volume < 1e-12) return f;

    const gp_Pnt c = props.CentreOfMass();
    f.center = {c.X(), c.Y()};

    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    double b[6];
    box.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
    f.z_min = b[2];
    f.z_max = b[5];
    f.extent_x = b[3] - b[0];
    f.extent_y = b[4] - b[1];
    f.fingerprint = fingerprint(f.volume, b);
    fit_box(shape, f);
    find_holes(shape, f);

    const Outline outline = sample_outline(shape, 0.5 * (f.z_min + f.z_max), c.X(), c.Y(), options);
    if (!outline.ok) return f;

    f.outer_radius = *std::max_element(outline.outer.begin(), outline.outer.end());
    const double root = *std::min_element(outline.outer.begin(), outline.outer.end());
    if (outline.min_radius < 0.9 * root) f.bore_radius = outline.min_radius;

    const ToothResult teeth = detect_teeth(outline.outer, options.min_teeth);
    f.teeth = teeth.teeth;
    f.tooth_confidence = teeth.confidence;
    if (f.teeth > 0) {
        f.root_radius = root;
        const double phase = tooth_phase(outline.outer, f.teeth);
        const FlankFit fit = fit_involute_flank(outline.points, f.teeth, phase, root, f.outer_radius);
        if (fit.ok) {
            f.flank_base_radius = fit.base_radius;
            f.flank_rms = fit.rms;
        }
        // A flank the involute does not explain (more than about 0.02 module off) may be cycloidal.
        const double depth = f.outer_radius - root;
        if (!fit.ok || fit.rms > 0.008 * depth) {
            const CycloidFit cyc = fit_cycloidal_flank(outline.points, f.teeth, phase, root, f.outer_radius);
            if (cyc.ok) {
                f.flank_cycloid_rms = cyc.rms;
                f.flank_cycloid_pitch_radius = cyc.pitch_radius;
                f.flank_cycloid_rolling_radius = cyc.rolling_radius;
            }
        }
    }

    // A ring (a date wheel) has its teeth on the inside: the smallest radius per angle, wherever the
    // inner loop is. Only parts that are mostly hole are looked at, so a wheel's spokes and
    // crossings do not pass for teeth.
    if (outline.min_radius > 0.5 * f.outer_radius) {
        const ToothResult inner = detect_teeth(outline.inner, options.min_teeth);
        if (inner.teeth > 0) {
            f.inner_teeth = inner.teeth;
            f.inner_tip_radius = *std::min_element(outline.inner.begin(), outline.inner.end());
            f.inner_root_radius = *std::max_element(outline.inner.begin(), outline.inner.end());
        }
    }
    return f;
}

const char* axis_name(Axis axis) {
    return axis == Axis::X ? "x" : axis == Axis::Y ? "y" : "z";
}

Axis detect_dial_axis(const std::vector<ImportedPart>& parts) {
    std::vector<double> extent[3];
    for (const auto& part : parts) {
        Bnd_Box box;
        BRepBndLib::AddOptimal(part.shape, box, false, false);
        if (box.IsVoid()) continue;
        double lo[3], hi[3];
        box.Get(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
        for (int a = 0; a < 3; ++a) extent[a].push_back(hi[a] - lo[a]);
    }
    if (extent[0].empty()) return Axis::Z;
    double median[3];
    for (int a = 0; a < 3; ++a) {
        auto mid = extent[a].begin() + static_cast<std::ptrdiff_t>(extent[a].size() / 2);
        std::nth_element(extent[a].begin(), mid, extent[a].end());
        median[a] = *mid;
    }
    // Z wins ties and near ties, so a normally oriented CAD file is never reinterpreted.
    int best = 2;
    for (int a : {0, 1}) {
        if (median[a] < 0.9 * median[best]) best = a;
    }
    return best == 0 ? Axis::X : best == 1 ? Axis::Y : Axis::Z;
}

}  // namespace ohs::geometry
