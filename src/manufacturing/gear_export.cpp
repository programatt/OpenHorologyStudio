// SPDX-License-Identifier: Apache-2.0
#include "manufacturing/gear_export.hpp"

#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>

#include "core/error.hpp"
#include "geometry/step_io.hpp"

namespace ohs::manufacturing {

namespace {

gear::GearSpec fine(gear::GearSpec s) {
    s.points_per_flank = std::max(s.points_per_flank, 48);
    s.points_per_fillet = std::max(s.points_per_fillet, 16);
    return s;
}

std::string num(double v, int prec = 3) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

}  // namespace

TopoDS_Shape gear_solid(const gear::GearSpec& spec, double thickness) {
    if (thickness <= 0) throw Error("the wheel needs a positive thickness");
    gear::outline(spec);  // validates the specification with the usual messages

    // Exact geometry: per tooth two cubic splines through the flank points (fillet and involute
    // together, tangent-continuous) and circular arcs for the tip and the root, a few hundred edges
    // for a whole wheel instead of thousands of straight segments.
    const mesh::Ring flank = gear::tooth_flank(fine(spec));
    const double pitch = 2 * std::numbers::pi / spec.teeth;
    auto point = [](const mesh::P2& p, double turn, bool mirror) {
        const double y = mirror ? -p.y : p.y;
        return gp_Pnt(p.x * std::cos(turn) - y * std::sin(turn), p.x * std::sin(turn) + y * std::cos(turn), 0);
    };
    auto spline = [&](double turn, bool mirror) {
        Handle(TColgp_HArray1OfPnt) pts = new TColgp_HArray1OfPnt(1, static_cast<int>(flank.size()));
        for (std::size_t i = 0; i < flank.size(); ++i) {
            const std::size_t k = mirror ? flank.size() - 1 - i : i;  // the left side runs from the tip down to the root
            pts->SetValue(static_cast<int>(i) + 1, point(flank[k], turn, mirror));
        }
        GeomAPI_Interpolate interpolate(pts, false, 1e-7);
        interpolate.Perform();
        if (!interpolate.IsDone()) throw Error("the tooth flank could not be interpolated");
        return BRepBuilderAPI_MakeEdge(interpolate.Curve()).Edge();
    };
    auto arc = [&](const gp_Pnt& a, double mid_radius, double mid_angle, const gp_Pnt& b) {
        const gp_Pnt mid(mid_radius * std::cos(mid_angle), mid_radius * std::sin(mid_angle), 0);
        GC_MakeArcOfCircle maker(a, mid, b);
        if (!maker.IsDone()) throw Error("an arc of the wheel outline could not be made");
        return BRepBuilderAPI_MakeEdge(maker.Value()).Edge();
    };
    BRepBuilderAPI_MakeWire wire;
    for (int t = 0; t < spec.teeth; ++t) {
        const double turn = t * pitch;
        const double tip_angle = turn;  // the tooth is centred on its own axis
        wire.Add(spline(turn, false));
        wire.Add(arc(point(flank.back(), turn, false), spec.tip_radius(), tip_angle, point(flank.back(), turn, true)));
        wire.Add(spline(turn, true));
        const gp_Pnt from = point(flank.front(), turn, true), to = point(flank.front(), turn + pitch, false);
        wire.Add(arc(from, spec.root_radius(), turn + 0.5 * pitch, to));
    }
    if (!wire.IsDone()) throw Error("the wheel outline could not be turned into a wire");
    BRepBuilderAPI_MakeFace face(wire.Wire(), true);
    if (!face.IsDone()) throw Error("the wheel outline could not be turned into a face");
    TopoDS_Shape solid = BRepPrimAPI_MakePrism(face.Face(), gp_Vec(0, 0, thickness)).Shape();
    if (spec.bore_radius > 0) {
        const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -thickness), gp_Dir(0, 0, 1)), spec.bore_radius, 3 * thickness).Shape();
        solid = BRepAlgoAPI_Cut(solid, bore).Shape();
    }
    return solid;
}

void export_gear_step(const gear::GearSpec& spec, double thickness, const std::string& name, const std::filesystem::path& path) {
    geometry::ExportPart part;
    part.name = name;
    part.shape = gear_solid(spec, thickness);
    geometry::export_step({part}, path);
}

report::Drawing gear_profile(const gear::GearSpec& spec) {
    report::Drawing d;
    d.title = "gear profile";
    std::vector<report::Pt> pts;
    for (const auto& p : gear::outline(fine(spec))) pts.push_back({p.x, p.y});
    d.polyline(pts, true, report::Layer::Outline);
    if (spec.bore_radius > 0) d.circle({0, 0}, spec.bore_radius, report::Layer::Outline);
    return d;
}

std::vector<std::string> gear_info(const gear::GearSpec& s, const DrawingNotes& n) {
    std::vector<std::string> info;
    if (s.profile == gear::Profile::Cycloidal) {
        info.push_back("Spur gear, cycloidal: " + std::to_string(s.teeth) + " teeth, module " + num(s.module, 4) + ", rolling circle radius " + num(s.rolling_radius * s.module, 4));
    } else {
        info.push_back("Spur gear, involute: " + std::to_string(s.teeth) + " teeth, module " + num(s.module, 4) + ", pressure angle " + num(s.pressure_angle_deg, 1) + " deg");
    }
    info.push_back("Tip diameter " + num(2 * s.tip_radius()) + " +-" + num(n.tolerance_diameter, 3) + ", pitch diameter " + num(2 * s.pitch_radius()) + ", root diameter " + num(2 * s.root_radius()) +
                   (s.bore_radius > 0 ? ", bore " + num(2 * s.bore_radius) : ""));
    info.push_back("Thickness " + num(n.thickness) + (n.material.empty() ? "" : ", material " + n.material) + ", root fillet " + num(s.root_fillet * s.module, 3));
    return info;
}

report::Drawing gear_drawing(const gear::GearSpec& s, const DrawingNotes& n) {
    using report::Layer;
    using report::Pt;
    report::Drawing d;
    d.title = n.title.empty() ? "gear" : n.title;
    const double rt = s.tip_radius(), rp = s.pitch_radius(), rr = s.root_radius();

    // Front view.
    std::vector<Pt> pts;
    for (const auto& p : gear::outline(fine(s))) pts.push_back({p.x, p.y});
    d.polyline(pts, true, Layer::Outline);
    if (s.bore_radius > 0) d.circle({0, 0}, s.bore_radius, Layer::Outline);
    d.circle({0, 0}, rp, Layer::Centre);
    d.circle({0, 0}, rr, Layer::Hidden);
    d.line({-rt - 3, 0}, {rt + 3, 0}, Layer::Centre);
    d.line({0, -rt - 3}, {0, rt + 3}, Layer::Centre);

    // Diameter labels in a column to the right, each joined to its circle by a leader. Larger
    // leader angles go higher so the leaders do not cross.
    struct Dim {
        double radius, angle_deg;
        std::string label;
    };
    const Dim dims[] = {{s.bore_radius, 80, "bore D " + num(2 * s.bore_radius)}, {rt, 62, "tip D " + num(2 * rt)}, {rp, 44, "pitch D " + num(2 * rp)}, {rr, 26, "root D " + num(2 * rr)}};
    const double text_h = std::max(0.6, 0.07 * rt);
    const double column = 1.45 * rt, elbow = 1.2 * rt;
    double y_label = 1.1 * rt;
    double label_w = 0;
    for (const Dim& m : dims) {
        if (m.radius <= 0) continue;
        const double a = m.angle_deg * std::numbers::pi / 180;
        const Pt on{m.radius * std::cos(a), m.radius * std::sin(a)};
        d.line(on, {elbow, y_label}, Layer::Dimension);
        d.line({elbow, y_label}, {column - 0.1 * text_h, y_label}, Layer::Dimension);
        d.text({column, y_label - 0.3 * text_h}, m.label, text_h, Layer::Dimension, 0);
        label_w = std::max(label_w, 0.55 * text_h * static_cast<double>(m.label.size()));
        y_label -= 2.6 * text_h;
    }

    // Side view to the right of the labels, showing the thickness and the bore.
    const double sx = column + label_w + 2 * text_h;
    const double w = n.thickness;
    d.polyline({{sx, -rt}, {sx + w, -rt}, {sx + w, rt}, {sx, rt}}, true, Layer::Outline);
    if (s.bore_radius > 0) {
        d.line({sx, s.bore_radius}, {sx + w, s.bore_radius}, Layer::Hidden);
        d.line({sx, -s.bore_radius}, {sx + w, -s.bore_radius}, Layer::Hidden);
    }
    d.line({sx - 0.4 * rt, 0}, {sx + w + 0.4 * rt, 0}, Layer::Centre);
    d.line({sx, -rt - 0.5 * text_h}, {sx + w, -rt - 0.5 * text_h}, Layer::Dimension);
    d.text({sx + w / 2, -rt - 2.2 * text_h}, "t " + num(w), text_h, Layer::Dimension, 1);
    d.text({sx + w / 2, rt + text_h}, "side view", text_h, Layer::Text, 1);
    d.text({0, rt + 2.2 * text_h}, "front view", text_h, Layer::Text, 1);
    return d;
}

}  // namespace ohs::manufacturing
