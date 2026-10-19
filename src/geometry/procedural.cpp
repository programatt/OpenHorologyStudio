// SPDX-License-Identifier: Apache-2.0
#include "geometry/procedural.hpp"

#include <cmath>
#include <iterator>
#include <numbers>
#include <string>
#include <vector>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>

#include "core/error.hpp"

namespace ohs::geometry {

TopoDS_Shape make_gear(int teeth, double module, double z0, double thickness, double bore_radius) {
    if (teeth < 3) throw Error("a gear needs at least 3 teeth");
    const double tip = (teeth + 2) * module / 2.0;
    const double root = tip - 2.25 * module;
    const double pitch = 2.0 * std::numbers::pi / teeth;

    BRepBuilderAPI_MakePolygon polygon;
    auto point = [&](double angle, double radius) {
        polygon.Add(gp_Pnt(radius * std::cos(angle), radius * std::sin(angle), z0));
    };
    for (int i = 0; i < teeth; ++i) {
        const double a = i * pitch;
        point(a - 0.25 * pitch, root);
        point(a - 0.12 * pitch, tip);
        point(a + 0.12 * pitch, tip);
        point(a + 0.25 * pitch, root);
    }
    polygon.Close();
    const TopoDS_Shape body =
        BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(), gp_Vec(0, 0, thickness)).Shape();
    if (bore_radius <= 0.0) return body;

    const TopoDS_Shape bore =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0 - 1.0), gp_Dir(0, 0, 1)), bore_radius, thickness + 2.0).Shape();
    return BRepAlgoAPI_Cut(body, bore).Shape();
}

namespace {

double inv(double alpha) { return std::tan(alpha) - alpha; }

}  // namespace

TopoDS_Shape make_involute_gear(int teeth, double module, double z0, double thickness, double bore_radius,
                                double pressure_angle_deg) {
    if (teeth < 3) throw Error("a gear needs at least 3 teeth");
    const double alpha_p = pressure_angle_deg * std::numbers::pi / 180.0;
    const double rp = teeth * module / 2.0;
    const double rb = rp * std::cos(alpha_p);
    const double tip = rp + module;
    const double root = rp - 1.25 * module;
    const double pitch = 2.0 * std::numbers::pi / teeth;

    // Half-angle of the tooth at radius r, measured from the tooth centre line.
    auto half_angle = [&](double r) {
        if (r <= rb) return std::numbers::pi / (2.0 * teeth) + inv(alpha_p);
        return std::numbers::pi / (2.0 * teeth) + inv(alpha_p) - inv(std::acos(rb / r));
    };

    // Involute part of the flank, base circle (or root, if that is higher) to tip. Below the base
    // circle the flank is a radial line, built as its own edge so the spline cannot ring.
    const double start = std::max(root, rb);
    constexpr int kSamples = 14;
    std::vector<double> radii;
    for (int i = 0; i <= kSamples; ++i) radii.push_back(start + (tip - start) * i / kSamples);

    auto at = [&](double angle, double r) { return gp_Pnt(r * std::cos(angle), r * std::sin(angle), z0); };
    auto arc = [&](double a0, double a1, double r) {
        return BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(at(a0, r), at(0.5 * (a0 + a1), r), at(a1, r)).Value()).Edge();
    };
    auto spline = [&](double centre, double sign, bool rising) {
        TColgp_Array1OfPnt pts(1, static_cast<int>(radii.size()));
        for (std::size_t i = 0; i < radii.size(); ++i) {
            const double r = radii[rising ? i : radii.size() - 1 - i];
            pts(static_cast<int>(i) + 1) = at(centre + sign * half_angle(r), r);
        }
        return BRepBuilderAPI_MakeEdge(GeomAPI_PointsToBSpline(pts, 3, 8, GeomAbs_C2, 1e-6).Curve()).Edge();
    };
    auto radial = [&](double centre, double sign, bool rising) {
        const gp_Pnt lo = at(centre + sign * half_angle(root), root);
        const gp_Pnt hi = at(centre + sign * half_angle(start), start);
        return rising ? BRepBuilderAPI_MakeEdge(lo, hi).Edge() : BRepBuilderAPI_MakeEdge(hi, lo).Edge();
    };
    const bool undercut = rb > root;

    BRepBuilderAPI_MakeWire wire;
    for (int t = 0; t < teeth; ++t) {
        const double a = t * pitch;
        if (undercut) wire.Add(radial(a, -1.0, true));
        wire.Add(spline(a, -1.0, true));
        wire.Add(arc(a - half_angle(tip), a + half_angle(tip), tip));
        wire.Add(spline(a, +1.0, false));
        if (undercut) wire.Add(radial(a, +1.0, false));
        wire.Add(arc(a + half_angle(root), a + pitch - half_angle(root), root));
    }
    if (!wire.IsDone()) throw Error("could not build the gear outline");
    const TopoDS_Shape body =
        BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(), gp_Vec(0, 0, thickness)).Shape();
    if (bore_radius <= 0.0) return body;
    const TopoDS_Shape bore =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0 - 1.0), gp_Dir(0, 0, 1)), bore_radius, thickness + 2.0).Shape();
    return BRepAlgoAPI_Cut(body, bore).Shape();
}

TopoDS_Shape make_ring_gear(int teeth, double module, double z0, double thickness, double outer_radius) {
    const double root = (teeth + 2) * module / 2.0;  // the hole is the outline of an external gear of these teeth
    if (outer_radius <= root) throw Error("the ring's outer radius must be larger than the roots of its teeth");
    const TopoDS_Shape hole = make_gear(teeth, module, z0 - 1.0, thickness + 2.0, 0.0);
    const TopoDS_Shape disc =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0), gp_Dir(0, 0, 1)), outer_radius, thickness).Shape();
    return BRepAlgoAPI_Cut(disc, hole).Shape();
}

TopoDS_Shape make_arbor(double radius, double z0, double z1) {
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0), gp_Dir(0, 0, 1)), radius, z1 - z0).Shape();
}

TopoDS_Shape make_plate(double cx, double cy, double size, double z0, double z1) {
    return BRepPrimAPI_MakeBox(gp_Pnt(cx - size / 2, cy - size / 2, z0), size, size, z1 - z0).Shape();
}

std::vector<ExportPart> make_test_movement() {
    constexpr double m = 0.15;
    constexpr double bore = 0.1;
    constexpr double deg = std::numbers::pi / 180.0;
    constexpr double two_pi = 2.0 * std::numbers::pi;

    // barrel(80) -> center pinion(10); center wheel(80) -> third pinion(10);
    // third wheel(75) -> fourth pinion(10); fourth wheel(84) -> escape pinion(6) on the escape wheel.
    // The fourth wheel turns at 1 rpm and the 15-tooth escape wheel at 14 rpm: 25,200 vph.
    struct Mobile {
        const char* prefix;
        int wheel_teeth;
        int pinion_teeth;  // 0 for the barrel, which has no pinion
        double wheel_z0, pinion_z0, thickness;
        double hop_distance, hop_angle_deg;  // from the previous mobile
    };
    const Mobile mobiles[] = {
        {"barrel", 80, 0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {"center", 80, 10, 1.2, 0.0, 0.8, m * (80 + 10) / 2, 0.0},
        {"third", 75, 10, 2.2, 1.2, 0.8, m * (80 + 10) / 2, 120.0},
        {"fourth", 84, 10, 3.2, 2.2, 0.8, m * (75 + 10) / 2, 20.0},
        {"escape", 15, 6, 4.2, 3.2, 0.8, m * (84 + 6) / 2, 60.0},
    };

    std::vector<ExportPart> parts;
    double x = 0, y = 0;
    double prev_x = 0, prev_y = 0, prev_phase = 0;
    int prev_wheel_teeth = 0;
    for (std::size_t i = 0; i < std::size(mobiles); ++i) {
        const Mobile& mb = mobiles[i];
        x += mb.hop_distance * std::cos(mb.hop_angle_deg * deg);
        y += mb.hop_distance * std::sin(mb.hop_angle_deg * deg);

        // Phase of this mobile so that the previous mobile's wheel and this pinion interleave:
        // with a = fraction of a driver pitch at which the line of centres falls, the driven gear
        // must see the line at (0.5 - a) of its own pitch.
        double phase = 0.0;
        if (i > 0) {
            const double theta = std::atan2(y - prev_y, x - prev_x);
            const double driver_pitch = two_pi / prev_wheel_teeth;
            const double driven_pitch = two_pi / mb.pinion_teeth;
            double a = std::fmod((theta - prev_phase) / driver_pitch, 1.0);
            if (a < 0) a += 1.0;
            phase = theta + std::numbers::pi - (0.5 - a) * driven_pitch;
        }

        const std::string name = mb.prefix;
        auto add = [&](std::string part_name, TopoDS_Shape shape, double rz) {
            parts.push_back({std::move(part_name), std::move(shape), x, y, 0.0, rz});
        };
        if (i == 0) {
            add("barrel", make_involute_gear(mb.wheel_teeth, m, mb.wheel_z0, mb.thickness, bore), phase);
        } else {
            add(name == "escape" ? "escape_pinion" : name + "_pinion",
                make_involute_gear(mb.pinion_teeth, m, mb.pinion_z0, mb.thickness, bore), phase);
            add(name == "escape" ? "escape_wheel" : name + "_wheel",
                make_involute_gear(mb.wheel_teeth, m, mb.wheel_z0, mb.thickness, bore), phase);
        }
        const double z_lo = (i == 0 ? mb.wheel_z0 : mb.pinion_z0) - 0.2;
        add(name + "_arbor", make_arbor(bore, z_lo, mb.wheel_z0 + mb.thickness + 0.2), 0.0);

        prev_x = x;
        prev_y = y;
        prev_phase = phase;
        prev_wheel_teeth = mb.wheel_teeth;
    }
    parts.push_back({"main_plate", make_plate(0, 0, 40.0, -0.8, -0.3), 10.0, 7.0, 0.0, 0.0});
    return parts;
}

}  // namespace ohs::geometry
