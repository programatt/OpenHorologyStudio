// SPDX-License-Identifier: Apache-2.0
#include "geometry/procedural.hpp"

#include <cmath>
#include <numbers>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
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

    // barrel(80) -> center pinion(10); center wheel(80) -> third pinion(10);
    // third wheel(75) -> fourth pinion(10); fourth wheel(84) -> escape pinion(6) on the escape wheel.
    // The fourth wheel turns at 1 rpm and the 15-tooth escape wheel at 14 rpm: 25,200 vph.
    std::vector<ExportPart> parts;
    auto add = [&parts](std::string name, TopoDS_Shape shape, double x, double y) {
        parts.push_back({std::move(name), std::move(shape), x, y, 0.0});
    };

    double x = 0, y = 0;
    add("barrel", make_gear(80, m, 0.0, 1.0, bore), x, y);
    add("barrel_arbor", make_arbor(bore, -0.2, 1.2), x, y);

    struct Hop { double distance, angle_deg; };
    const double d_center = m * (80 + 10) / 2, d_third = m * (80 + 10) / 2;
    const double d_fourth = m * (75 + 10) / 2, d_escape = m * (84 + 6) / 2;
    const Hop hops[] = {{d_center, 0}, {d_third, 120}, {d_fourth, 20}, {d_escape, 60}};

    add("center_pinion", make_gear(10, m, 0.0, 1.0, bore), x + hops[0].distance, y);
    x += hops[0].distance;
    add("center_wheel", make_gear(80, m, 1.2, 0.8, bore), x, y);
    add("center_arbor", make_arbor(bore, -0.2, 2.2), x, y);

    x += hops[1].distance * std::cos(hops[1].angle_deg * deg);
    y += hops[1].distance * std::sin(hops[1].angle_deg * deg);
    add("third_pinion", make_gear(10, m, 1.2, 0.8, bore), x, y);
    add("third_wheel", make_gear(75, m, 2.2, 0.8, bore), x, y);
    add("third_arbor", make_arbor(bore, 1.0, 3.2), x, y);

    x += hops[2].distance * std::cos(hops[2].angle_deg * deg);
    y += hops[2].distance * std::sin(hops[2].angle_deg * deg);
    add("fourth_pinion", make_gear(10, m, 2.2, 0.8, bore), x, y);
    add("fourth_wheel", make_gear(84, m, 3.2, 0.8, bore), x, y);
    add("fourth_arbor", make_arbor(bore, 2.0, 4.2), x, y);

    x += hops[3].distance * std::cos(hops[3].angle_deg * deg);
    y += hops[3].distance * std::sin(hops[3].angle_deg * deg);
    add("escape_pinion", make_gear(6, m, 3.2, 0.8, bore), x, y);
    add("escape_wheel", make_gear(15, m, 4.2, 0.8, bore), x, y);
    add("escape_arbor", make_arbor(bore, 3.0, 5.2), x, y);

    add("main_plate", make_plate(0, 0, 40.0, -0.8, -0.3), 10.0, 7.0);
    return parts;
}

}  // namespace ohs::geometry
