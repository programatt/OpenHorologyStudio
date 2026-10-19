// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <numbers>
#include <vector>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"

namespace ohs::test {

// The going train of the test movement, with three more parts on its barrel (80 teeth, radii 5.81 to 6.15 mm):
// a rod along x for a stem, a click with its pivot hole at (3.3, 6) and its tip on the teeth at (0, 6), and a
// strip over the click, fixed at x = 7 and resting on it at x = 2.
inline std::vector<geometry::ExportPart> train_with_keyless_work() {
    auto parts = geometry::make_test_movement();
    gp_Trsf lay;
    lay.SetRotation(gp_Ax1(gp::Origin(), gp::DY()), std::numbers::pi / 2);  // z -> x
    parts.push_back({"part_a", BRepBuilderAPI_Transform(geometry::make_arbor(0.75, 0.0, 17.7), lay, true).Shape(), -8.85, 17.6, 0.0, 0.0});

    TopoDS_Shape click = BRepPrimAPI_MakeBox(gp_Pnt(0, 5.5, 0.3), 3.6, 1.0, 0.3).Shape();
    gp_Trsf to_pivot;
    to_pivot.SetTranslation(gp_Vec(3.3, 6.0, 0.0));
    click = BRepAlgoAPI_Cut(click, BRepBuilderAPI_Transform(geometry::make_arbor(0.3, 0.0, 1.0), to_pivot, true).Shape()).Shape();
    parts.push_back({"part_b", click, 0.0, 0.0, 0.0, 0.0});

    parts.push_back({"part_c", BRepPrimAPI_MakeBox(gp_Pnt(2.0, 6.4, 0.6), 5.0, 0.8, 0.2).Shape(), 0.0, 0.0, 0.0, 0.0});
    return parts;
}

}  // namespace ohs::test
