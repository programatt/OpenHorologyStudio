// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "geometry/step_io.hpp"

namespace ohs::geometry {

/// Straight-flank spur gear centred on the Z axis, spanning z0..z0+thickness. Tip radius is
/// (teeth + 2) * module / 2 and the root is 2.25 modules below it. A zero bore leaves it solid.
TopoDS_Shape make_gear(int teeth, double module, double z0, double thickness, double bore_radius);

/// Standard involute spur gear (addendum 1 module, dedendum 1.25 modules) centred on the Z axis,
/// with a tooth centred on +X. Tip radius is (teeth + 2) * module / 2.
TopoDS_Shape make_involute_gear(int teeth, double module, double z0, double thickness, double bore_radius,
                                double pressure_angle_deg = 20.0);

/// A ring with straight-flank teeth on the inside, like a date wheel, centred on the Z axis and spanning
/// z0..z0+thickness. The teeth point inwards: their tips are at (teeth - 2.5) * module / 2 and the roots
/// 2.25 modules further out, at (teeth + 2) * module / 2. The outside is a plain circle of `outer_radius`, which must clear the roots.
TopoDS_Shape make_ring_gear(int teeth, double module, double z0, double thickness, double outer_radius);

/// Solid cylinder on the Z axis spanning z0..z1.
TopoDS_Shape make_arbor(double radius, double z0, double z1);

/// Box centred on (cx, cy) spanning z0..z1.
TopoDS_Shape make_plate(double cx, double cy, double size, double z0, double z1);

/// A going train from barrel to escape wheel generated from code, so tests never need CAD files.
/// Beat rate 25,200 vph with a 1 rpm fourth wheel; involute gears of module 0.15 mm, each mobile
/// rotated so that meshing teeth interleave without touching.
std::vector<ExportPart> make_test_movement();

}  // namespace ohs::geometry
