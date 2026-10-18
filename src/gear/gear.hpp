// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "mesh/mesh2d.hpp"

namespace ohs::gear {

/// A spur gear with involute teeth, in the dial plane, centred on the origin with tooth 0 on the
/// +x axis. (Cycloidal horological profiles follow the same interface later.)
struct GearSpec {
    int teeth = 24;
    double module = 1.0;            ///< pitch diameter / teeth, mm
    double pressure_angle_deg = 20;
    double addendum = 1.0;          ///< in modules
    double dedendum = 1.25;         ///< in modules
    double root_fillet = 0.38;      ///< in modules; 0 gives a sharp root corner
    double bore_radius = 0;         ///< mm, 0 for none
    int points_per_flank = 20;
    int points_per_fillet = 8;

    double pitch_radius() const { return 0.5 * module * teeth; }
    double base_radius() const;
    double tip_radius() const { return pitch_radius() + addendum * module; }
    double root_radius() const { return pitch_radius() - dedendum * module; }
};

/// Closed outline of the whole wheel, counter-clockwise, without the bore. Throws ohs::Error if
/// the tooth is pointed or the parameters are impossible.
mesh::Ring outline(const GearSpec& spec);

/// One side of a tooth in the tooth's own frame (tooth centred on +x): from the root circle through the
/// fillet and up the flank to the tip. The other side is its mirror image about the x axis. Used to
/// build exact (spline) CAD geometry instead of a polygon.
mesh::Ring tooth_flank(const GearSpec& spec);

/// The wheel with its bore as a hole.
mesh::Region wheel_region(const GearSpec& spec);

/// A sector of `tooth_count` teeth (odd) centred on tooth 0, down to `rim_radius` below the
/// root, closed by the arc at that radius and straight sides. Used for tooth-bending analysis.
mesh::Region sector_region(const GearSpec& spec, int tooth_count, double rim_radius);

/// Angle (rad) from the tooth centre line to the flank at radius r: half the tooth thickness angle.
double half_tooth_angle(const GearSpec& spec, double r);

/// Lewis form factor Y for a 20 degree full-depth tooth (load at the tip), interpolated from the
/// standard table (Shigley), so that sigma = Wt / (F m Y).
double lewis_form_factor(int teeth);

/// Tooth bending stress by the Lewis formula in MPa: tangential load wt (N) on face width f (mm).
double lewis_stress(const GearSpec& spec, double wt, double face_width);

}  // namespace ohs::gear
