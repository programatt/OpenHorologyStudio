// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>

#include <TopoDS_Shape.hxx>

#include "gear/gear.hpp"
#include "report/drawing.hpp"

namespace ohs::manufacturing {

/// The wheel as a B-rep solid (outline extruded by `thickness`, bore cut), with the profile
/// sampled finely enough that the chord error is a few micrometres.
TopoDS_Shape gear_solid(const gear::GearSpec& spec, double thickness);

/// STEP file of the wheel (one named part), for CAD, CAM and the rest of OHS.
void export_gear_step(const gear::GearSpec& spec, double thickness, const std::string& name, const std::filesystem::path& path);

/// The bare profile (outline and bore on the OUTLINE layer, no annotation) for laser, wire or
/// wheel-cutting set-up files.
report::Drawing gear_profile(const gear::GearSpec& spec);

struct DrawingNotes {
    std::string title;
    std::string material;
    double thickness{};
    double tolerance_diameter{0.01};   ///< +- on the tip diameter, printed in the notes
};

/// Dimensioned part drawing: front view with tip, pitch, root and bore circles and a side view
/// showing the thickness, plus the gear data a cutter set-up needs.
report::Drawing gear_drawing(const gear::GearSpec& spec, const DrawingNotes& notes);

/// Lines of text for the title block of a gear drawing (module, teeth, pressure angle, diameters).
std::vector<std::string> gear_info(const gear::GearSpec& spec, const DrawingNotes& notes);

}  // namespace ohs::manufacturing
