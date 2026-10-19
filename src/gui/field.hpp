// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace ohs::gui {

using Point3 = std::array<double, 3>;

struct Triangle {
    Point3 a, b, c;  ///< millimetres, dial axis = Z, relative to the part's mobile centre
};

/// A result field drawn on one part in place of its CAD surface: the boundary of the finite element mesh,
/// each triangle carrying the field's value at its three corners.
struct FieldOverlay {
    std::string part;   ///< the scene part it replaces
    std::string label;  ///< e.g. "von Mises stress (MPa)"
    double lo{}, hi{};  ///< range of the colour scale
    std::vector<Triangle> triangles;
    std::vector<std::array<double, 3>> values;
};

/// Names of the fields a result file offers to plot, in a stable order: "von_mises" and "displacement".
std::vector<std::string> plottable_fields(const std::filesystem::path& vtu);

/// Builds the overlay of `field` ("von_mises", or "displacement" as the length of the displacement vector)
/// from a VTU result file. Triangles are put relative to (pivot_x, pivot_y) like a part's own. Throws ohs::Error.
FieldOverlay field_overlay_from_vtu(const std::filesystem::path& vtu, const std::string& part, const std::string& field,
                                    double pivot_x, double pivot_y);

/// Colour of the scale at t in [0, 1] (dark blue to yellow, readable in grey scale and for colour-blind viewers).
std::array<int, 3> field_colour(double t);

}  // namespace ohs::gui
