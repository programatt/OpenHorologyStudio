// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <vector>

#include "geometry/step_io.hpp"

namespace ohs::geometry {

/// Thickness (mm) given to a DXF profile that carries none of its own.
inline constexpr double kDefaultDxfThickness = 1.0;

/// Reads an IGES file. Every solid becomes a part ("part", "part#2", ...); a file with surfaces but no
/// solids gives one part per root entity. IGES carries no assembly names or placements worth trusting,
/// so parts are named by order. Throws ohs::Error when the file cannot be read or holds no geometry.
std::vector<ImportedPart> import_iges(const std::filesystem::path& path);

/// Writes parts as one IGES file (millimetres). Names are not stored. For tests and CAD exchange.
void export_iges(const std::vector<ExportPart>& parts, const std::filesystem::path& path);

/// Reads the closed 2D contours of a DXF file (LINE, ARC, CIRCLE, LWPOLYLINE and POLYLINE with bulges)
/// in millimetres. Contours nested inside another become holes in it; each outermost contour is
/// extruded along Z into a part "profile", "profile#2", ... in order of appearance. The thickness is the
/// entity's own DXF thickness (group 39) when it has one, else `default_thickness`.
/// Throws ohs::Error for open contours, unreadable text, or a file with no closed contour.
std::vector<ImportedPart> import_dxf(const std::filesystem::path& path, double default_thickness = kDefaultDxfThickness);

/// Reads a CAD file by extension: .step/.stp, .iges/.igs or .dxf (case-insensitive).
std::vector<ImportedPart> import_cad(const std::filesystem::path& path);

}  // namespace ohs::geometry
