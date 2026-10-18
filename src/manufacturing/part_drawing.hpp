// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "mesh/mesh2d.hpp"
#include "report/drawing.hpp"

namespace ohs::manufacturing {

struct PartDrawingInput {
    std::string title;
    mesh::Region region;       ///< section outline: outer ring first, then holes
    double thickness{};
    std::string material;
    std::vector<std::string> notes;
    int teeth{};               ///< when > 0 the drawing carries the tooth data
    double module{};
};

struct HoleInfo {
    double x{}, y{};           ///< centre from the datum (lower left of the part's bounding box), mm
    double diameter{};
    bool round{true};
};

struct PartDrawing {
    report::Drawing drawing;
    std::vector<std::string> info;     ///< lines for the title block
    std::vector<HoleInfo> holes;
    double width{}, height{};          ///< overall size, mm
};

/// A dimensioned drawing of a flat part: outline, round holes as circles with centre marks and
/// diameters, overall width and height, hole positions from the lower-left datum.
PartDrawing make_part_drawing(const PartDrawingInput& input);

}  // namespace ohs::manufacturing
