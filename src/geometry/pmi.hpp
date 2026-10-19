// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "geometry/step_io.hpp"

namespace ohs::geometry {

/// One dimension with a tolerance, from the product and manufacturing information (PMI) of a STEP AP242 file.
struct PmiDimension {
    std::string part;      ///< name of the part the dimension is attached to ("" when it could not be placed)
    std::string kind;      ///< "diameter", "radius", "length" (a size or a distance) or "other"
    double nominal{};      ///< mm
    double plus{};         ///< upper deviation, mm (>= 0)
    double minus{};        ///< lower deviation as a magnitude, mm (>= 0)
};

/// Reads the dimensional tolerances of a STEP file. A file without PMI gives an empty list. Throws ohs::Error
/// when the file cannot be read.
std::vector<PmiDimension> read_step_pmi(const std::filesystem::path& path);

/// Writes parts as a STEP AP242 assembly carrying the given dimensions (attached to the part with that name),
/// so tolerances can be tested and exchanged. Dimensions whose part is not in `parts` are an error.
void export_step_with_pmi(const std::vector<ExportPart>& parts, const std::vector<PmiDimension>& dimensions,
                          const std::filesystem::path& path);

}  // namespace ohs::geometry
