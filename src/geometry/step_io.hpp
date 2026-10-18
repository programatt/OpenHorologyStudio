// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace ohs::geometry {

/// One leaf part of an assembly, with the assembly placement already applied to `shape`.
struct ImportedPart {
    std::string name;
    TopoDS_Shape shape;
};

/// Reads a STEP file (AP203/AP214/AP242), flattening the assembly tree into leaf parts.
/// Part names come from the instance name, falling back to the product name; duplicates get a
/// "#n" suffix. Throws ohs::Error when the file cannot be read.
std::vector<ImportedPart> import_step(const std::filesystem::path& path);

/// A part for export. `placement` is applied as an assembly component transform.
struct ExportPart {
    std::string name;
    TopoDS_Shape shape;
    double dx{};
    double dy{};
    double dz{};
};

/// Writes the parts as one STEP assembly with named components.
void export_step(const std::vector<ExportPart>& parts, const std::filesystem::path& path);

}  // namespace ohs::geometry
