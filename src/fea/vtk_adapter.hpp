// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "fea/results.hpp"

namespace ohs::fea {

// VTK (BSD-3) is linked in-process, but only for its XML file readers and writers: OHS's own VTU writer is
// text, and this adapter writes the same data as VTK's own binary, zlib-compressed files (several times
// smaller, quick to load) and opens any VTU file with VTK's reader to say what VTK sees.

/// True when this build links VTK.
bool vtk_available();

/// Writes the mesh and fields as a VTK XML unstructured grid with zlib-compressed binary data.
/// Throws ohs::Error without VTK, for a field that does not match the mesh, or when the file cannot be written.
void write_vtu_compressed(const std::filesystem::path& path, const mesh::TetMesh& mesh, const std::vector<NodeField>& fields);

struct VtkSummary {
    bool ok{false};
    std::string error;
    std::size_t points{};
    std::size_t cells{};
    int cell_type{};                  ///< VTK cell type of the first cell: 10 tetrahedron, 24 quadratic tetrahedron
    std::vector<std::string> arrays;  ///< names of the point data arrays
    double bounds[6]{};               ///< xmin xmax ymin ymax zmin zmax
};

/// Opens a .vtu file with VTK's own reader (ASCII or binary). `ok` is false, with VTK's complaint in
/// `error`, for a file VTK cannot read. Throws ohs::Error without VTK.
VtkSummary inspect_with_vtk(const std::filesystem::path& path);

}  // namespace ohs::fea
