// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "fea/fea.hpp"

namespace ohs::fea {

/// A named per-node field written to the VTU file: 1 value per node (scalar) or 3 (vector).
struct NodeField {
    std::string name;
    std::vector<double> values;
    int components{1};
};

/// ParaView-readable VTK XML unstructured grid (linear or quadratic tetrahedra, ASCII).
std::string to_vtu(const mesh::TetMesh& mesh, const std::vector<NodeField>& fields);
void write_vtu(const std::filesystem::path& path, const mesh::TetMesh& mesh, const std::vector<NodeField>& fields);

/// Displacement (vector), von Mises and the stress components of a static result.
std::vector<NodeField> static_fields(const StaticResult& result);

/// A mode shape as a displacement field.
NodeField mode_field(const Vec& mode, const std::string& name);

}  // namespace ohs::fea
