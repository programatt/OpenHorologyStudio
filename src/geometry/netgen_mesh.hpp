// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <optional>

#include <TopoDS_Shape.hxx>

#include "mesh/tetmesh.hpp"

namespace ohs::geometry {

// Netgen (LGPL-2.1) meshes the B-rep itself, so features that vary through the thickness (counterbores,
// steps, bosses, chamfers) are meshed where they are, which the section-and-extrude mesher cannot do.
// It runs out of process: a short Python script (embedded in netgen_mesh.cpp) reads the solid from a STEP
// file with Netgen's `netgen.occ` module and writes its tetrahedra to a text file, so OHS links nothing
// from it and needs no Netgen at build time. Install it with `pip install netgen-mesher`.

/// The Python interpreter that can `import netgen.occ`: $OHS_NETGEN_PYTHON, else `python3` on PATH.
std::optional<std::filesystem::path> find_netgen_python();

/// Tetrahedral mesh of a solid, elements about `max_size` mm across, linear (order 1) or quadratic (order 2,
/// midside nodes on the straight edges, as in the rest of OHS). Tetrahedra come out positively oriented.
/// `work_dir` holds the intermediate files. Throws ohs::Error when Netgen is not installed or fails.
mesh::TetMesh mesh_solid_with_netgen(const TopoDS_Shape& shape, double max_size, int order, const std::filesystem::path& work_dir);

}  // namespace ohs::geometry
