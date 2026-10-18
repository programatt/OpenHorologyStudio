// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <TopoDS_Shape.hxx>

#include "mesh/mesh2d.hpp"

namespace ohs::geometry {

/// The outline of a part cut by the horizontal plane z: the largest closed loop is the outer
/// ring, loops inside it are holes (bores, cut-outs). Curves are sampled to within `deflection`
/// mm; with `simplify` > 0 the outlines are then thinned (Douglas-Peucker) so that no point moves
/// by more than that distance, which keeps the mesh from following CAD tessellation noise. Throws ohs::Error when the plane misses the part or the section does not close.
mesh::Region section_region(const TopoDS_Shape& shape, double z, double deflection = 0.005, double simplify = 0.0);

}  // namespace ohs::geometry
