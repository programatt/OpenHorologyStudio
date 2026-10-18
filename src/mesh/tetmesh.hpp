// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <functional>
#include <vector>

#include "mesh/mesh2d.hpp"

namespace ohs::mesh {

struct V3 {
    double x{};
    double y{};
    double z{};
};

/// Tetrahedral mesh with 4 (order 1) or 10 (order 2) nodes per element. Node numbering: the four
/// corners 0..3, then the edge midpoints 4..9 on edges (0,1) (1,2) (2,0) (0,3) (1,3) (2,3).
/// Every element has positive volume.
struct TetMesh {
    std::vector<V3> nodes;
    std::vector<std::array<int, 10>> elems;  ///< entries 4..9 are -1 for order 1
    int order = 1;

    int nodes_per_element() const { return order == 1 ? 4 : 10; }
    std::size_t dofs() const { return 3 * nodes.size(); }
};

/// Local edge number of the corner pair (i, j) of a tetrahedron, 0..5.
int tet_edge(int i, int j);

/// Prisms from extruding a triangle mesh between z0 and z1 in `layers` layers, each split into
/// three tetrahedra with the diagonal rule "lower index on top" so neighbours always agree.
TetMesh extrude(const Mesh2d& surface, double z0, double z1, int layers, int order = 1);

/// Structured rectangle of nx * ny cells (two triangles each), for box-shaped test bodies.
Mesh2d structured_rect(double x0, double y0, double x1, double y1, int nx, int ny);

/// Adds the edge midpoints: order 1 -> 2.
void promote_to_quadratic(TetMesh& mesh);

double volume(const TetMesh& mesh);

struct BoundaryFace {
    std::array<int, 6> nodes;  ///< corners 0..2 (outward counter-clockwise) then midpoints of edges 01, 12, 20; -1 for order 1
    int elem;
};

/// Faces that belong to exactly one element, oriented outward.
std::vector<BoundaryFace> boundary_faces(const TetMesh& mesh);

V3 face_centroid(const TetMesh& mesh, const BoundaryFace& face);
V3 face_normal(const TetMesh& mesh, const BoundaryFace& face);  ///< unit outward normal
double face_area(const TetMesh& mesh, const BoundaryFace& face);

/// Nodes (indices) whose position satisfies the predicate.
std::vector<int> nodes_where(const TetMesh& mesh, const std::function<bool(const V3&)>& pred);
std::vector<BoundaryFace> faces_where(const TetMesh& mesh, const std::vector<BoundaryFace>& faces,
                                      const std::function<bool(const V3& centroid, const V3& normal)>& pred);

/// Worst element quality: smallest value of 3 * inscribed / circumscribed radius ratio proxy
/// (1 for a regular tetrahedron), as 6 sqrt(2) V / l_rms^3.
double min_quality(const TetMesh& mesh);

}  // namespace ohs::mesh
