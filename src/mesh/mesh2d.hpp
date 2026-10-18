// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <functional>
#include <vector>

namespace ohs::mesh {

struct P2 {
    double x{};
    double y{};
};

/// A closed polygon given by its corners (no repeated last point). Curved boundaries must
/// already be sampled finely enough for the accuracy wanted.
using Ring = std::vector<P2>;

/// One outer ring followed by any number of holes; orientations do not matter.
struct Region {
    std::vector<Ring> rings;
};

struct Mesh2dOptions {
    double size = 1.0;                       ///< target edge length
    std::function<double(P2)> size_at;       ///< optional local target edge length (overrides `size`)
    double max_radius_edge_ratio = 1.5;      ///< Ruppert quality bound; 1.414 gives a 20.7 degree minimum angle
    int max_nodes = 400000;
    int smoothing_passes = 3;
};

struct Mesh2d {
    std::vector<P2> nodes;
    std::vector<std::array<int, 3>> triangles;  ///< counter-clockwise
    /// Boundary segments (node index pairs) of all rings after refinement.
    std::vector<std::array<int, 2>> boundary;
};

/// Quality triangulation of the region: conforming Delaunay (boundary edges are split until they
/// are Delaunay edges) refined by Ruppert's algorithm to the radius-edge bound and the size field.
/// Throws ohs::Error for degenerate input or when `max_nodes` would be exceeded.
Mesh2d triangulate(const Region& region, const Mesh2dOptions& options);

/// Smallest and largest interior angle in degrees over all triangles.
std::array<double, 2> angle_range(const Mesh2d& mesh);

double area(const Mesh2d& mesh);

}  // namespace ohs::mesh
