// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "animation/gltf_animation.hpp"

namespace ohs::animation {

/// Mobile angles (radians, continuous, so they never wrap) sampled at keyframe times.
struct Keyframes {
    std::vector<double> times;                 ///< animation seconds
    std::vector<std::vector<double>> angles;   ///< [mobile][keyframe]
    std::vector<std::vector<double>> part_angles;  ///< [articulation of the train][keyframe]: stem, levers, springs
    double max_step{};                         ///< largest turn of any mobile between two keyframes, radians
};

/// Chooses the keyframe spacing (at least `fps`, fine enough that no mobile turns more than the cap
/// and every escapement impulse is sampled several times) and samples the train. Throws Error for a
/// movement with no beat rate or a request that would need more than `max_keyframes`.
Keyframes make_keyframes(const model::Movement& movement, const kinematics::TrainAnalysis& train, const AnimationOptions& options);

struct Vec3f {
    float x, y, z;
};

/// Flat-shaded triangles (three vertices each, so sharp tooth edges stay sharp) of a part, with its
/// pivot moved to the origin in x and y (and in z, when `pivot_z` is given).
struct MeshData {
    std::vector<Vec3f> positions;
    std::vector<Vec3f> normals;
};
MeshData tessellate(const TopoDS_Shape& shape, double deflection, double pivot_x, double pivot_y, double pivot_z = 0.0);

}  // namespace ohs::animation
