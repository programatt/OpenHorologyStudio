// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <vector>

#include "geometry/features.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/model.hpp"

namespace ohs::animation {

struct AnimationOptions {
    double duration_seconds = 10.0;  ///< length of the animation
    double fps = 30.0;               ///< at least this many keyframes per second
    double time_scale = 1.0;         ///< real seconds of movement time per animation second
    bool stepped = false;            ///< advance one escapement step per beat
    double max_keyframe_rotation = 1.5707963267948966;  ///< radians; keyframes are added to stay below it
    double linear_deflection = 0.01;                    ///< mm, tessellation tolerance
    std::size_t max_keyframes = 200000;
};

struct AnimationStats {
    std::size_t keyframes{};
    std::size_t triangles{};
    std::size_t animated_nodes{};
    double max_degrees_per_keyframe{};
};

/// Writes a binary glTF 2.0 file with one node per part and a rotation animation per mobile.
/// `parts` are the imported CAD parts (as import_step returns them) and are matched to the
/// movement's components by name; `dial_axis` must be the axis used when the movement was
/// imported. Units are millimetres scaled to metres, Z-up CAD data rotated to glTF's Y-up.
AnimationStats write_animated_glb(const model::Movement& movement, const std::vector<geometry::ImportedPart>& parts,
                                  const kinematics::TrainAnalysis& train, geometry::Axis dial_axis,
                                  const AnimationOptions& options, const std::filesystem::path& path);

/// Writes the same animation as an OpenUSD text file (.usda): millimetres (metersPerUnit 0.001), Z up,
/// one Xform per part with a rotateZ time sample per keyframe (seconds, timeCodesPerSecond = 1) and a
/// Mesh child holding the flat-shaded triangles. Statistics are those of write_animated_glb.
AnimationStats write_animated_usd(const model::Movement& movement, const std::vector<geometry::ImportedPart>& parts,
                                  const kinematics::TrainAnalysis& train, geometry::Axis dial_axis,
                                  const AnimationOptions& options, const std::filesystem::path& path);

}  // namespace ohs::animation
