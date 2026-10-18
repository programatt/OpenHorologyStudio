// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "geometry/features.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "mesh/mesh2d.hpp"
#include "model/model.hpp"

namespace ohs::manufacturing {

/// A part reduced to what the sweep needs: its outline in the plane of the dial, its height
/// range, and the mobile (rotation axis) it turns with; -1 for the plate, bridges and screws.
struct CollisionPart {
    std::string name;
    mesh::Region region;
    double z0{}, z1{};
    int mobile{-1};
};

struct CollisionConfig {
    int steps_per_revolution = 120;   ///< for parts that are not in mesh
    int steps_per_pitch = 24;         ///< for meshing wheels, over one tooth pitch of the driver
    double warn_clearance = 0.02;     ///< mm; closer than this is reported
    double overlap_tolerance = 0.02;  ///< mm; overlaps shallower than this count as contact within the outline accuracy, not as a collision
    double search_range = 0.5;        ///< mm; clearances above this are not measured
    double outline_tolerance = 0.003; ///< mm the section outlines may be thinned by
};

struct CollisionPair {
    std::string a, b;
    bool meshing{false};
    int steps{};
    int overlapping_steps{};
    double min_clearance{};   ///< mm; negative = deepest estimated overlap
    double at_degrees{};      ///< rotation of the pair's faster mobile at the closest approach
};

struct CollisionReport {
    std::vector<CollisionPair> pairs;   ///< every pair that shares a height range and can reach each other
    int pairs_checked{};
    int overlaps{};                     ///< pairs with any overlap
    int close_calls{};                  ///< pairs without overlap but under warn_clearance
};

/// Sweeps the kinematic motion of the train from the assembled pose and measures the closest
/// approach of every pair of parts on different mobiles whose height ranges overlap. A meshing pair
/// is swept through one tooth pitch of the driving wheel, any other pair through one revolution of
/// its faster mobile (poses beyond that depend on the whole train's common period and are not covered).
CollisionReport sweep_collisions(const model::Movement& movement, const kinematics::TrainAnalysis& train,
                                 const std::vector<CollisionPart>& parts, const CollisionConfig& config = {});

/// Section outlines of every component with a part in `step_parts` of the same name.
std::vector<CollisionPart> collision_parts(const model::Movement& movement, const std::vector<geometry::ImportedPart>& step_parts,
                                           geometry::Axis dial_axis, double outline_tolerance = 0.003);

}  // namespace ohs::manufacturing
