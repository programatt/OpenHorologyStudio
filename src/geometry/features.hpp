// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "geometry/step_io.hpp"
#include "model/model.hpp"

namespace ohs::geometry {

enum class Axis { X, Y, Z };

struct FeatureOptions {
    Axis dial_axis = Axis::Z;       ///< CAD axis normal to the dial; analysis is done with it as Z
    double sample_step = 0.002;     ///< mm between outline samples
    int angular_bins = 1440;        ///< resolution of the radius-vs-angle outline
    int min_teeth = 6;
};

/// Extracts mass properties, bounding box, outline radii and tooth count of one part.
/// Parts without volume (surfaces) come back with volume 0 and no teeth.
model::PartFeatures extract_features(const ImportedPart& part, const FeatureOptions& options = {});

}  // namespace ohs::geometry
