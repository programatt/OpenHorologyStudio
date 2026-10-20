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

/// Finds the CAD axis normal to the dial: wheels, plates and bridges are flat, so it is the axis
/// along which the parts are thinnest (smallest median bounding-box extent). Z when there are no parts.
Axis detect_dial_axis(const std::vector<ImportedPart>& parts);

/// "x", "y" or "z".
const char* axis_name(Axis axis);

/// Rotates the shape so the CAD dial axis becomes Z (identity for Axis::Z).
TopoDS_Shape orient_dial_axis_to_z(const TopoDS_Shape& shape, Axis axis);

/// Extracts mass properties, bounding box, outline radii and tooth count of one part.
/// Parts without volume (surfaces) come back with volume 0 and no teeth.
model::PartFeatures extract_features(const ImportedPart& part, const FeatureOptions& options = {});

/// The features of `to` from those of `from` (worked out by extract_features with the same options), when the two are
/// instances of one shape that differ only by a translation: a movement's screws and jewels are many placements of
/// a few shapes, and only the centre, the heights, the holes and the fingerprint move with the placement. False (nothing
/// written) for anything else, which the caller extracts in full.
bool translate_features(const ImportedPart& from, const model::PartFeatures& features, const ImportedPart& to,
                        const FeatureOptions& options, model::PartFeatures& out);

}  // namespace ohs::geometry
