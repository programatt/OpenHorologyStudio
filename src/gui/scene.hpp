// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include "geometry/features.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/model.hpp"

namespace ohs::gui {

using Point3 = std::array<double, 3>;

struct Triangle {
    Point3 a, b, c;  ///< millimetres, dial axis = Z, relative to the part's mobile centre
};

/// One part of the movement as the viewport draws it.
struct ScenePart {
    std::string name;
    model::Role role{model::Role::Unknown};
    int mobile{-1};           ///< index into Movement::mobiles, or -1 for fixed parts
    double pivot_x{}, pivot_y{};
    double z_min{}, z_max{};
    std::vector<Triangle> triangles;
};

/// A cut-away plane: triangles on the positive side of `normal . p = offset` are removed.
struct SectionPlane {
    bool enabled{false};
    Point3 normal{0, 0, 1};
    double offset{0};
};

/// The movement made ready for drawing: tessellated parts, the train, and the view state the
/// workspaces change (time, exploded factor, section plane, visibility). Qt-free so it can be
/// tested without a display.
class Scene {
public:
    /// Loads a project file and its STEP geometry, oriented by the project's dial axis. Throws ohs::Error.
    static Scene load(const std::filesystem::path& project, const std::filesystem::path& step_override = {},
                      double beat_rate_vph = 0.0);

    const model::Movement& movement() const { return movement_; }
    const kinematics::TrainAnalysis& train() const { return train_; }
    const std::vector<ScenePart>& parts() const { return parts_; }

    void set_time(double seconds) { time_ = seconds; }
    double time() const { return time_; }
    void set_stepped(bool stepped) { stepped_ = stepped; }
    void set_explode(double factor) { explode_ = factor; }
    double explode() const { return explode_; }
    void set_section(const SectionPlane& plane) { section_ = plane; }
    const SectionPlane& section() const { return section_; }
    void set_visible(const std::string& part, bool visible);
    bool visible(const std::string& part) const;
    bool can_animate() const { return train_.root_period_hours > 0.0; }
    /// Why the going train could not be analysed (empty when it could).
    const std::string& train_error() const { return train_error_; }

    /// Triangles of every visible part in world space at the current time, exploded and cut.
    /// `part_index[i]` is the index into parts() of triangle i.
    struct Drawn {
        std::vector<Triangle> triangles;
        std::vector<int> part_index;
    };
    Drawn draw_list() const;

    /// Bounding box centre and radius of the assembled (unexploded) movement.
    Point3 centre() const;
    double radius() const;

private:
    model::Movement movement_;
    kinematics::TrainAnalysis train_;
    std::vector<ScenePart> parts_;
    std::string train_error_;
    std::vector<std::string> hidden_;
    double time_{0};
    bool stepped_{false};
    double explode_{0};
    SectionPlane section_;
};

}  // namespace ohs::gui
