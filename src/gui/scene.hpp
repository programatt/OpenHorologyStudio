// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry/features.hpp"
#include "gui/field.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/model.hpp"
#include "rigging/rigging.hpp"

namespace ohs::gui {

/// One part of the movement as the viewport draws it.
struct ScenePart {
    std::string name;
    model::Role role{model::Role::Unknown};
    int mobile{-1};           ///< index into Movement::mobiles, or -1 for fixed parts
    double pivot_x{}, pivot_y{};
    int articulation{-1};     ///< index into TrainAnalysis::articulations for a stem, lever or spring that moves, else -1
    double z_min{}, z_max{};
    bool closed{false};       ///< made only of solids, with outward faces: its back faces are hidden by its front ones
    std::vector<Triangle> triangles;
    std::vector<Triangle> coarse;  ///< the same part at a larger deflection, for frames drawn while interacting (may be empty)
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

    /// Re-reads the rig (roles, mobiles, meshes) from the project file and keeps the tessellated geometry, so a
    /// rig edit does not re-import the STEP file. Returns false when the parts themselves changed (a different
    /// part list or dial axis); then the scene is left as it was and a full load() is needed.
    bool reload_rig(const std::filesystem::path& project, double beat_rate_vph = 0.0);
    /// The same from a rig already in memory (an edit made in this process), with no file to read.
    bool reload_rig(model::Movement movement, double beat_rate_vph = 0.0);

    /// What is left to rig, worked out whenever the rig is loaded: the issues `ohs rig check` reports, how
    /// the gear chain runs from the barrel to the escape wheel, and meshes worth declaring.
    struct RigReview {
        std::vector<rigging::Issue> issues;
        rigging::TrainChain chain;
        std::vector<rigging::MeshSuggestion> suggestions;
        rigging::TrainWalk walk;  ///< the steps from the barrel to the escape wheel, or to what is missing
    };
    const RigReview& review() const { return review_; }

    const model::Movement& movement() const { return movement_; }
    const kinematics::TrainAnalysis& train() const { return train_; }
    const std::vector<ScenePart>& parts() const { return parts_; }

    /// Counts the changes that alter what draw_list() returns (time, exploded factor, cut-away, visibility, field,
    /// rig); a view that caches the list draws it again only when this moves.
    std::uint64_t revision() const { return revision_; }

    void set_time(double seconds) {
        time_ = seconds;
        ++revision_;
    }
    double time() const { return time_; }
    void set_stepped(bool stepped) {
        stepped_ = stepped;
        ++revision_;
    }
    void set_explode(double factor) {
        explode_ = factor;
        ++revision_;
    }
    double explode() const { return explode_; }
    void set_section(const SectionPlane& plane) {
        section_ = plane;
        ++revision_;
    }
    const SectionPlane& section() const { return section_; }
    void set_visible(const std::string& part, bool visible);
    bool visible(const std::string& part) const;
    bool can_animate() const { return train_.root_period_hours > 0.0; }
    /// Why the going train could not be analysed (empty when it could).
    const std::string& train_error() const { return train_error_; }

    /// Draws a result field on one part, replacing its CAD surface (the finite element mesh of a part
    /// carries the part's own placement, so the overlay turns with the part).
    void set_field(FieldOverlay field) {
        field_ = std::move(field);
        has_field_ = true;
        ++field_serial_;
        ++revision_;
    }
    void clear_field() {
        has_field_ = false;
        field_ = {};
        ++field_serial_;
        ++revision_;
    }
    const FieldOverlay* field() const { return has_field_ ? &field_ : nullptr; }

    /// The tooth force at one gear mesh: the driver's rim pushes the driven wheel along `direction` (a unit
    /// vector in the dial plane) at the pitch point `at`.
    struct ForceArrow {
        std::string driver, driven;  ///< mobile names
        Point3 at{};
        Point3 direction{};
        double newtons{};
    };
    /// Computes the tangential tooth force at every mesh on the path from the barrel to the escape wheel for
    /// a state of wind between 0 and 1 (torque of the driven mobile over the pitch radius of the wheel it
    /// meshes with), with the default calibre for the movement's beat rate. Throws ohs::Error when the torque
    /// chain cannot be built (no barrel, no path to the escape wheel).
    void compute_forces(double wind = 1.0);
    const std::vector<ForceArrow>& forces() const { return forces_; }
    void set_show_forces(bool on) { show_forces_ = on; }
    bool show_forces() const { return show_forces_; }

    /// A marker on the rim of a turning mobile and where it has been over the last `trail_seconds` of movement
    /// time (at most one turn), oldest point first, in world space.
    struct Trail {
        std::string mobile;
        std::vector<Point3> points;
    };
    void set_show_trails(bool on, double trail_seconds = 10.0) {
        show_trails_ = on;
        trail_seconds_ = trail_seconds;
    }
    bool show_trails() const { return show_trails_; }
    double trail_seconds() const { return trail_seconds_; }
    std::vector<Trail> trails(int samples = 48) const;

    /// Where one part is now: `m` holds the rows of the affine map [R | t] (3 x 4, row major) that takes the part's
    /// stored triangles (in its pivot frame) to world space: the turn of its mobile, or of the stem, lever or spring
    /// about its own axis, and the lift of the exploded view. The cut-away is not part of it. draw_list() places
    /// every point with this, and a GPU renderer keeps the geometry still and moves it with the matrix.
    struct PartPose {
        std::array<double, 12> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
        bool visible{true};
    };
    std::vector<PartPose> part_poses() const;
    /// Moves when the stored triangles of the parts change (a load, or a rig edit that moved a pivot): a GPU copy of
    /// them is then stale. Time, explode, cut-away, visibility and rig edits that move nothing leave it alone.
    std::uint64_t geometry_revision() const { return geometry_revision_; }
    /// Moves when the field overlay is set or cleared.
    std::uint64_t field_serial() const { return field_serial_; }

    /// Triangles of every visible part in world space at the current time, exploded and cut.
    /// `part_index[i]` is the index into parts() of triangle i; `values[i]` the field at its corners
    /// (NaN where there is no field).
    struct Drawn {
        std::vector<Triangle> triangles;
        std::vector<int> part_index;
        std::vector<std::array<double, 3>> values;
    };
    Drawn draw_list(bool coarse = false) const;

    /// Bounding box centre and radius of the assembled (unexploded) movement. Worked out once when the
    /// geometry is loaded: the viewport asks for them for every point it projects.
    const Point3& centre() const { return centre_; }
    double radius() const { return radius_; }

private:
    void update_bounds();
    void update_review();

    RigReview review_;
    model::Movement movement_;
    kinematics::TrainAnalysis train_;
    std::vector<ScenePart> parts_;
    std::string train_error_;
    std::unordered_set<std::string> hidden_;  ///< looked up for every part on every frame
    std::uint64_t revision_{0};
    std::uint64_t geometry_revision_{1};
    std::uint64_t field_serial_{0};
    double time_{0};
    bool stepped_{false};
    double explode_{0};
    SectionPlane section_;
    FieldOverlay field_;
    bool has_field_{false};
    std::vector<ForceArrow> forces_;
    bool show_forces_{false};
    bool show_trails_{false};
    double trail_seconds_{10.0};
    Point3 centre_{0, 0, 0};
    double radius_{1.0};
};

}  // namespace ohs::gui
