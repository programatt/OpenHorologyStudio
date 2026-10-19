// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ohs::model {

struct Vec2 {
    double x{};
    double y{};
};

enum class Role {
    Unknown,
    Plate,
    Bridge,
    Barrel,
    Mainspring,
    Arbor,
    Wheel,
    Pinion,
    EscapeWheel,
    PalletFork,
    Balance,
    Hairspring,
    Roller,
    Jewel,
    Screw,
};

std::string_view to_string(Role role);
std::optional<Role> role_from_string(std::string_view text);

/// True for roles that carry gear teeth.
bool is_toothed_role(Role role);
/// True for roles that rotate with a mobile.
bool is_rotating_role(Role role);

/// Geometric facts extracted from one B-rep part. The dial axis is Z.
struct PartFeatures {
    std::string name;
    double volume{};
    Vec2 center{};          ///< centre of mass projected on the dial plane
    double z_min{};
    double z_max{};
    double extent_x{};
    double extent_y{};
    double outer_radius{};  ///< largest radius from the centre in the mid-height section
    double root_radius{};   ///< smallest radius of the toothed outline (0 when not toothed)
    double bore_radius{};   ///< 0 when the part has no central bore
    double flank_base_radius{};  ///< base circle of the best-fit involute flank (0 if no fit)
    double flank_rms{-1};        ///< rms distance of the flank from that fit in mm (-1 if no fit)
    double flank_cycloid_rms{-1};          ///< rms distance from the best cycloidal fit (-1 if not tried or no fit)
    double flank_cycloid_pitch_radius{};   ///< pitch circle of that fit, mm
    double flank_cycloid_rolling_radius{}; ///< rolling circle of that fit, mm
    int teeth{};            ///< 0 when no toothing was detected
    double tooth_confidence{};
    std::uint64_t fingerprint{};

    double thickness() const { return z_max - z_min; }
};

/// Gear parameters fitted to a toothed part and its meshes.
struct GearFit {
    double module{};
    std::string module_source;  ///< "mesh" (from centre distances) or "tip" (assumed one-module addendum)
    double addendum{};          ///< addendum height in modules
    double dedendum{};          ///< dedendum depth in modules
    double base_radius{};       ///< 0 when the flank fit failed
    double pressure_angle_deg{};
    double rolling_radius{};    ///< rolling circle in modules; cycloidal profiles only
    std::string profile;        ///< "involute", "cycloidal" or "unknown"
};

struct Component {
    PartFeatures features;
    std::optional<GearFit> gear;
    Role role{Role::Unknown};
    double role_confidence{};
    std::string role_source;  ///< "preset", "name", "geometry" or "user"
};

/// Components that rotate together about one axis.
struct Mobile {
    std::string name;
    std::vector<int> components;  ///< indices into Movement::components
    Vec2 center{};
};

/// Gear mesh between two toothed components of different mobiles.
struct MeshJoint {
    int a{};
    int b{};
    double centre_distance{};
    double module{};
    double confidence{};
    bool manual{false};  ///< declared by the user (`ohs rig mesh`, a preset), kept when the rig is re-inferred
};

struct Movement {
    std::string name;
    std::string source_geometry;  ///< path of the CAD file, relative to the project when possible
    double beat_rate_vph{};       ///< 0 when unknown
    std::string dial_axis{"z"};   ///< CAD axis normal to the dial ("x", "y" or "z"), found or given at import
    std::vector<Component> components;
    std::vector<Mobile> mobiles;
    std::vector<MeshJoint> meshes;

    /// Index of the mobile holding component `component`, or -1.
    int mobile_of(int component) const;
    /// Index of the component called `name`, or -1.
    int find_component(std::string_view name) const;
};

/// Gear module estimated from the tip radius assuming a standard addendum of one module.
double module_from_tip(double outer_radius, int teeth);

}  // namespace ohs::model
