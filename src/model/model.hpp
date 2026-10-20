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
    /// The oscillating weight of an automatic movement. It turns about the movement's centre, but its
    /// winding is not modelled yet, so it is not a rotating role: it joins no mobile and is drawn still.
    Rotor,
    /// A calendar ring (date wheel) with internal teeth. It turns about the movement's centre on the
    /// mainplate, held by its edge or by guides rather than by an arbor, and a finger on the 24-hour
    /// wheel pushes it round one tooth a day. It is always a mobile of its own, however its centre
    /// lines up with the going train, and it is not geared to the train, so it has no meshes.
    DateRing,
    /// The winding stem: a long rod on an axis in the plane of the movement (the crown's), which turns about
    /// its own axis. It is not a mobile of the train: it is posed as turning with the winding of the barrel.
    Stem,
    /// A spring that is not the mainspring or the hairspring: a click, jumper, setting or detent spring, of
    /// bent wire or cut from plate. It is fixed at one end and bends at the other as what it presses on moves.
    Spring,
    /// A plate that swings about a pivot, such as a click, a pawl, a jumper, a setting or clutch lever or a
    /// yoke. It is not a mobile of the train: it swings as the toothed wheel it rides on turns.
    Lever,
    /// A brass tube or bush pressed into a plate or bridge, which an arbor or pinion runs in. It is fixed.
    Bearing,
};

/// The last role, for going through every role (`Role::Unknown` is the first).
inline constexpr Role kLastRole = Role::Bearing;

std::string_view to_string(Role role);
std::optional<Role> role_from_string(std::string_view text);

/// True for roles that carry gear teeth.
bool is_toothed_role(Role role);
/// True for roles that rotate with a mobile.
bool is_rotating_role(Role role);

/// A round hole through a part along the dial axis.
struct Hole {
    double x{};
    double y{};
    double radius{};
};

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
    int inner_teeth{};      ///< teeth on the inside of a ring (a date wheel); 0 when it has none
    double inner_tip_radius{};   ///< smallest radius of the internal teeth, the tips (0 when none)
    double inner_root_radius{};  ///< largest radius of the internal teeth, the roots (0 when none)
    double tooth_confidence{};
    std::uint64_t fingerprint{};
    /// The oriented bounding box: the direction of its longest side (a unit vector, dial axis Z) and its
    /// three sides, longest first. All 0 for a rig written before they were recorded.
    double axis[3]{};
    double length{};
    double width{};
    double height{};
    /// Round holes along the dial axis, largest first (at most 8): where a lever pivots or a screw passes.
    std::vector<Hole> holes;

    double thickness() const { return z_max - z_min; }
};

/// The gear code refuses a pressure angle at or above this, so a fit that gives one is not stored.
inline constexpr double kMaxPressureAngleDeg = 45.0;

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

/// True when the fit carries a pressure angle worth using (an older project may hold one outside the range).
inline bool has_pressure_angle(const GearFit& g) {
    return g.pressure_angle_deg > 5 && g.pressure_angle_deg < kMaxPressureAngleDeg;
}

struct Component {
    PartFeatures features;
    std::optional<GearFit> gear;
    Role role{Role::Unknown};
    double role_confidence{};
    std::string role_source;  ///< "preset", "name", "geometry" or "user"
    std::string label;        ///< a name the user gave the part, shown in place of the part name; empty for none

    /// What to call the part on screen: its label when it has one, else its part name.
    const std::string& display_name() const { return label.empty() ? features.name : label; }
};

/// True for a part of the winding work (a ratchet, click, crown, winding or clutch part, by its name or label).
bool is_winding_part(const Component& component);

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
    bool winding{false}; ///< not a link of the going train (the winding work, say): `ohs rig winding`, kept when re-inferred
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
    /// For every component the index of its mobile, or -1: mobile_of for all of them in one pass, for loops over
    /// pairs of parts (mobile_of alone scans every mobile each time).
    std::vector<int> mobile_index() const;
    /// Index of the component called `name`, or -1.
    int find_component(std::string_view name) const;
    /// True when the mesh is a link of the going train, which runs from the barrel to the escape wheel. Not so a mesh
    /// the user marked as winding, nor one with a part of the winding work (a ratchet or click wheel, a crown wheel):
    /// that one turns the barrel, so it is the wrong way round for the train.
    bool in_train(const MeshJoint& joint) const;
};

/// Gear module estimated from the tip radius assuming a standard addendum of one module.
double module_from_tip(double outer_radius, int teeth);

}  // namespace ohs::model
