// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace ohs::dynamics {

/// Gravity orientation of the watch. In the flat positions the balance staff is vertical, so
/// gravity exerts no torque on an unpoised balance and the pivots run on their end stones. In
/// the four hanging positions the staff is horizontal: the unpoise gives a gravity torque and the
/// pivots carry the weight on their sides (more friction).
enum class Position { DialUp, DialDown, CrownUp, CrownDown, CrownLeft, CrownRight };

const char* to_string(Position p);
Position position_from_string(const std::string& text);  // throws ohs::Error
const std::vector<Position>& all_positions();
bool is_vertical(Position p);

/// The escapement the reduced model describes.
enum class EscapementType {
    SwissLever,  ///< an impulse in every swing of the balance: two beats per oscillation, tick and tock
    Detent,      ///< spring detent (chronometer): one impulse per oscillation, in one swing only; on the
                 ///< other swing the roller brushes a passing spring aside and the wheel stays locked
};

const char* to_string(EscapementType e);
EscapementType escapement_from_string(const std::string& text);  // "swiss_lever" or "detent"; throws ohs::Error

/// Everything the dynamics models need beyond the geometry in the rig. Units are what a
/// watchmaker reads on a data sheet; the models convert to SI internally.
struct Calibre {
    std::string name = "reference lever calibre";

    // Mainspring and barrel
    double barrel_torque_nmm = 1.10;     ///< torque at the barrel arbor when fully wound
    double barrel_turns = 6.25;          ///< turns of unwinding available
    /// Normalised torque against state of wind (1 = full, 0 = run down), piecewise linear.
    std::vector<std::pair<double, double>> torque_curve = {{0.0, 0.0}, {0.04, 0.74}, {0.5, 0.88}, {1.0, 1.0}};

    // Train losses
    double mesh_efficiency = 0.90;       ///< torque transmitted by one gear mesh
    double escape_pivot_loss_nmm = 0.0;  ///< constant friction torque at the escape wheel arbor

    // Balance and hairspring
    double balance_inertia_mgcm2 = 5.0;      ///< balance + hairspring inertia
    double balance_mass_mg = 800.0;
    double unpoise_um = 0.08;                ///< centre of mass offset from the staff axis
    double unpoise_angle_deg = 40.0;         ///< where that offset sits on the balance
    double beat_offset_deg = 0.0;            ///< hairspring rest position relative to the lever centre
    double hairspring_youngs_gpa = 210.0;
    double hairspring_width_mm = 0.14;
    double hairspring_thickness_mm = 0.025;
    double hairspring_length_mm = 0.0;       ///< 0: derive from the beat rate (default_calibre does)
    /// Cubic term of the restoring torque, N·m/rad^3: torque = -(k a + c a^3) with a the angle from rest. 0 is
    /// a perfectly linear spring; `ohs calibre hairspring` fits it from a beam model of the spiral.
    double hairspring_cubic_nm_rad3 = 0.0;
    double air_damping_nms = 1.0e-11;        ///< viscous damping torque per rad/s, N·m·s
    double pivot_friction_flat_nm = 8.0e-10; ///< Coulomb friction torque on the balance, dial up/down
    double vertical_friction_factor = 1.3;   ///< multiplier in the hanging positions

    // Escapement
    EscapementType escapement = EscapementType::SwissLever;
    double escape_teeth = 15;                ///< escape wheel teeth (one tooth passes per two beats)
    double impulse_lead_deg = 4.0;           ///< impulse starts this far before the dead point (balance angle)
    double lift_angle_deg = 50.0;           ///< balance angle during which the lever pushes it
    double unlock_angle_deg = 6.0;           ///< balance angle spent unlocking, before the impulse
    double drop_angle_deg = 1.0;             ///< escape wheel angle lost to drop and run
    double impulse_efficiency = 0.92;        ///< share of the wheel's work that reaches the balance
    double unlock_friction_ratio = 0.30;     ///< unlocking resistance as a share of the impulse torque
    /// Detent only: the drag of the passing spring on the balance during the return swing, over the
    /// unlocking angle, as a share of the impulse torque.
    double passing_spring_ratio = 0.25;
    double escape_wheel_inertia_mgcm2 = 0.05;
    double contact_damping_ratio = 0.8;      ///< detailed model: damping of the wheel/pallet contact
    double contact_frequency_hz = 12000.0;   ///< detailed model: stiffness of the wheel/pallet contact

    double beat_rate_vph = 25200;
    double min_running_amplitude_deg = 150.0;

    // Derived
    double inertia() const;                  ///< kg·m²
    double escape_wheel_inertia() const;     ///< kg·m²
    double hairspring_stiffness() const;     ///< N·m/rad
    double natural_frequency_hz() const;     ///< free oscillation frequency of balance + hairspring
    double unpoise_torque() const;           ///< m·g·e in N·m
    double mainspring_torque_nmm(double wind) const;  ///< 0 <= wind <= 1
    double friction_torque(Position p) const;
    double beat_period_s() const { return 3600.0 / beat_rate_vph; }
    /// Impulses the escapement gives in one oscillation (two beats): 2 for the lever, 1 for the detent.
    int impulses_per_oscillation() const { return escapement == EscapementType::Detent ? 1 : 2; }
    /// The time between impulses at the nominal beat rate.
    double nominal_impulse_interval_s() const { return 2.0 * beat_period_s() / impulses_per_oscillation(); }
};

/// A calibre with the hairspring length chosen so the free frequency is half the beat rate.
Calibre default_calibre(double beat_rate_vph = 25200);

/// Reads `[calibre]` overrides from TOML text on top of default_calibre(); throws ohs::Error.
Calibre calibre_from_toml(const std::string& text);
Calibre load_calibre(const std::string& path);
std::string to_toml(const Calibre& calibre);

/// Named scalar parameters for what-if studies and sweeps. Returns a reference into `c`; throws
/// ohs::Error for an unknown name.
double& parameter(Calibre& c, const std::string& name);
const std::vector<std::string>& parameter_names();

}  // namespace ohs::dynamics
