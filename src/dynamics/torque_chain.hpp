// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "kinematics/kinematics.hpp"
#include "model/model.hpp"

namespace ohs::dynamics {

/// One mobile on the path from the barrel to the escape wheel.
struct TorqueStage {
    std::string mobile;
    int meshes_from_barrel{};
    double speed_ratio{};   ///< |angular speed| relative to the barrel
    double fraction{};      ///< share of the barrel torque that reaches this mobile (efficiency / speed ratio)
};

/// How torque flows from the barrel to the escape wheel. Torque falls by the speed-up of every
/// step and by the efficiency of every mesh; the product is independent of how the path is cut.
struct TorqueChain {
    std::vector<TorqueStage> stages;  ///< barrel mobile first, escape wheel mobile last
    int meshes{};
    double speed_ratio{};             ///< escape wheel speed / barrel speed
    double efficiency{};              ///< mesh_efficiency^meshes
    int escape_teeth{};
    double barrel_period_hours{};     ///< 0 when the beat rate is unknown

    /// Fraction of the barrel torque that arrives at the escape wheel (before pivot losses).
    double transmission() const { return efficiency / speed_ratio; }
};

/// Follows the meshes from the root mobile to the escape wheel. Throws ohs::Error when the
/// escape wheel is not connected to the barrel.
TorqueChain build_chain(const model::Movement& movement, const kinematics::TrainAnalysis& train,
                        const Calibre& calibre);

/// A chain described by its totals only, for tests and calibres without a rig.
TorqueChain direct_chain(double speed_ratio, int meshes, const Calibre& calibre, int escape_teeth,
                         double barrel_period_hours);

/// Torque at the escape wheel in N·m for a state of wind between 0 and 1.
double escape_torque_nm(const TorqueChain& chain, const Calibre& calibre, double wind);

struct TorqueRow {
    std::string mobile;
    double torque_nmm{};  ///< at this mobile
    double lost_nmm{};    ///< friction loss in the mesh that feeds it
};

/// Torque at every stage and the loss in every mesh for a state of wind.
std::vector<TorqueRow> torque_table(const TorqueChain& chain, const Calibre& calibre, double wind);

/// Mean power delivered to the escape wheel in W: torque times the wheel's angular speed.
double escape_power_w(const TorqueChain& chain, const Calibre& calibre, double wind);

/// Copies the escape tooth count and beat rate that the rig fixes into a calibre.
void adopt_train(Calibre& calibre, const TorqueChain& chain, double beat_rate_vph);

}  // namespace ohs::dynamics
