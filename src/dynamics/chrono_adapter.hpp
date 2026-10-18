// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "dynamics/detailed.hpp"

namespace ohs::dynamics {

/// True when OHS was built with Project Chrono (`-DOHS_WITH_CHRONO=ON`).
bool chrono_available();

/// Version string of the linked Chrono library, empty when it is not built in.
const char* chrono_version();

/// Level-3 simulation on Project Chrono. The same physical model as `simulate_detailed`, the
/// same parameters and the same events, but the dynamics are Chrono's: the balance and the escape
/// wheel are `ChShaft` bodies advanced by Chrono's system and timestepper, the hairspring is a
/// `ChShaftsTorsionSpring`, the mainspring drive a `ChShaftsAppliedTorque`, and the escapement
/// (the wheel/pallet contact with the lever kinematics, pivot friction, unpoise and unlocking) is a
/// `ChShaftsTorque` element. Throws ohs::Error when Chrono is not built in.
RunSummary simulate_chrono(const Calibre& calibre, const DetailedConditions& conditions, const EventSink& sink);

}  // namespace ohs::dynamics
