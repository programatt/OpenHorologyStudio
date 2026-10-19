// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "model/model.hpp"

namespace ohs::model {

// Levers and springs that ride on toothed parts: a click on a ratchet wheel, a jumper on a date ring, a
// spring pressing on either. What a part rides on is read from the part features in the rig, so the
// classifier (to tell a lever from a plate) and the kinematics (to move it) share it.

/// True for a part with teeth that rotates: outside (wheels, pinions, the barrel) or inside (a date ring).
bool is_follow_target(const Component& c);

/// The band of radii in which a follower meets the teeth of a part: tips to roots, whichever side they are on.
struct ToothBand {
    double lo{};
    double hi{};
    double depth() const { return hi - lo; }
};
ToothBand tooth_band(const Component& c);

/// What a lever rides on and where it turns.
struct LeverContact {
    int driver{-1};  ///< component index of the toothed part
    Vec2 pivot{};    ///< the hole the lever swings about
    Vec2 tip{};      ///< where it meets the teeth (estimated from the far end of the part)
    double arm{};    ///< pivot to tip, mm
    double fit{};    ///< 0 when the tip is in the middle of the band of the teeth, 1 at the limit
};

/// The toothed part the lever `index` rides on and the hole it pivots on, or nothing when no hole of it puts
/// its tip on the teeth of a part it overlaps in height. The pivot is the hole that fits best, and a hole
/// that another part is centred in counts for more. Only parts for which `usable` (given the component index)
/// is true are looked at as the toothed part, when it is given.
using Usable = std::function<bool(int)>;
std::optional<LeverContact> lever_contact(const std::vector<Component>& components, int index, const Usable& usable = {});

/// The turn of a lever about its pivot, in radians, when its tip is pushed over the teeth as far as they go:
/// positive is counter-clockwise seen from +Z. Teeth push a tip away from the centre of a wheel and towards
/// the centre of a ring with teeth on the inside.
double lever_swing(const std::vector<Component>& components, const LeverContact& contact);

/// How a spring moves: fixed at one end, bent at the other as what it presses on moves.
struct SpringFlex {
    int driver{-1};    ///< component index of the toothed part whose turning bends it
    int lever{-1};     ///< the lever it presses, or -1 when it presses the teeth itself
    Vec2 fixed{};      ///< the end that stays put
    Vec2 free_end{};   ///< the end that moves
    double length{};   ///< fixed to free end, mm
    double depth{};    ///< how far its free end is pushed at the most, mm
    double swing{};    ///< turn about `fixed` for that push, radians (signed)
};

/// What the spring `index` presses on, or nothing when it touches neither a moving lever nor a toothed part.
/// `usable` is as for `lever_contact`.
std::optional<SpringFlex> spring_flex(const std::vector<Component>& components, int index, const Usable& usable = {});

}  // namespace ohs::model
