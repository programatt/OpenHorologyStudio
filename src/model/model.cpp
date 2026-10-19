// SPDX-License-Identifier: Apache-2.0
#include "model/model.hpp"

#include <array>
#include <cctype>
#include <utility>

namespace ohs::model {

namespace {
constexpr std::array<std::pair<Role, std::string_view>, 21> kRoleNames{{
    {Role::Unknown, "unknown"},
    {Role::Plate, "plate"},
    {Role::Bridge, "bridge"},
    {Role::Barrel, "barrel"},
    {Role::Mainspring, "mainspring"},
    {Role::Arbor, "arbor"},
    {Role::Wheel, "wheel"},
    {Role::Pinion, "pinion"},
    {Role::EscapeWheel, "escape_wheel"},
    {Role::PalletFork, "pallet_fork"},
    {Role::Balance, "balance"},
    {Role::Hairspring, "hairspring"},
    {Role::Roller, "roller"},
    {Role::Jewel, "jewel"},
    {Role::Screw, "screw"},
    {Role::Rotor, "rotor"},
    {Role::DateRing, "date_ring"},
    {Role::Stem, "stem"},
    {Role::Spring, "spring"},
    {Role::Lever, "lever"},
    {Role::Bearing, "bearing"},
}};
}  // namespace

std::string_view to_string(Role role) {
    for (const auto& [r, name] : kRoleNames) {
        if (r == role) return name;
    }
    return "unknown";
}

std::optional<Role> role_from_string(std::string_view text) {
    for (const auto& [r, name] : kRoleNames) {
        if (name == text) return r;
    }
    return std::nullopt;
}

bool is_toothed_role(Role role) {
    return role == Role::Wheel || role == Role::Pinion || role == Role::EscapeWheel ||
           role == Role::Barrel;
}

bool is_rotating_role(Role role) {
    return is_toothed_role(role) || role == Role::Arbor || role == Role::Balance ||
           role == Role::Roller || role == Role::DateRing;
}

namespace {

// Parts of the winding work, which a mesh of the going train never passes through.
bool is_winding_part(const Component& c) {
    for (const std::string* text : {&c.label, &c.features.name}) {
        std::string lower(*text);
        for (char& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        for (const char* word : {"ratchet", "click", "crown", "winding", "clutch", "sliding pinion"}) {
            if (lower.find(word) != std::string::npos) return true;
        }
    }
    return false;
}

}  // namespace

bool Movement::in_train(const MeshJoint& joint) const {
    return !joint.winding && !is_winding_part(components[static_cast<std::size_t>(joint.a)]) &&
           !is_winding_part(components[static_cast<std::size_t>(joint.b)]);
}

int Movement::mobile_of(int component) const {
    for (std::size_t m = 0; m < mobiles.size(); ++m) {
        for (int c : mobiles[m].components) {
            if (c == component) return static_cast<int>(m);
        }
    }
    return -1;
}

int Movement::find_component(std::string_view name) const {
    for (std::size_t i = 0; i < components.size(); ++i) {
        if (components[i].features.name == name) return static_cast<int>(i);
    }
    return -1;
}

double module_from_tip(double outer_radius, int teeth) {
    return teeth > 0 ? 2.0 * outer_radius / (teeth + 2) : 0.0;
}

}  // namespace ohs::model
