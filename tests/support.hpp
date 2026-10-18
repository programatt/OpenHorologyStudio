// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "model/model.hpp"

namespace ohs::test {

inline model::PartFeatures gear(std::string name, int teeth, double module, double x, double y, double z0,
                                double z1) {
    model::PartFeatures f;
    f.name = std::move(name);
    f.teeth = teeth;
    f.tooth_confidence = 1.0;
    f.outer_radius = (teeth + 2) * module / 2.0;
    f.bore_radius = 0.1;
    f.center = {x, y};
    f.z_min = z0;
    f.z_max = z1;
    f.extent_x = f.extent_y = 2 * f.outer_radius;
    f.volume = 1.0;
    return f;
}

inline model::PartFeatures arbor(std::string name, double x, double y, double z0, double z1) {
    model::PartFeatures f;
    f.name = std::move(name);
    f.outer_radius = f.extent_x = f.extent_y = 0.1;
    f.center = {x, y};
    f.z_min = z0;
    f.z_max = z1;
    f.volume = 0.1;
    return f;
}

/// Same layout as geometry::make_test_movement(), as plain features.
inline std::vector<model::PartFeatures> synthetic_train() {
    constexpr double m = 0.15;
    constexpr double deg = std::numbers::pi / 180.0;
    std::vector<model::PartFeatures> p;
    double x = 0, y = 0;
    p.push_back(gear("barrel", 80, m, x, y, 0.0, 1.0));
    p.push_back(arbor("barrel_arbor", x, y, -0.2, 1.2));
    x += m * 45;
    p.push_back(gear("center_pinion", 10, m, x, y, 0.0, 1.0));
    p.push_back(gear("center_wheel", 80, m, x, y, 1.2, 2.0));
    p.push_back(arbor("center_arbor", x, y, -0.2, 2.2));
    x += m * 45 * std::cos(120 * deg);
    y += m * 45 * std::sin(120 * deg);
    p.push_back(gear("third_pinion", 10, m, x, y, 1.2, 2.0));
    p.push_back(gear("third_wheel", 75, m, x, y, 2.2, 3.0));
    p.push_back(arbor("third_arbor", x, y, 1.0, 3.2));
    x += m * 42.5 * std::cos(20 * deg);
    y += m * 42.5 * std::sin(20 * deg);
    p.push_back(gear("fourth_pinion", 10, m, x, y, 2.2, 3.0));
    p.push_back(gear("fourth_wheel", 84, m, x, y, 3.2, 4.0));
    p.push_back(arbor("fourth_arbor", x, y, 2.0, 4.2));
    x += m * 45 * std::cos(60 * deg);
    y += m * 45 * std::sin(60 * deg);
    p.push_back(gear("escape_pinion", 6, m, x, y, 3.2, 4.0));
    p.push_back(gear("escape_wheel", 15, m, x, y, 4.2, 5.0));
    p.push_back(arbor("escape_arbor", x, y, 3.0, 5.2));
    model::PartFeatures plate;
    plate.name = "main_plate";
    plate.extent_x = plate.extent_y = 40;
    plate.z_min = -0.8;
    plate.z_max = -0.3;
    plate.volume = 100;
    p.push_back(plate);
    return p;
}

}  // namespace ohs::test
