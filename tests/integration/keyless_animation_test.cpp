// SPDX-License-Identifier: Apache-2.0
// The stem, a click and its spring, from CAD geometry to the animation files.
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "animation/gltf_animation.hpp"
#include "app/import.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"
#include "keyless_parts.hpp"

using namespace ohs;
using Catch::Approx;
using model::Role;

TEST_CASE("a stem, a click and a spring are found in CAD geometry and animated about their own axes") {
    const auto dir = std::filesystem::temp_directory_path() / "ohs_keyless_anim_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto step = dir / "train.step";
    geometry::export_step(test::train_with_keyless_work(), step);

    app::ImportOptions opt;
    opt.rig.beat_rate_vph = 25200;
    const model::Movement m = app::import_movement(step, opt);
    CHECK(m.components[static_cast<std::size_t>(m.find_component("part_a"))].role == Role::Stem);
    CHECK(m.components[static_cast<std::size_t>(m.find_component("part_b"))].role == Role::Lever);
    CHECK(m.components[static_cast<std::size_t>(m.find_component("part_c"))].role == Role::Spring);

    // The features survive the rig file.
    const auto saved = dir / "train.hsproj";
    model::save_rig(m, saved);
    const model::Movement again = model::load_rig(saved);
    const auto& lever = again.components[static_cast<std::size_t>(again.find_component("part_b"))].features;
    CHECK(lever.length == Approx(3.6).epsilon(0.01));
    REQUIRE(lever.holes.size() == 1);
    CHECK(lever.holes[0].x == Approx(3.3).epsilon(1e-3));
    CHECK(lever.holes[0].radius == Approx(0.3).epsilon(1e-3));
    const auto& rod = again.components[static_cast<std::size_t>(again.find_component("part_a"))].features;
    CHECK(rod.axis[0] == Approx(1.0).margin(1e-6));
    CHECK(rod.length == Approx(17.7).epsilon(0.002));

    const auto parts = geometry::import_step(step);
    const auto train = kinematics::analyze_train(again, again.beat_rate_vph);
    REQUIRE(train.articulations.size() == 3);

    animation::AnimationOptions ao;
    ao.duration_seconds = 5.0;
    ao.time_scale = 600.0;  // 3000 s of movement time: the barrel turns 37.5 degrees
    const auto glb = animation::write_animated_glb(again, parts, train, geometry::Axis::Z, ao, dir / "train.glb");
    CHECK(glb.animated_nodes == 14 + 3);
    CHECK(glb.max_degrees_per_keyframe <= 90.0 + 1e-6);

    const auto usd = animation::write_animated_usd(again, parts, train, geometry::Axis::Z, ao, dir / "train.usda");
    CHECK(usd.animated_nodes == 14 + 3);
    std::ifstream in(dir / "train.usda");
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    auto count = [&](const std::string& needle) {
        std::size_t n = 0;
        for (std::size_t pos = 0; (pos = text.find(needle, pos)) != std::string::npos; pos += needle.size()) ++n;
        return n;
    };
    CHECK(count("xformOp:rotateZ.timeSamples") == 14);  // the train, as before
    CHECK(count("xformOp:orient.timeSamples") == 3);

    // The last quaternion (w, x, y, z) of a part.
    auto last_orient = [&](const std::string& doc_name) {
        const std::size_t part = text.find("doc = \"" + doc_name + "\"");
        REQUIRE(part != std::string::npos);
        const std::size_t end = text.find("        }\n", text.find("orient.timeSamples", part));
        const std::size_t open = text.rfind(": (", end);
        double q[4];
        std::sscanf(text.c_str() + open + 3, "%lf, %lf, %lf, %lf", &q[0], &q[1], &q[2], &q[3]);
        return std::array<double, 4>{q[0], q[1], q[2], q[3]};
    };
    // The stem has turned six times the barrel's 37.5 degrees backwards, about the x axis.
    const auto stem = last_orient("part_a");
    CHECK(stem[1] == Approx(std::sin(-6 * 37.5 / 2 * std::numbers::pi / 180)).epsilon(1e-4));
    CHECK(stem[2] == Approx(0.0).margin(1e-9));
    CHECK(stem[3] == Approx(0.0).margin(1e-9));
    // The click turns about z, clockwise because the teeth push its tip out, a few degrees at most; the
    // spring bends the same way about its fixed end.
    for (const char* name : {"part_b", "part_c"}) {
        const auto q = last_orient(name);
        INFO(name);
        CHECK(q[1] == Approx(0.0).margin(1e-9));
        CHECK(q[2] == Approx(0.0).margin(1e-9));
        CHECK(q[3] <= 0.0);
        CHECK(q[3] > -0.05);
    }
    // The nodes of the parts sit on their pivots, in metres' worth of millimetres.
    const auto click_node = text.find("doc = \"part_b\"");
    REQUIRE(click_node != std::string::npos);
    const auto translate = text.find("xformOp:translate = (3.3", click_node);
    CHECK(translate != std::string::npos);
    CHECK(translate - click_node < 200);

    std::filesystem::remove_all(dir);
}
