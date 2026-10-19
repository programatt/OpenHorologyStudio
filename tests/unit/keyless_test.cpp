// SPDX-License-Identifier: Apache-2.0
// The stem, springs, levers and bearings: how they are recognised, what they ride on and how they move.
#include <algorithm>
#include <cmath>
#include <numbers>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include "geometry/features.hpp"
#include "geometry/procedural.hpp"
#include "kinematics/kinematics.hpp"
#include "model/followers.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using Catch::Approx;
using model::Role;

namespace {

model::PartFeatures named(const std::string& name) {
    auto f = test::gear(name, 0, 0.15, 0, 0, 0, 1);  // a shapeless blob: neither toothed, plate-like nor arbor-like
    f.tooth_confidence = 0;
    f.outer_radius = 2.0;
    f.extent_x = f.extent_y = 4.0;
    return f;
}

// The barrel of the synthetic train has 80 teeth between radii 5.81 and 6.15 mm. A click lies along x with its
// pivot hole at (3.3, 6) and its tip at (0, 6), on the teeth at the top of the barrel.
model::PartFeatures click() {
    auto f = named("Body7");
    f.center = {1.8, 6.0};
    f.extent_x = 3.6;
    f.extent_y = 1.0;
    f.z_min = 0.3;
    f.z_max = 0.6;
    f.length = 3.6;
    f.width = 1.0;
    f.height = 0.3;
    f.axis[0] = 1.0;
    f.holes = {{3.3, 6.0, 0.3}};
    return f;
}

// A strip over the click, fixed at x = 7 and pressing on the click at x = 2.
model::PartFeatures click_spring() {
    auto f = named("Body8");
    f.center = {4.5, 6.8};
    f.extent_x = 5.0;
    f.extent_y = 0.4;
    f.z_min = 0.3;
    f.z_max = 0.5;
    f.volume = 0.7;  // nearly the whole of its 5 x 0.8 x 0.2 box
    f.length = 5.0;
    f.width = 0.8;
    f.height = 0.2;
    f.axis[0] = 1.0;
    return f;
}

model::PartFeatures stem_part() {
    auto f = named("Body1");
    f.outer_radius = 0.75;
    f.center = {0.0, 17.6};
    f.extent_x = 1.5;
    f.extent_y = 17.7;
    f.z_min = -0.75;
    f.z_max = 0.75;
    f.length = 17.7;
    f.width = f.height = 1.5;
    f.axis[0] = 1.0;
    return f;
}

std::vector<model::PartFeatures> train_with_keyless_work() {
    auto parts = test::synthetic_train();
    parts.push_back(click());
    parts.push_back(click_spring());
    parts.push_back(stem_part());
    return parts;
}

}  // namespace

TEST_CASE("the keyless work is found by name") {
    rigging::RigOptions o;
    CHECK(rigging::classify(named("Winding Stem"), o).role == Role::Stem);
    CHECK(rigging::classify(named("stem"), o).role == Role::Stem);
    CHECK(rigging::classify(named("system plate"), o).role == Role::Plate);  // "stem" only as a word
    CHECK(rigging::classify(named("click spring"), o).role == Role::Spring);
    CHECK(rigging::classify(named("Jumper spring"), o).role == Role::Spring);
    CHECK(rigging::classify(named("setting lever"), o).role == Role::Lever);
    CHECK(rigging::classify(named("yoke"), o).role == Role::Lever);
    CHECK(rigging::classify(named("ratchet click"), o).role == Role::Lever);
    CHECK(rigging::classify(named("winding pawl"), o).role == Role::Lever);
    CHECK(rigging::classify(named("date wheel jumper"), o).role == Role::Lever);
    CHECK(rigging::classify(named("brass bearing"), o).role == Role::Bearing);
    CHECK(rigging::classify(named("bush 0.6"), o).role == Role::Bearing);
    CHECK(rigging::classify(named("tube"), o).role == Role::Bearing);
    // What already had a role keeps it.
    CHECK(rigging::classify(named("pallet lever"), o).role == Role::PalletFork);
    CHECK(rigging::classify(named("Mainspring"), o).role == Role::Mainspring);
    CHECK(rigging::classify(named("balance spring"), o).role == Role::Hairspring);
    CHECK(rigging::classify(named("spring screw"), o).role == Role::Screw);
    CHECK(rigging::classify(named("bearing bridge"), o).role == Role::Bridge);
    CHECK(rigging::classify(named("jewel bearing"), o).role == Role::Jewel);
}

TEST_CASE("an unnamed rod in the plane of the movement is the stem, and a short tube is a bearing") {
    rigging::RigOptions o;
    auto rod = stem_part();
    rod.name = "Body1";
    CHECK(rigging::classify(rod, o).role == Role::Stem);  // not a plate, however long
    rod.axis[0] = 0.0;
    rod.axis[2] = 1.0;  // standing on the dial: not a stem
    CHECK(rigging::classify(rod, o).role != Role::Stem);

    auto tube = named("Body2");
    tube.outer_radius = 0.6;
    tube.bore_radius = 0.3;
    tube.extent_x = tube.extent_y = 1.2;
    tube.z_min = -1.7;
    tube.z_max = 0.5;
    const auto c = rigging::classify(tube, o);
    CHECK(c.role == Role::Bearing);
    CHECK(c.source == "geometry");
    tube.z_max = tube.z_min + 0.3;  // thin: a washer, not a tube
    CHECK(rigging::classify(tube, o).role == Role::Unknown);
    tube.z_max = 0.5;
    tube.bore_radius = 0.0;  // solid: a pin
    CHECK(rigging::classify(tube, o).role != Role::Bearing);
}

TEST_CASE("a flat part on the teeth of a wheel is a lever, and a strip pressing on it is a spring") {
    auto parts = test::synthetic_train();
    parts.push_back(click());
    parts.push_back(click_spring());
    const int lever = static_cast<int>(parts.size()) - 2, spring = lever + 1;
    const auto m = rigging::auto_rig("train", parts, {});
    CHECK(m.components[lever].role == Role::Lever);
    CHECK(m.components[lever].role_confidence <= 0.5);
    CHECK(m.components[lever].role_source == "geometry");
    CHECK(m.components[spring].role == Role::Spring);
    CHECK(m.mobile_of(lever) < 0);  // neither is a mobile of the train
    CHECK(m.mobile_of(spring) < 0);

    // The same parts away from the barrel touch no teeth: they are left for the user to name.
    auto away = test::synthetic_train();
    auto lone = click();
    lone.center.y += 30;
    lone.holes = {{3.3, 36.0, 0.3}};
    away.push_back(lone);
    auto loose = click_spring();
    loose.center.y += 30;
    away.push_back(loose);
    const auto n = rigging::auto_rig("train", away, {});
    CHECK(n.components[away.size() - 2].role == Role::Unknown);
    CHECK(n.components[away.size() - 1].role == Role::Unknown);
}

TEST_CASE("a bent wire is a spring on its shape") {
    auto parts = test::synthetic_train();
    auto wire = named("Body9");
    wire.center = {20, 20};
    wire.volume = 2.1;
    wire.length = 4.2;
    wire.width = 2.2;
    wire.height = 0.9;
    wire.z_min = -0.5;
    wire.z_max = 0.4;
    parts.push_back(wire);
    const auto m = rigging::auto_rig("train", parts, {});
    CHECK(m.components.back().role == Role::Spring);
    CHECK(m.components.back().role_confidence < 0.5);
    parts.back().volume = 20.0;  // a solid block of the same size is not
    CHECK(rigging::auto_rig("train", parts, {}).components.back().role == Role::Unknown);
}

TEST_CASE("check_rig says which levers and springs rest on nothing") {
    auto parts = train_with_keyless_work();
    auto m = rigging::auto_rig("train", parts, {});
    auto messages = [](const model::Movement& movement) {
        std::string all;
        for (const auto& i : rigging::check_rig(movement)) all += i.message + "\n";
        return all;
    };
    CHECK(messages(m).find("rests on no toothed part") == std::string::npos);
    CHECK(messages(m).find("presses on no lever") == std::string::npos);
    // A lever the user names, away from the teeth, and a spring away from everything.
    parts.push_back(named("setting lever"));
    parts.push_back(named("click spring"));
    const auto n = rigging::auto_rig("train", parts, {});
    CHECK(messages(n).find("lever 'setting lever' rests on no toothed part") != std::string::npos);
    CHECK(messages(n).find("spring 'click spring' presses on no lever or toothed part") != std::string::npos);
}

TEST_CASE("the pivot of a lever is the hole that puts its tip on the teeth") {
    auto parts = test::synthetic_train();
    auto lever = click();
    // Another hole, towards the tip: turning on it would put the far end of the part off the barrel, so it is not the pivot.
    lever.holes = {{1.0, 6.0, 0.5}, {3.3, 6.0, 0.3}};
    parts.push_back(lever);
    const auto m = rigging::auto_rig("train", parts, {});
    const int idx = static_cast<int>(parts.size()) - 1;
    const auto contact = model::lever_contact(m.components, idx);
    REQUIRE(contact);
    CHECK(contact->pivot.x == Approx(3.3));
    CHECK(contact->tip.x == Approx(0.0).margin(1e-9));
    CHECK(contact->arm == Approx(3.3));
    CHECK(m.components[static_cast<std::size_t>(contact->driver)].features.name == "barrel");
    CHECK(contact->fit < 0.5);
    // The tip is pushed outwards by the teeth, which turns this lever clockwise.
    CHECK(model::lever_swing(m.components, *contact) == Approx(-(6.15 - 5.8125) / 3.3).epsilon(0.01));
}

TEST_CASE("the stem, a click and its spring are articulated from the train") {
    const auto m = rigging::auto_rig("train", train_with_keyless_work(), {});
    const int n = static_cast<int>(m.components.size());
    const int click_idx = n - 3, spring_idx = n - 2, stem_idx = n - 1;
    REQUIRE(m.components[click_idx].role == Role::Lever);
    REQUIRE(m.components[spring_idx].role == Role::Spring);
    REQUIRE(m.components[stem_idx].role == Role::Stem);
    const auto t = kinematics::analyze_train(m, 25200);
    REQUIRE(t.articulations.size() == 3);

    auto of = [&](int component) -> const kinematics::Articulation& {
        for (const auto& a : t.articulations) {
            if (a.component == component) return a;
        }
        FAIL("no articulation for component " << component);
        return t.articulations.front();
    };
    // The stem turns about its own axis, through its middle, along the x axis.
    const auto& s = of(stem_idx);
    CHECK(s.kind == kinematics::Articulation::Kind::Spin);
    CHECK(s.axis[0] == Approx(1.0));
    CHECK(s.point[1] == Approx(17.6));
    CHECK(s.gain == Approx(-kinematics::kWindingRatio));
    // The click turns on its pivot hole, once for each of the barrel's 80 teeth; the tooth depth over the arm
    // gives its swing, clockwise here because the tip is pushed away from the barrel's centre.
    const auto& l = of(click_idx);
    CHECK(l.kind == kinematics::Articulation::Kind::Follow);
    CHECK(l.teeth == 80);
    CHECK(l.driver == m.mobile_of(0));
    CHECK(l.point[0] == Approx(3.3));
    CHECK(l.amplitude == Approx(-(6.15 - 5.8125) / 3.3).epsilon(0.01));
    // The spring bends about its far end, the way the click's tip goes.
    const auto& k = of(spring_idx);
    CHECK(k.kind == kinematics::Articulation::Kind::Follow);
    CHECK(k.driver == l.driver);
    CHECK(k.point[0] == Approx(7.0));
    CHECK(k.amplitude < 0.0);
    CHECK(std::abs(k.amplitude) < 0.05);
}

TEST_CASE("the stem winds, and a click rides up each tooth and drops back") {
    const auto m = rigging::auto_rig("train", train_with_keyless_work(), {});
    const int n = static_cast<int>(m.components.size());
    const auto t = kinematics::analyze_train(m, 25200);
    std::size_t stem = 0, click = 0;
    for (std::size_t i = 0; i < t.articulations.size(); ++i) {
        if (t.articulations[i].component == n - 1) stem = i;
        if (t.articulations[i].component == n - 3) click = i;
    }
    auto at = [&](double seconds) {
        const auto angles = kinematics::mobile_angles(t, seconds);
        return std::pair{angles, kinematics::articulation_angles(t, angles)};
    };
    // Everything is at rest at the start.
    for (double a : at(0.0).second) CHECK(a == 0.0);
    // The barrel turns once in 8 h: the stem turns six times as far, the other way.
    const auto [a1, p1] = at(3600.0);
    CHECK(p1[stem] == Approx(-kinematics::kWindingRatio * a1[static_cast<std::size_t>(t.root)]));
    CHECK(p1[stem] < 0.0);
    // A tooth passes every 360 s: the click is halfway up at 126 s, fully up at 252 s, halfway down at 306 s
    // (it drops in the last 30 % of the pitch) and at rest again at 360 s, and so on tooth after tooth.
    const double swing = t.articulations[click].amplitude;
    CHECK(at(126.0).second[click] == Approx(0.5 * swing).epsilon(1e-3));
    CHECK(at(252.0).second[click] == Approx(swing).epsilon(1e-3));
    CHECK(at(306.0).second[click] == Approx(0.5 * swing).epsilon(1e-3));
    CHECK(at(360.0).second[click] == Approx(0.0).margin(1e-6));
    CHECK(at(360.0 + 252.0).second[click] == Approx(swing).epsilon(1e-3));
}

TEST_CASE("features: the long axis, the sides and the round holes of a part") {
    // A rod lying along x, 1.5 mm across.
    gp_Trsf lay;
    lay.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), std::numbers::pi / 2);
    const TopoDS_Shape rod = BRepBuilderAPI_Transform(geometry::make_arbor(0.75, 0.0, 17.7), lay, true).Shape();
    const auto r = geometry::extract_features({"rod", rod});
    CHECK(r.length == Approx(17.7).epsilon(0.002));
    CHECK(r.width == Approx(1.5).epsilon(0.01));
    CHECK(r.height == Approx(1.5).epsilon(0.01));
    CHECK(std::abs(r.axis[0]) == Approx(1.0).margin(1e-6));
    CHECK(r.axis[0] > 0.0);  // pointed along +x, whichever way round the part is
    CHECK(rigging::classify(r, {}).role == Role::Stem);

    // A tube and a plate with a pivot hole, one off-centre.
    const TopoDS_Shape bush = BRepAlgoAPI_Cut(geometry::make_arbor(0.6, -1.7, 0.5), geometry::make_arbor(0.3, -2.0, 1.0)).Shape();
    const auto b = geometry::extract_features({"bush", bush});
    REQUIRE(b.holes.size() == 1);
    CHECK(b.holes[0].radius == Approx(0.3).epsilon(1e-3));
    CHECK(b.holes[0].x == Approx(0.0).margin(1e-6));
    CHECK(b.length == Approx(2.2).epsilon(0.01));
    CHECK(rigging::classify(b, {}).role == Role::Bearing);

    TopoDS_Shape lever = geometry::make_plate(0, 0, 3.0, 0.0, 0.3);
    gp_Trsf shift;
    shift.SetTranslation(gp_Vec(1.0, 0.5, 0.0));
    lever = BRepAlgoAPI_Cut(lever, BRepBuilderAPI_Transform(geometry::make_arbor(0.25, -1.0, 1.0), shift, true).Shape()).Shape();
    const auto p = geometry::extract_features({"lever", lever});
    REQUIRE(p.holes.size() == 1);
    CHECK(p.holes[0].x == Approx(1.0).epsilon(1e-3));
    CHECK(p.holes[0].y == Approx(0.5).epsilon(1e-3));
    CHECK(p.holes[0].radius == Approx(0.25).epsilon(1e-3));
    // A pin through a plate is not a hole.
    CHECK(geometry::extract_features({"arbor", geometry::make_arbor(0.2, 0.0, 3.0)}).holes.empty());
    // A gear has its bore, and nothing else round.
    CHECK(geometry::extract_features({"gear", geometry::make_gear(40, 0.15, 0, 0.8, 0.1)}).holes.size() == 1);
}
