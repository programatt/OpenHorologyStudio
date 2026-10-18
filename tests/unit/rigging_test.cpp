// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <set>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using model::Role;

namespace {
model::PartFeatures named(const std::string& name, int teeth = 0) {
    auto f = test::gear(name, teeth, 0.15, 0, 0, 0, 1);
    if (teeth == 0) {  // a shapeless blob: neither toothed, plate-like nor arbor-like
        f.tooth_confidence = 0;
        f.outer_radius = 2.0;
        f.extent_x = f.extent_y = 4.0;
    }
    return f;
}
}  // namespace

TEST_CASE("name hints take precedence in a sensible order") {
    rigging::RigOptions o;
    CHECK(rigging::classify(named("escape_pinion", 6), o).role == Role::Pinion);
    CHECK(rigging::classify(named("Escape Wheel", 15), o).role == Role::EscapeWheel);
    CHECK(rigging::classify(named("Balance Wheel"), o).role == Role::Balance);
    CHECK(rigging::classify(named("Barrel", 80), o).role == Role::Barrel);
    CHECK(rigging::classify(named("pallet fork"), o).role == Role::PalletFork);
    CHECK(rigging::classify(named("Gear Train Bridge"), o).role == Role::Bridge);
    CHECK(rigging::classify(named("third_wheel_arbor"), o).role == Role::Arbor);
}

TEST_CASE("geometry classifies unnamed parts and flags contradictions") {
    rigging::RigOptions o;
    const auto wheel = rigging::classify(named("Body1", 60), o);
    CHECK(wheel.role == Role::Wheel);
    CHECK(wheel.source == "geometry");
    CHECK(rigging::classify(named("Body2", 10), o).role == Role::Pinion);
    CHECK(rigging::classify(named("Body3"), o).role == Role::Unknown);

    // A "wheel" with no teeth is still a wheel by name, but with low confidence.
    CHECK(rigging::classify(named("odd wheel"), o).confidence < 0.6);
}

TEST_CASE("presets override name and geometry") {
    rigging::RigOptions o;
    o.presets = {{"^gp[0-9]+$", Role::Wheel}};
    const auto c = rigging::classify(named("GP12"), o);
    CHECK(c.role == Role::Wheel);
    CHECK(c.source == "preset");
    CHECK(c.confidence == 1.0);

    o.presets = {{"([", Role::Wheel}};
    CHECK_THROWS_AS(rigging::classify(named("x"), o), Error);
}

TEST_CASE("the synthetic train rigs into 5 mobiles and exactly the 4 real meshes") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    REQUIRE(m.mobiles.size() == 5);

    std::set<std::pair<std::string, std::string>> meshes;
    for (const auto& j : m.meshes) {
        meshes.insert({m.components[j.a].features.name, m.components[j.b].features.name});
        CHECK(j.confidence > 0.9);
        CHECK(j.module == Catch::Approx(0.15));
    }
    // barrel/third_pinion and similar coincidences sit at the right distance but not the right height.
    const std::set<std::pair<std::string, std::string>> expected{{"barrel", "center_pinion"},
                                                                 {"center_wheel", "third_pinion"},
                                                                 {"third_wheel", "fourth_pinion"},
                                                                 {"fourth_wheel", "escape_pinion"}};
    CHECK(meshes == expected);

    // Wheel and pinion that share an axis are one mobile.
    CHECK(m.mobile_of(3) == m.mobile_of(2));  // center_wheel with center_pinion
    CHECK(m.mobile_of(0) != m.mobile_of(2));
    // The plate does not rotate.
    CHECK(m.mobile_of(static_cast<int>(m.components.size()) - 1) == -1);
}

TEST_CASE("centre distance and module errors reject near-misses") {
    auto parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "center_pinion" || p.name == "center_wheel" || p.name == "center_arbor") p.center.x += 0.5;
    }
    const auto moved = rigging::auto_rig("moved", parts, {});
    CHECK(moved.meshes.size() == 2);  // barrel/center_pinion and center_wheel/third_pinion both break

    parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "escape_pinion") p = test::gear(p.name, 6, 0.3, p.center.x, p.center.y, p.z_min, p.z_max);
    }
    const auto mismatched = rigging::auto_rig("mismatch", parts, {});
    CHECK(mismatched.meshes.size() == 3);  // module 0.3 vs 0.15
}

TEST_CASE("check_rig reports what a user should confirm") {
    const auto good = rigging::auto_rig("train", test::synthetic_train(), {});
    for (const auto& i : rigging::check_rig(good)) CHECK(i.severity != rigging::Severity::Error);

    auto parts = test::synthetic_train();
    parts.push_back(named("Body7"));
    const auto with_unknown = rigging::auto_rig("train", parts, {});
    const auto issues = rigging::check_rig(with_unknown);
    CHECK(std::any_of(issues.begin(), issues.end(), [](const rigging::Issue& i) {
        return i.severity == rigging::Severity::Warning && i.message.find("Body7") != std::string::npos;
    }));

    const auto empty = rigging::check_rig(rigging::auto_rig("none", {}, {}));
    CHECK(std::any_of(empty.begin(), empty.end(),
                      [](const rigging::Issue& i) { return i.severity == rigging::Severity::Error; }));
}
