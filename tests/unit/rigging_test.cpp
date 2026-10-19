// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <set>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "model/rig_io.hpp"
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

TEST_CASE("an automatic's rotor is a rotor, by name or by its place in the movement") {
    rigging::RigOptions o;
    CHECK(rigging::classify(named("Oscillating Weight"), o).role == Role::Rotor);
    CHECK(rigging::classify(named("rotor"), o).role == Role::Rotor);
    CHECK(rigging::classify(named("Rotor Plate"), o).role == Role::Rotor);
    // Its fixings and bearings keep their own roles.
    CHECK(rigging::classify(named("rotor screw"), o).role == Role::Screw);
    CHECK(rigging::classify(named("rotor bridge"), o).role == Role::Bridge);
    CHECK(rigging::classify(named("rotor_pinion", 8), o).role == Role::Pinion);

    // Unnamed, large and thin: alone, a rotor passes for a plate. Next to the main plate it covers half the
    // movement with its mass to one side, which makes it the rotor, whichever way it is turned in the file.
    const auto slab = [](std::string name, double ex, double ey, double cx, double cy, double area, double thick) {
        model::PartFeatures f;
        f.name = std::move(name);
        f.extent_x = ex;
        f.extent_y = ey;
        f.center = {cx, cy};
        f.z_min = 0;
        f.z_max = thick;
        f.volume = area * thick;
        return f;
    };
    const double r = 15.0, pi = 3.14159265358979;
    const auto plate = slab("Body1", 2 * r, 2 * r, 0, 0, 0.8 * pi * r * r, 0.5);
    const auto bridge = slab("Body2", 12, 5, 6, -4, 50, 0.4);
    for (const auto& rotor : {slab("Body3", 2 * r, 1.1 * r, 0, 6.4, 0.5 * pi * r * r, 0.6),           // square to the axes
                              slab("Body3", 1.71 * r, 1.71 * r, 4.5, 4.5, 0.5 * pi * r * r, 0.6)}) {  // turned 45 degrees
        CHECK(rigging::classify(rotor, o).role == Role::Plate);
        const auto m = rigging::auto_rig("automatic", {plate, bridge, rotor}, o);
        CHECK(m.components[0].role == Role::Plate);
        CHECK(m.components[1].role == Role::Plate);  // the small bridge is left as it was
        CHECK(m.components[2].role == Role::Rotor);
        CHECK(m.components[2].role_confidence == Catch::Approx(0.5));
        CHECK(m.components[2].role_source == "geometry");
        CHECK(m.mobile_of(2) < 0);  // not animated yet, so in no mobile
    }

    // A movement without a rotor keeps its plates.
    const auto m = rigging::auto_rig("manual", {plate, bridge, slab("Body4", 2 * r, 2 * r, 0.5, 0, 0.7 * pi * r * r, 0.4)}, o);
    for (const auto& c : m.components) CHECK(c.role == Role::Plate);
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

TEST_CASE("a date wheel is a ring of its own, by name or by its internal teeth") {
    rigging::RigOptions o;
    CHECK(rigging::classify(named("Date Wheel"), o).role == Role::DateRing);
    CHECK(rigging::classify(named("date_ring"), o).role == Role::DateRing);
    CHECK(rigging::classify(named("Quantieme disc"), o).role == Role::DateRing);
    CHECK(rigging::classify(named("Date indicator"), o).role == Role::DateRing);
    // Its fixings and what drives it keep their own roles.
    CHECK(rigging::classify(named("date wheel screw"), o).role == Role::Screw);
    CHECK(rigging::classify(named("date wheel bridge"), o).role == Role::Bridge);
    CHECK(rigging::classify(named("date driving wheel", 24), o).role == Role::Wheel);
    CHECK(rigging::classify(named("date wheel jumper"), o).role != Role::DateRing);
    CHECK(rigging::classify(named("date finger"), o).role != Role::DateRing);

    // Named, but no teeth found anywhere: a guess. With internal teeth it is surer.
    CHECK(rigging::classify(named("date wheel"), o).confidence < 0.8);
    auto ring = named("date wheel");
    ring.inner_teeth = 31;
    CHECK(rigging::classify(ring, o).confidence == Catch::Approx(0.9));

    // Unnamed: internal teeth and none outside make a date ring, not a plate.
    auto bare = named("Body9");
    bare.extent_x = bare.extent_y = 30;
    bare.inner_teeth = 31;
    const auto c = rigging::classify(bare, o);
    CHECK(c.role == Role::DateRing);
    CHECK(c.source == "geometry");
}

TEST_CASE("a date ring is a mobile of its own even when it shares an axis with the train") {
    auto parts = test::synthetic_train();
    const std::size_t barrel_mobiles = rigging::auto_rig("train", parts, {}).mobiles.size();
    auto ring = test::arbor("date wheel", 0, 0, 0.2, 0.6);  // concentric with the barrel and its arbor
    ring.outer_radius = 14.0;
    ring.inner_teeth = 31;
    parts.push_back(ring);
    const auto m = rigging::auto_rig("train", parts, {});
    const int idx = static_cast<int>(parts.size()) - 1;
    REQUIRE(m.components[idx].role == Role::DateRing);
    CHECK(m.mobiles.size() == barrel_mobiles + 1);
    const int mobile = m.mobile_of(idx);
    REQUIRE(mobile >= 0);
    CHECK(m.mobiles[mobile].components == std::vector<int>{idx});
    CHECK(m.mobile_of(0) != mobile);   // the barrel
    CHECK(m.meshes.size() == rigging::auto_rig("train", test::synthetic_train(), {}).meshes.size());  // not geared to anything
}

TEST_CASE("a date ring with no teeth found is flagged") {
    auto parts = test::synthetic_train();
    auto ring = test::arbor("date wheel", 0, 0, 0.2, 0.6);
    parts.push_back(ring);
    const auto m = rigging::auto_rig("train", parts, {});
    const auto issues = rigging::check_rig(m);
    CHECK(std::any_of(issues.begin(), issues.end(), [](const rigging::Issue& i) {
        return i.message.find("date ring 'date wheel'") != std::string::npos;
    }));
}

TEST_CASE("the train chain runs from the barrel to the escape wheel, and shows where it breaks") {
    const auto good = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto chain = rigging::train_chain(good);
    REQUIRE(chain.connected);
    CHECK(chain.path.size() == 5);
    CHECK(chain.path.front() == chain.barrel);
    CHECK(chain.path.back() == chain.escape);
    CHECK(std::all_of(chain.fed.begin(), chain.fed.end(), [](char f) { return f == 1; }));
    CHECK(rigging::describe_chain(good, chain).find(" -> ") != std::string::npos);

    // Move the centre mobile off its mesh with the barrel and with the third pinion.
    auto parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "center_pinion" || p.name == "center_wheel" || p.name == "center_arbor") p.center.x += 0.5;
    }
    const auto broken = rigging::auto_rig("moved", parts, {});
    const auto split = rigging::train_chain(broken);
    CHECK_FALSE(split.connected);
    CHECK(split.path.empty());
    REQUIRE_FALSE(split.from_barrel.empty());
    CHECK(split.from_barrel.front() == split.barrel);
    REQUIRE_FALSE(split.from_escape.empty());
    CHECK(split.from_escape.back() == split.escape);
    CHECK(split.fed[static_cast<std::size_t>(split.barrel)] == 1);
    CHECK(split.fed[static_cast<std::size_t>(split.escape)] == 0);
    CHECK(rigging::describe_chain(broken, split).find("[no mesh]") != std::string::npos);

    // check_rig says so, and names the parts at the two ends of the break so that a viewer can select them.
    const auto issues = rigging::check_rig(broken);
    const auto it = std::find_if(issues.begin(), issues.end(), [](const rigging::Issue& i) {
        return i.message.find("the train is broken") != std::string::npos;
    });
    REQUIRE(it != issues.end());
    CHECK(it->severity == rigging::Severity::Warning);
    CHECK(it->parts.size() == 2);
}

TEST_CASE("meshes that just missed the strict test are suggested, and declaring them joins the train") {
    auto parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "center_pinion" || p.name == "center_wheel" || p.name == "center_arbor") p.center.x += 0.05;
    }
    auto m = rigging::auto_rig("nudged", parts, {});
    REQUIRE_FALSE(rigging::train_chain(m).connected);

    const auto suggestions = rigging::suggest_meshes(m, {});
    REQUIRE_FALSE(suggestions.empty());
    for (const auto& s : suggestions) {
        CHECK(s.confidence > 0.0);
        CHECK(s.confidence <= 0.5);
    }
    CHECK(std::is_sorted(suggestions.begin(), suggestions.end(),
                         [](const auto& a, const auto& b) { return a.confidence > b.confidence; }));
    const auto offers = [&](const char* a, const char* b) {
        return std::any_of(suggestions.begin(), suggestions.end(), [&](const rigging::MeshSuggestion& s) {
            const auto& na = m.components[static_cast<std::size_t>(s.a)].features.name;
            const auto& nb = m.components[static_cast<std::size_t>(s.b)].features.name;
            return (na == a && nb == b) || (na == b && nb == a);
        });
    };
    CHECK(offers("barrel", "center_pinion"));
    CHECK_FALSE(offers("barrel", "center_wheel"));  // not at that height

    for (const auto& s : suggestions) {
        rigging::add_manual_mesh(m, m.components[static_cast<std::size_t>(s.a)].features.name,
                                 m.components[static_cast<std::size_t>(s.b)].features.name, {});
    }
    CHECK(rigging::train_chain(m).connected);
    CHECK(rigging::suggest_meshes(m, {}).empty());  // what was declared is not offered again

    // A properly rigged train has nothing to suggest.
    CHECK(rigging::suggest_meshes(rigging::auto_rig("train", test::synthetic_train(), {}), {}).empty());
}

TEST_CASE("a mesh left out of the train as winding work is not walked, and stays out when re-rigged") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    REQUIRE(rigging::train_chain(m).connected);

    rigging::set_mesh_winding(m, "barrel", "center_pinion", true);
    CHECK_FALSE(rigging::train_chain(m).connected);
    const auto walk = rigging::walk_train(m, rigging::train_chain(m), {});
    REQUIRE_FALSE(walk.steps.empty());
    CHECK(walk.steps.front().kind == rigging::WalkStep::Kind::Gap);
    CHECK(m.components[static_cast<std::size_t>(walk.steps.front().driver)].features.name == "barrel");

    rigging::re_rig(m, {}, true);
    CHECK_FALSE(rigging::train_chain(m).connected);
    const auto back = model::from_toml(model::to_toml(m));
    CHECK_FALSE(rigging::train_chain(back).connected);

    rigging::set_mesh_winding(m, "barrel", "center_pinion", false);
    CHECK(rigging::train_chain(m).connected);
    CHECK_THROWS_AS(rigging::set_mesh_winding(m, "barrel", "escape_wheel", true), Error);

    // A ratchet or click wheel is the winding work whatever the user calls it: its meshes are not the train's.
    for (auto& c : m.components) {
        if (c.features.name == "center_wheel") c.label = "Ratchet wheel";
    }
    CHECK_FALSE(rigging::train_chain(m).connected);
}

TEST_CASE("walking the train goes mesh by mesh from the barrel, and stops at the gap with what could fill it") {
    using Kind = rigging::WalkStep::Kind;
    const auto good = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto done = rigging::walk_train(good, rigging::train_chain(good), {});
    REQUIRE(done.complete);
    REQUIRE(done.steps.size() == 4);
    for (const auto& s : done.steps) CHECK(s.kind == Kind::Mesh);
    CHECK(good.components[static_cast<std::size_t>(done.steps[0].driver)].features.name == "barrel");
    CHECK(good.components[static_cast<std::size_t>(done.steps[0].driven)].features.name == "center_pinion");
    // Each step starts where the one before ended: the driven wheel's mobile holds the next driver.
    for (std::size_t i = 1; i < done.steps.size(); ++i) {
        CHECK(good.mobile_of(done.steps[i].driver) == good.mobile_of(done.steps[i - 1].driven));
        CHECK(done.steps[i].driver != done.steps[i - 1].driven);  // the other gear of that mobile
    }
    CHECK(done.steps[0].text.find("80 teeth") != std::string::npos);
    CHECK(done.steps[0].text.find("8 times") != std::string::npos);

    // A train nudged off its first mesh: the walk ends at a gap at the barrel, and the pair that would join it
    // is the one offered.
    auto parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "center_pinion" || p.name == "center_wheel" || p.name == "center_arbor") p.center.x += 0.05;
    }
    const auto nudged = rigging::auto_rig("nudged", parts, {});
    const auto suggestions = rigging::suggest_meshes(nudged, {});
    const auto walk = rigging::walk_train(nudged, rigging::train_chain(nudged), suggestions);
    CHECK_FALSE(walk.complete);
    REQUIRE_FALSE(walk.steps.empty());
    const auto& gap = walk.steps.back();
    REQUIRE(gap.kind == Kind::Gap);
    CHECK(walk.steps.size() == 1);  // the barrel meshes with nothing yet
    CHECK(nudged.components[static_cast<std::size_t>(gap.driver)].features.name == "barrel");
    REQUIRE_FALSE(gap.suggestions.empty());
    const auto& offered = suggestions[static_cast<std::size_t>(gap.suggestions[0])];
    const auto& a = nudged.components[static_cast<std::size_t>(offered.a)].features.name;
    const auto& b = nudged.components[static_cast<std::size_t>(offered.b)].features.name;
    CHECK(((a == "barrel" && b == "center_pinion") || (b == "barrel" && a == "center_pinion")));

    // Without a barrel, or without an escape wheel, the first thing missing is a part to pick.
    auto no_barrel = good;
    rigging::set_role(no_barrel, "barrel", Role::Plate, {});
    const auto first = rigging::walk_train(no_barrel, rigging::train_chain(no_barrel), {});
    REQUIRE(first.steps.size() == 1);
    CHECK(first.steps[0].kind == Kind::Missing);
    CHECK(first.steps[0].needs_role == Role::Barrel);
    auto no_escape = good;
    rigging::set_role(no_escape, "escape_wheel", Role::Plate, {});
    const auto last = rigging::walk_train(no_escape, rigging::train_chain(no_escape), {});
    REQUIRE_FALSE(last.steps.empty());
    CHECK(last.steps.back().kind == Kind::Missing);
    CHECK(last.steps.back().needs_role == Role::EscapeWheel);
}

TEST_CASE("re-running the auto-rig recognises parts again but keeps what the user decided") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const std::size_t meshes = m.meshes.size();
    const int third = m.find_component("third_wheel");
    REQUIRE(third >= 0);

    // A part whose role is stale (decided by name or geometry) is decided afresh.
    m.components[static_cast<std::size_t>(third)].role = Role::Unknown;
    m.components[static_cast<std::size_t>(third)].role_confidence = 0.0;
    rigging::infer_structure(m, {});
    REQUIRE(m.meshes.size() < meshes);
    const auto report = rigging::re_rig(m, {});
    REQUIRE(report.role_changes.size() == 1);
    CHECK(report.role_changes[0].part == "third_wheel");
    CHECK(report.role_changes[0].from == Role::Unknown);
    CHECK(report.role_changes[0].to == Role::Wheel);
    CHECK(report.meshes_before < report.meshes_after);
    CHECK(m.meshes.size() == meshes);
    CHECK(rigging::re_rig(m, {}).role_changes.empty());  // nothing more to find

    // A role the user set stays, and so does a mesh the user declared; resetting forgets both.
    rigging::set_role(m, "third_wheel", Role::Plate, {});
    rigging::add_manual_mesh(m, "barrel", "fourth_pinion", {});
    CHECK(rigging::re_rig(m, {}).role_changes.empty());
    CHECK(m.components[static_cast<std::size_t>(third)].role == Role::Plate);
    CHECK(std::any_of(m.meshes.begin(), m.meshes.end(), [](const model::MeshJoint& j) { return j.manual; }));
    const auto reset = rigging::re_rig(m, {}, false);
    REQUIRE(reset.role_changes.size() == 1);
    CHECK(m.components[static_cast<std::size_t>(third)].role == Role::Wheel);
    CHECK(m.components[static_cast<std::size_t>(third)].role_source != "user");
    CHECK(std::none_of(m.meshes.begin(), m.meshes.end(), [](const model::MeshJoint& j) { return j.manual; }));
}
