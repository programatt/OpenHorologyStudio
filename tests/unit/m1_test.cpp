// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "explain/explain.hpp"
#include "geometry/profile.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using Catch::Approx;

namespace {

// Radius-vs-angle outline and raw polar points of a gear with involute flanks.
struct SyntheticGear {
    std::vector<double> bins;
    std::vector<geometry::PolarPoint> points;
    double root, tip, base;
};

SyntheticGear involute_outline(int teeth, double module, double phase, double pressure_deg = 20.0) {
    const double two_pi = 2 * std::numbers::pi;
    const double alpha_p = pressure_deg * std::numbers::pi / 180;
    const double rp = teeth * module / 2, rb = rp * std::cos(alpha_p);
    SyntheticGear g{{}, {}, rp - 1.25 * module, rp + module, rb};
    auto inv = [](double a) { return std::tan(a) - a; };
    auto half = [&](double r) {
        return std::numbers::pi / (2 * teeth) + inv(alpha_p) - (r > rb ? inv(std::acos(rb / r)) : 0.0);
    };
    const double pitch = two_pi / teeth;
    // Sample r(theta) by inverting the flank: for each angle find the radius of the tooth edge.
    auto radius_at = [&](double theta) {
        double u = std::fmod(theta - phase, pitch);
        u -= pitch * std::round(u / pitch);
        u = std::abs(u);
        if (u >= half(g.root)) return g.root;
        if (u <= half(g.tip)) return g.tip;
        double lo = g.root, hi = g.tip;
        for (int i = 0; i < 60; ++i) {
            const double mid = 0.5 * (lo + hi);
            (half(mid) > u ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    };
    const int n = 1440;
    for (int i = 0; i < n; ++i) g.bins.push_back(radius_at(two_pi * (i + 0.5) / n));
    for (int i = 0; i < 40000; ++i) {
        const double theta = two_pi * (i + 0.5) / 40000;
        g.points.push_back({radius_at(theta), theta});
    }
    return g;
}

}  // namespace

TEST_CASE("tooth phase locates the tooth tips") {
    const double pitch = 2 * std::numbers::pi / 40;
    const auto g = involute_outline(40, 0.15, 0.3 * pitch);
    CHECK(geometry::tooth_phase(g.bins, 40) == Approx(0.3 * pitch).margin(0.02 * pitch));
}

TEST_CASE("involute flank fit recovers the base circle and rejects straight flanks") {
    const int teeth = GENERATE(40, 60, 84);
    const auto g = involute_outline(teeth, 0.15, 0.123);
    const double phase = geometry::tooth_phase(g.bins, teeth);
    const auto fit = geometry::fit_involute_flank(g.points, teeth, phase, g.root, g.tip);
    REQUIRE(fit.ok);
    CHECK(fit.base_radius == Approx(g.base).epsilon(0.01));
    CHECK(fit.rms < 0.003);

    // The same tooth with a straight (trapezoid) flank must fit clearly worse than 1 % of a module.
    std::vector<geometry::PolarPoint> straight;
    const double pitch = 2 * std::numbers::pi / teeth;
    for (int t = 0; t < teeth; ++t) {
        for (int i = 0; i <= 400; ++i) {
            const double f = i / 400.0;
            const double r = g.root + (g.tip - g.root) * f;
            const double u = pitch * (0.25 - 0.13 * f);  // linear in r
            for (double sign : {-1.0, 1.0}) {
                straight.push_back({r, std::fmod(t * pitch + phase + sign * u + 2 * std::numbers::pi, 2 * std::numbers::pi)});
            }
        }
    }
    const auto bad = geometry::fit_involute_flank(straight, teeth, phase, g.root, g.tip);
    REQUIRE(bad.ok);
    CHECK(bad.rms > fit.rms);
}

TEST_CASE("gear fit takes the module from meshes and reports addendum, dedendum and pressure angle") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    for (const auto& c : m.components) {
        if (!model::is_toothed_role(c.role)) {
            CHECK_FALSE(c.gear.has_value());
            continue;
        }
        REQUIRE(c.gear.has_value());
        CHECK(c.gear->module == Approx(0.15));
        // The escape wheel's partner, the pallet fork, is not part of the rig yet.
        CHECK(c.gear->module_source == (c.role == model::Role::EscapeWheel ? "tip" : "mesh"));
        CHECK(c.gear->addendum == Approx(1.0));
        CHECK(c.gear->dedendum == Approx(1.25));
        CHECK(c.gear->profile == "involute");
        CHECK(c.gear->pressure_angle_deg == Approx(20.0).epsilon(0.005));
    }
}

TEST_CASE("an unmeshed gear falls back to the tip-radius module") {
    auto parts = test::synthetic_train();
    for (auto& p : parts) {
        if (p.name == "escape_wheel") p.center.x += 5;  // pull it out of the escape pinion's mobile
    }
    const auto m = rigging::auto_rig("train", parts, {});
    const int idx = m.find_component("escape_wheel");
    REQUIRE(idx >= 0);
    REQUIRE(m.components[idx].gear.has_value());
    CHECK(m.components[idx].gear->module_source == "tip");
    CHECK(m.components[idx].gear->module == Approx(0.15));
}

TEST_CASE("gear fits survive the rig file round trip") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto copy = model::from_toml(model::to_toml(m));
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        REQUIRE(m.components[i].gear.has_value() == copy.components[i].gear.has_value());
        if (!m.components[i].gear) continue;
        CHECK(copy.components[i].gear->module == m.components[i].gear->module);
        CHECK(copy.components[i].gear->profile == m.components[i].gear->profile);
        CHECK(copy.components[i].gear->pressure_angle_deg == m.components[i].gear->pressure_angle_deg);
    }
    CHECK(model::to_toml(copy) == model::to_toml(m));
}

TEST_CASE("set_role re-infers mobiles and meshes") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    REQUIRE(m.meshes.size() == 4);

    // Demoting the fourth wheel to a plain part removes its mesh and its place in the mobile.
    rigging::set_role(m, "fourth_wheel", model::Role::Screw, {});
    CHECK(m.meshes.size() == 3);
    const int idx = m.find_component("fourth_wheel");
    CHECK(m.components[idx].role_source == "user");
    CHECK(m.components[idx].role_confidence == 1.0);
    CHECK(m.mobile_of(idx) == -1);

    rigging::set_role(m, "fourth_wheel", model::Role::Wheel, {});
    CHECK(m.meshes.size() == 4);

    CHECK_THROWS_AS(rigging::set_role(m, "no_such_part", model::Role::Wheel, {}), Error);
}

TEST_CASE("poses: smooth and stepped trains agree at whole beats") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto t = kinematics::analyze_train(m, 25200);
    const int escape = m.mobile_of(m.find_component("escape_wheel"));
    const int fourth = m.mobile_of(m.find_component("fourth_wheel"));

    // One real minute: the fourth wheel makes one turn and the escape wheel 14.
    const auto a = kinematics::mobile_angles(t, 60.0);
    CHECK(std::abs(a[fourth]) == Approx(2 * std::numbers::pi));
    CHECK(std::abs(a[escape]) == Approx(14 * 2 * std::numbers::pi));
    CHECK(a[fourth] * a[escape] < 0);  // adjacent mobiles turn opposite ways

    // Stepped: at the end of each beat the train is exactly where the smooth model puts it.
    kinematics::PoseOptions stepped{true, 0.2};
    const double beat = 3600.0 / 25200.0;
    for (int n : {1, 7, 100}) {
        const auto s = kinematics::mobile_angles(t, n * beat, stepped);
        const auto c = kinematics::mobile_angles(t, n * beat);
        for (std::size_t i = 0; i < s.size(); ++i) CHECK(s[i] == Approx(c[i]).margin(1e-9));
    }
    // Mid-beat the stepped train is at rest at the previous beat.
    const auto rest = kinematics::mobile_angles(t, 3.4 * beat, stepped);
    const auto prev = kinematics::mobile_angles(t, 3.0 * beat);
    for (std::size_t i = 0; i < rest.size(); ++i) CHECK(rest[i] == Approx(prev[i]).margin(1e-9));
    // One escapement step turns the escape wheel by half a tooth pitch.
    const auto one = kinematics::mobile_angles(t, beat, stepped);
    CHECK(std::abs(one[escape]) == Approx(2 * std::numbers::pi / 30));
}

TEST_CASE("poses need a beat rate") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto t = kinematics::analyze_train(m, 0.0);
    CHECK_THROWS_AS(kinematics::mobile_angles(t, 1.0), Error);
}

TEST_CASE("explain: part facts and train walkthrough carry this movement's numbers") {
    using Catch::Matchers::ContainsSubstring;
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto t = kinematics::analyze_train(m, 25200);

    const auto e = explain::explain_part(m, t, "fourth_wheel");
    const std::string text = explain::to_text(e);
    CHECK_THAT(text, ContainsSubstring("84 teeth"));
    CHECK_THAT(text, ContainsSubstring("1 rpm"));
    CHECK_THAT(text, ContainsSubstring("meshes with escape_pinion"));
    CHECK_THAT(text, ContainsSubstring("involute"));
    CHECK_THROWS_AS(explain::explain_part(m, t, "nope"), Error);

    const auto w = explain::walk_train(m, t);
    REQUIRE(w.steps.size() == 4);
    CHECK(w.steps[0].driver == "barrel");
    CHECK(w.steps[0].step_ratio == Approx(8.0));
    CHECK(w.steps[3].driven == "escape_pinion");
    CHECK(w.steps[3].step_ratio == Approx(14.0));
    CHECK(w.steps[3].revs_per_hour == Approx(840.0));
    const std::string walk = explain::to_text(w);
    CHECK_THAT(walk, ContainsSubstring("15 teeth x 2 beats per tooth = 30 beats per turn"));
    CHECK_THAT(walk, ContainsSubstring("25200 beats/h"));
}
