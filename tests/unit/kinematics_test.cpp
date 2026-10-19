// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "kinematics/kinematics.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using Catch::Approx;

TEST_CASE("the going train reaches 25,200 vph with a 1 rpm fourth wheel") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto t = kinematics::analyze_train(m, 25200);

    CHECK(t.warnings.empty());
    CHECK(t.escape_teeth == 15);
    // Fourth mobile : 80/10 * 75/10 = 60 times the center mobile, which turns once per hour.
    const int center = m.mobile_of(2), fourth = m.mobile_of(9), escape = m.mobile_of(12);
    CHECK(t.mobiles[center].revs_per_hour == Approx(1.0));
    CHECK(t.mobiles[fourth].revs_per_hour / 60.0 == Approx(1.0));
    CHECK(t.mobiles[escape].revs_per_hour / 60.0 == Approx(14.0));
    CHECK(t.root_period_hours == Approx(8.0));  // barrel turns once per 8 h
    // Direction alternates down the train: barrel +, center -, third +, fourth -, escape +.
    CHECK(t.mobiles[t.root].ratio == Approx(1.0));
    CHECK(t.mobiles[center].ratio == Approx(-8.0));
    CHECK(t.mobiles[m.mobile_of(5)].ratio == Approx(64.0));
    CHECK(t.mobiles[fourth].ratio == Approx(-480.0));
    CHECK(t.mobiles[escape].ratio == Approx(6720.0));
}

TEST_CASE("without a beat rate only ratios are produced") {
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto t = kinematics::analyze_train(m, 0.0);
    CHECK(t.root_period_hours == 0.0);
    CHECK(t.beats_per_root_rev == Approx(2.0 * 15 * 6720));
    for (const auto& mm : t.mobiles) CHECK(mm.revs_per_hour == 0.0);
}

TEST_CASE("a missing barrel is an error unless there is an escape wheel to work back from, or a root is given") {
    auto parts = test::synthetic_train();
    parts[0].name = "gear_0";
    parts[0].tooth_confidence = 1;
    rigging::RigOptions o;
    o.presets = {{"^gear_0$", model::Role::Wheel}};
    auto m = rigging::auto_rig("train", parts, o);
    // The escape wheel is there: the train starts at the far end of the chain from it, with a warning.
    const auto t = kinematics::analyze_train(m, 25200);
    CHECK(t.root >= 0);
    CHECK(!t.warnings.empty());

    // No barrel and no escape wheel: nothing says where to start, unless a root is given.
    o.presets.push_back({"^escape_wheel$", model::Role::Wheel});
    m = rigging::auto_rig("train", parts, o);
    CHECK_THROWS_AS(kinematics::analyze_train(m, 25200), Error);
    CHECK_NOTHROW(kinematics::analyze_train(m, 25200, 0));
}

TEST_CASE("ratio there and back is 1 for random trains") {
    std::mt19937 rng(1234);
    std::uniform_int_distribution<int> teeth(7, 120);
    for (int trial = 0; trial < 50; ++trial) {
        model::Movement m;
        const int stages = 2 + trial % 5;
        double product = 1.0;
        int driver = teeth(rng);
        for (int s = 0; s < stages; ++s) {
            model::Component a, b;
            a.features.teeth = driver;
            a.role = s == 0 ? model::Role::Barrel : model::Role::Pinion;
            a.features.name = "a" + std::to_string(s);
            const int driven = teeth(rng);
            b.features.teeth = driven;
            b.role = s == stages - 1 ? model::Role::EscapeWheel : model::Role::Wheel;
            b.features.name = "b" + std::to_string(s);
            product *= -static_cast<double>(driver) / driven;
            const int ia = static_cast<int>(m.components.size());
            m.components.push_back(a);
            m.components.push_back(b);
            m.mobiles.push_back({"M" + std::to_string(s), {ia}, {}});
            if (s == stages - 1) m.mobiles.push_back({"M" + std::to_string(s + 1), {ia + 1}, {}});
            m.meshes.push_back({ia, ia + 1, 1, 1, 1});
            driver = teeth(rng);
        }
        // Chain: mobile s holds the driven gear b_{s-1} and pinion a_s.
        for (int s = 1; s < stages; ++s) {
            m.mobiles[s].components.push_back(2 * (s - 1) + 1);
        }
        const auto fwd = kinematics::analyze_train(m, 0, 0);
        const int last = static_cast<int>(m.mobiles.size()) - 1;
        const auto back = kinematics::analyze_train(m, 0, last);
        REQUIRE(fwd.mobiles[last].ratio * back.mobiles[0].ratio == Approx(1.0));
        REQUIRE(fwd.mobiles[last].ratio == Approx(product));
    }
}

TEST_CASE("gear loops with inconsistent ratios are flagged") {
    auto parts = test::synthetic_train();
    auto m = rigging::auto_rig("train", parts, {});
    // Add a bogus second path that bypasses the train with the wrong ratio.
    m.meshes.push_back({0, 12, 1, 1, 1});  // barrel -> escape wheel directly
    const auto t = kinematics::analyze_train(m, 25200);
    CHECK_FALSE(t.warnings.empty());
}

namespace {
// The synthetic train plus a 31-tooth date ring concentric with the barrel.
model::Movement train_with_date_ring() {
    auto parts = test::synthetic_train();
    auto ring = test::arbor("date wheel", 0, 0, 0.2, 0.6);
    ring.outer_radius = 14.0;
    ring.inner_teeth = 31;
    parts.push_back(ring);
    return rigging::auto_rig("train", parts, {});
}
constexpr double kDay = 86400.0;
}  // namespace

TEST_CASE("a date ring steps one tooth a day, driven by the wheel that turns once in 24 h") {
    const auto m = train_with_date_ring();
    const int ring = m.mobile_of(static_cast<int>(m.components.size()) - 1);
    REQUIRE(ring >= 0);
    // At 8,400 vph the barrel turns once in 24 h, so it is the 24-hour wheel.
    const auto t = kinematics::analyze_train(m, 8400);
    REQUIRE(t.root_period_hours == Approx(24.0));
    CHECK(t.mobiles[ring].reached);
    CHECK(t.mobiles[ring].index_teeth == 31);
    CHECK(t.mobiles[ring].index_driver == t.root);
    CHECK(t.mobiles[ring].ratio == Approx(1.0 / 31.0));
    CHECK(t.mobiles[ring].revs_per_hour == Approx(1.0 / (24.0 * 31.0)));
    for (const auto& w : t.warnings) CHECK(w.find("date ring") == std::string::npos);

    const double tooth = 2.0 * std::numbers::pi / 31.0;
    for (bool stepped : {false, true}) {
        const auto angle = [&](double seconds) { return kinematics::mobile_angles(t, seconds, {stepped})[ring]; };
        CHECK(angle(0) == Approx(0.0).margin(1e-9));
        CHECK(angle(0.5 * kDay) == Approx(0.0).margin(1e-9));      // at rest most of the day
        CHECK(angle(0.8 * kDay) == Approx(0.0).margin(1e-9));
        CHECK(angle((1.0 - 0.075) * kDay) == Approx(0.5 * tooth).margin(1e-3 * tooth));  // half way through the step
        CHECK(angle(kDay + 1.0) == Approx(tooth).margin(1e-3 * tooth));
        CHECK(angle(1.5 * kDay) == Approx(tooth).margin(1e-9));
        CHECK(angle(10.0 * kDay + 1.0) == Approx(10.0 * tooth).margin(1e-3 * tooth));
        // A step never goes backwards.
        double previous = -1.0;
        for (double s = 0; s < 3 * kDay; s += 600) {
            const double a = angle(s);
            CHECK(a >= previous - 1e-12);
            previous = a;
        }
    }
    // The train itself is not disturbed.
    const auto a = kinematics::mobile_angles(t, 3600.0);
    CHECK(a[t.root] == Approx(2.0 * std::numbers::pi / 24.0));
}

TEST_CASE("without a 24-hour wheel a date ring steps on clock time, with a warning") {
    const auto m = train_with_date_ring();
    const int ring = m.mobile_of(static_cast<int>(m.components.size()) - 1);
    const auto t = kinematics::analyze_train(m, 25200);  // no mobile turns once a day at this rate
    CHECK(t.mobiles[ring].index_driver == -1);
    CHECK(t.mobiles[ring].index_teeth == 31);
    CHECK(std::any_of(t.warnings.begin(), t.warnings.end(), [](const std::string& w) {
        return w.find("date ring") != std::string::npos;
    }));
    const double tooth = 2.0 * std::numbers::pi / 31.0;
    CHECK(kinematics::mobile_angles(t, 0.5 * kDay)[ring] == Approx(0.0).margin(1e-9));
    CHECK(kinematics::mobile_angles(t, 2.0 * kDay + 1.0)[ring] == Approx(2.0 * tooth).margin(1e-3 * tooth));
}

TEST_CASE("a date ring is not reported as unconnected, and without a beat rate it is left out") {
    const auto m = train_with_date_ring();
    const int ring = m.mobile_of(static_cast<int>(m.components.size()) - 1);
    const auto t = kinematics::analyze_train(m, 0.0);
    CHECK_FALSE(t.mobiles[ring].reached);
    for (const auto& w : t.warnings) CHECK(w.find("not connected") == std::string::npos);
}

TEST_CASE("a date ring with no teeth found assumes 31 and says so") {
    auto parts = test::synthetic_train();
    parts.push_back(test::arbor("date wheel", 0, 0, 0.2, 0.6));
    const auto m = rigging::auto_rig("train", parts, {});
    const auto t = kinematics::analyze_train(m, 8400);
    CHECK(t.mobiles[m.mobile_of(static_cast<int>(parts.size()) - 1)].index_teeth == 31);
    CHECK(std::any_of(t.warnings.begin(), t.warnings.end(), [](const std::string& w) {
        return w.find("assuming 31") != std::string::npos;
    }));
}
