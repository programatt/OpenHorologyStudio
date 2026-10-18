// SPDX-License-Identifier: Apache-2.0
#include <cmath>
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

TEST_CASE("a missing barrel is an error unless a root is given") {
    auto parts = test::synthetic_train();
    parts[0].name = "gear_0";
    parts[0].tooth_confidence = 1;
    rigging::RigOptions o;
    o.presets = {{"^gear_0$", model::Role::Wheel}};
    const auto m = rigging::auto_rig("train", parts, o);
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
