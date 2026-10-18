// SPDX-License-Identifier: Apache-2.0
#include <filesystem>
#include <set>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "app/import.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"

using namespace ohs;
using Catch::Approx;

TEST_CASE("generated movement: STEP -> import -> rig -> kinematics gives 25,200 vph") {
    const auto step = std::filesystem::temp_directory_path() / "ohs_test_pipeline.step";
    geometry::export_step(geometry::make_test_movement(), step);

    app::ImportOptions opt;
    opt.rig.beat_rate_vph = 25200;
    const model::Movement m = app::import_movement(step, opt);
    std::filesystem::remove(step);

    REQUIRE(m.components.size() == 15);
    REQUIRE(m.mobiles.size() == 5);
    REQUIRE(m.meshes.size() == 4);

    auto role_of = [&m](const std::string& name) {
        for (const auto& c : m.components) {
            if (c.features.name == name) return c.role;
        }
        FAIL("missing component " << name);
        return model::Role::Unknown;
    };
    CHECK(role_of("barrel") == model::Role::Barrel);
    CHECK(role_of("escape_wheel") == model::Role::EscapeWheel);
    CHECK(role_of("escape_pinion") == model::Role::Pinion);
    CHECK(role_of("third_arbor") == model::Role::Arbor);
    CHECK(role_of("main_plate") == model::Role::Plate);

    const auto train = kinematics::analyze_train(m, m.beat_rate_vph);
    CHECK(train.warnings.empty());
    CHECK(train.escape_teeth == 15);
    // Component order after STEP import is not guaranteed, so find the fourth wheel by name.
    int fourth_wheel = -1, escape_wheel = -1;
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        if (m.components[i].features.name == "fourth_wheel") fourth_wheel = static_cast<int>(i);
        if (m.components[i].features.name == "escape_wheel") escape_wheel = static_cast<int>(i);
    }
    REQUIRE(fourth_wheel >= 0);
    REQUIRE(escape_wheel >= 0);
    CHECK(train.mobiles[m.mobile_of(fourth_wheel)].revs_per_hour / 60.0 == Approx(1.0));
    CHECK(train.mobiles[m.mobile_of(escape_wheel)].revs_per_hour / 60.0 == Approx(14.0));

    // The saved project reproduces the same analysis.
    const auto reloaded = model::from_toml(model::to_toml(m));
    const auto again = kinematics::analyze_train(reloaded, reloaded.beat_rate_vph);
    CHECK(again.root_period_hours == Approx(train.root_period_hours));
}
