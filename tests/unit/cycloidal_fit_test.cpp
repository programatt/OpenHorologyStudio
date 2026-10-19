// SPDX-License-Identifier: Apache-2.0
#include <filesystem>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "app/import.hpp"
#include "explain/explain.hpp"
#include "gear/gear.hpp"
#include "geometry/features.hpp"
#include "geometry/step_io.hpp"
#include "manufacturing/gear_export.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"

using namespace ohs;
using Catch::Approx;

namespace {

gear::GearSpec cycloidal(int teeth, double module, double rolling) {
    gear::GearSpec s;
    s.teeth = teeth;
    s.module = module;
    s.profile = gear::Profile::Cycloidal;
    s.rolling_radius = rolling;
    s.root_fillet = 0.2;
    return s;
}

}  // namespace

TEST_CASE("the cycloidal flank fit recovers the pitch and rolling radii of a generated wheel") {
    for (auto [teeth, rolling] : {std::pair{12, 3.0}, {30, 3.0}, {30, 2.0}, {60, 3.0}}) {
        const gear::GearSpec s = cycloidal(teeth, 0.2, rolling);
        const auto f = geometry::extract_features({"w", manufacturing::gear_solid(s, 0.4)}, {});
        INFO(teeth << " teeth, rolling circle " << rolling << " modules");
        REQUIRE(f.teeth == teeth);
        REQUIRE(f.flank_cycloid_rms >= 0);
        CHECK(f.flank_cycloid_rms < 1e-3 * s.module);
        CHECK(f.flank_cycloid_pitch_radius == Approx(s.pitch_radius()).epsilon(2e-3));
        CHECK(f.flank_cycloid_rolling_radius == Approx(rolling * s.module).epsilon(5e-3));
    }
}

TEST_CASE("an involute wheel is not offered a cycloidal fit") {
    gear::GearSpec s;
    s.teeth = 30;
    s.module = 0.2;
    const auto f = geometry::extract_features({"w", manufacturing::gear_solid(s, 0.4)}, {});
    CHECK(f.flank_rms < 1e-3 * s.module);
    CHECK(f.flank_cycloid_rms < 0);
}

TEST_CASE("a meshing cycloidal wheel and pinion rig as cycloidal with their rolling circle") {
    const gear::GearSpec wheel = cycloidal(30, 0.2, 3.0), pinion = cycloidal(12, 0.2, 3.0);
    const auto step = std::filesystem::temp_directory_path() / "ohs_cycloidal_pair.step";
    std::vector<geometry::ExportPart> parts(2);
    parts[0].name = "wheel";
    parts[0].shape = manufacturing::gear_solid(wheel, 0.4);
    parts[1].name = "pinion";
    parts[1].shape = manufacturing::gear_solid(pinion, 0.4);
    parts[1].dx = wheel.pitch_radius() + pinion.pitch_radius();
    geometry::export_step(parts, step);

    app::ImportOptions opt;
    const model::Movement m = app::import_movement(step, opt);
    std::filesystem::remove(step);
    REQUIRE(m.components.size() == 2);
    REQUIRE(m.meshes.size() == 1);
    for (const auto& c : m.components) {
        INFO(c.features.name);
        REQUIRE(c.gear);
        CHECK(c.gear->profile == "cycloidal");
        CHECK(c.gear->module == Approx(0.2).epsilon(5e-3));
        CHECK(c.gear->rolling_radius == Approx(3.0).epsilon(1e-2));
    }

    // The fit survives the project file, and Explain says what it is.
    const model::Movement back = model::from_toml(model::to_toml(m));
    REQUIRE(back.components[0].gear);
    CHECK(back.components[0].gear->profile == "cycloidal");
    CHECK(back.components[0].gear->rolling_radius == Approx(m.components[0].gear->rolling_radius));
    CHECK(back.components[0].features.flank_cycloid_rms == Approx(m.components[0].features.flank_cycloid_rms));
    const auto info = explain::explain_part(back, kinematics::analyze_train(back, 0.0, 0), "wheel");
    bool said = false;
    for (const auto& f : info.facts) said = said || f.find("cycloidal flank, rolling circle 3") != std::string::npos;
    CHECK(said);
}
