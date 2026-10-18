// SPDX-License-Identifier: Apache-2.0
// What running the plan's gates on the real OM10 file taught, as regression tests on the generated
// movement: a CAD file need not have Z as its dial axis, a smooth barrel has no teeth to mesh, and a
// rig without a barrel can still be read from the escape wheel back.
#include <filesystem>
#include <numbers>

#include <BRepBuilderAPI_Transform.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include "app/import.hpp"
#include "core/error.hpp"
#include "geometry/features.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
namespace fs = std::filesystem;

namespace {

// The generated movement written to `path` with its dial axis (Z) turned onto `axis` of the CAD file:
// the whole placed assembly is rotated, as a CAD export with another up direction would be.
void write_turned_movement(geometry::Axis axis, const fs::path& path) {
    const fs::path flat = path.string() + ".z.step";
    geometry::export_step(geometry::make_test_movement(), flat);
    gp_Trsf rotation;
    if (axis == geometry::Axis::Y) rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), -std::numbers::pi / 2);  // Z -> Y
    else if (axis == geometry::Axis::X) rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), std::numbers::pi / 2);  // Z -> X
    std::vector<geometry::ExportPart> turned;
    for (const auto& p : geometry::import_step(flat)) {
        turned.push_back({p.name, BRepBuilderAPI_Transform(p.shape, rotation, true).Shape(), 0, 0, 0, 0});
    }
    geometry::export_step(turned, path);
}

}  // namespace

TEST_CASE("the dial axis is found from the geometry, whichever way the CAD file points") {
    for (const auto axis : {geometry::Axis::Z, geometry::Axis::Y, geometry::Axis::X}) {
        const fs::path file = fs::temp_directory_path() / "ohs-findings-axis-detect.step";
        write_turned_movement(axis, file);
        CHECK(geometry::detect_dial_axis(geometry::import_step(file)) == axis);
    }
    CHECK(geometry::detect_dial_axis({}) == geometry::Axis::Z);
}

TEST_CASE("a movement exported with Y up imports like the Z-up one, and the project remembers the axis") {
    const fs::path dir = fs::temp_directory_path() / "ohs-findings-axis";
    fs::create_directories(dir);
    write_turned_movement(geometry::Axis::Z, dir / "z.step");
    write_turned_movement(geometry::Axis::Y, dir / "y.step");

    // Import both orientations with the axis found automatically.
    app::ImportOptions opt;
    opt.auto_axis = true;
    opt.rig.beat_rate_vph = 25200;
    const model::Movement z_up = app::import_movement(dir / "z.step", opt);
    const model::Movement y_up = app::import_movement(dir / "y.step", opt);
    CHECK(z_up.dial_axis == "z");
    CHECK(y_up.dial_axis == "y");
    CHECK(y_up.mobiles.size() == z_up.mobiles.size());
    CHECK(y_up.meshes.size() == z_up.meshes.size());
    CHECK(y_up.meshes.size() == 4);

    // Without the axis, the Y-up file is misread (this is what the OM10 did: no meshes at all).
    app::ImportOptions wrong;
    wrong.features.dial_axis = geometry::Axis::Z;
    CHECK(app::import_movement(dir / "y.step", wrong).meshes.size() < 4);

    // The axis is stored in the project file.
    const model::Movement again = model::from_toml(model::to_toml(y_up));
    CHECK(again.dial_axis == "y");
    // Files written before the axis was stored read as Z.
    std::string old_file = model::to_toml(z_up);
    const auto at = old_file.find("dial_axis");
    REQUIRE(at != std::string::npos);
    old_file.erase(at, old_file.find('\n', at) - at + 1);
    CHECK(model::from_toml(old_file).dial_axis == "z");
}

TEST_CASE("a mesh the geometry does not show can be declared, and it survives re-inference and the file") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    REQUIRE(m.meshes.size() == 4);

    // Forget the barrel / centre mesh, as happens with a smooth barrel drum.
    const auto first = m.meshes.front();
    const std::string a = m.components[first.a].features.name, b = m.components[first.b].features.name;
    m.meshes.erase(m.meshes.begin());
    REQUIRE(m.meshes.size() == 3);

    rigging::add_manual_mesh(m, a, b, {});
    REQUIRE(m.meshes.size() == 4);
    const auto& declared = m.meshes.back();
    CHECK(declared.manual);
    CHECK(declared.confidence == 1.0);
    CHECK(declared.module == Catch::Approx(first.module).epsilon(0.01));
    // Declaring it again changes nothing.
    rigging::add_manual_mesh(m, a, b, {});
    CHECK(m.meshes.size() == 4);

    // A role change re-infers the structure; the declared mesh stays, the geometric ones are redone.
    rigging::set_role(m, "third_arbor", model::Role::Arbor, {});
    int manual = 0;
    for (const auto& j : m.meshes) manual += j.manual;
    CHECK(m.meshes.size() == 4);
    CHECK(manual >= 1);

    // And it is in the file.
    const model::Movement back = model::from_toml(model::to_toml(m));
    int manual_back = 0;
    for (const auto& j : back.meshes) manual_back += j.manual;
    CHECK(back.meshes.size() == 4);
    CHECK(manual_back == manual);

    // Mistakes are refused with a reason.
    CHECK_THROWS_WITH(rigging::add_manual_mesh(m, a, "no_such_part", {}), Catch::Matchers::ContainsSubstring("no component named"));
    CHECK_THROWS_WITH(rigging::add_manual_mesh(m, a, a, {}), Catch::Matchers::ContainsSubstring("itself"));
    CHECK_THROWS_WITH(rigging::add_manual_mesh(m, "center_pinion", "center_wheel", {}), Catch::Matchers::ContainsSubstring("same mobile"));
    CHECK_THROWS_WITH(rigging::add_manual_mesh(m, "main_plate", a, {}), Catch::Matchers::ContainsSubstring("rotating mobiles"));
}

TEST_CASE("without a barrel the train starts at the far end of the chain from the escape wheel") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto with_barrel = kinematics::analyze_train(m, 25200);
    REQUIRE(with_barrel.root >= 0);
    CHECK(with_barrel.warnings.empty());
    CHECK(with_barrel.root_period_hours > 0);

    // Take the barrel's role away: the barrel gear becomes an ordinary wheel on the same mobile.
    rigging::set_role(m, "barrel", model::Role::Wheel, {});
    const auto without = kinematics::analyze_train(m, 25200);
    REQUIRE(without.root >= 0);
    CHECK(without.mobiles[without.root].name == with_barrel.mobiles[with_barrel.root].name);  // still the end of the chain
    CHECK(without.root_period_hours == Catch::Approx(with_barrel.root_period_hours));
    bool said_so = false;
    for (const auto& w : without.warnings) said_so |= w.find("no barrel in the rig") != std::string::npos;
    CHECK(said_so);

    // With no escape wheel either there is nothing to start from.
    rigging::set_role(m, "escape_wheel", model::Role::Wheel, {});
    CHECK_THROWS_AS(kinematics::analyze_train(m, 25200), Error);
}
