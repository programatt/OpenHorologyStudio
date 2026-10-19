// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <numbers>

#include <BRepBuilderAPI_Transform.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <TopLoc_Location.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include "core/error.hpp"
#include "geometry/features.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"

using namespace ohs;
using Catch::Approx;

namespace {
std::filesystem::path temp_path(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("ohs_test_" + name);
}
}  // namespace

TEST_CASE("gear tooth counts, radius and bore are measured from the B-rep") {
    const int teeth = GENERATE(6, 10, 15, 60, 75, 84, 120);
    const double module = 0.15;
    const geometry::ImportedPart part{"gear", geometry::make_gear(teeth, module, 1.0, 0.8, 0.1)};
    const auto f = geometry::extract_features(part);

    INFO("teeth " << teeth);
    CHECK(f.teeth == teeth);
    CHECK(f.tooth_confidence > 0.7);
    CHECK(f.outer_radius == Approx((teeth + 2) * module / 2).epsilon(0.002));
    CHECK(f.bore_radius == Approx(0.1).epsilon(0.01));
    CHECK(f.center.x == Approx(0).margin(1e-3));
    CHECK(f.center.y == Approx(0).margin(1e-3));
    CHECK(f.z_min == Approx(1.0));
    CHECK(f.z_max == Approx(1.8));
}

TEST_CASE("plain parts have no teeth and no bore") {
    const auto arbor = geometry::extract_features({"arbor", geometry::make_arbor(0.1, 0, 3)});
    CHECK(arbor.teeth == 0);
    CHECK(arbor.bore_radius == 0.0);
    CHECK(arbor.outer_radius == Approx(0.1).epsilon(0.01));

    const auto plate = geometry::extract_features({"plate", geometry::make_plate(0, 0, 30, -1, -0.5)});
    CHECK(plate.teeth == 0);  // a square is 4-fold, below min_teeth
    CHECK(plate.extent_x == Approx(30));
}

TEST_CASE("a CAD export with Y as the dial normal is handled with the dial axis option") {
    gp_Trsf z_to_y;  // rotate -90 deg about X so Z maps to Y
    z_to_y.SetRotation(gp_Ax1(gp::Origin(), gp::DX()), -std::numbers::pi / 2);
    const TopoDS_Shape gear = geometry::make_gear(40, 0.15, 0.0, 0.8, 0.1);
    const geometry::ImportedPart rotated{"gear", BRepBuilderAPI_Transform(gear, z_to_y, true).Shape()};

    geometry::FeatureOptions opt;
    opt.dial_axis = geometry::Axis::Y;
    const auto f = geometry::extract_features(rotated, opt);
    CHECK(f.teeth == 40);
    CHECK(f.z_max - f.z_min == Approx(0.8));
}

TEST_CASE("STEP export then import keeps names, positions and geometry") {
    const auto path = temp_path("movement.step");
    const auto original = geometry::make_test_movement();
    geometry::export_step(original, path);
    const auto imported = geometry::import_step(path);
    std::filesystem::remove(path);

    REQUIRE(imported.size() == original.size());
    for (const auto& want : original) {
        const auto it = std::find_if(imported.begin(), imported.end(),
                                     [&](const geometry::ImportedPart& p) { return p.name == want.name; });
        REQUIRE(it != imported.end());
        const auto got = geometry::extract_features(*it);
        const auto expect = geometry::extract_features({want.name, want.shape.Moved(TopLoc_Location(gp_Trsf()))});
        CHECK(got.volume == Approx(expect.volume).epsilon(1e-6));
        CHECK(got.teeth == expect.teeth);
        // The component transform of the original is a plain translation by (dx, dy, dz).
        CHECK(got.center.x == Approx(expect.center.x + want.dx).margin(1e-4));
        CHECK(got.center.y == Approx(expect.center.y + want.dy).margin(1e-4));
    }
}

TEST_CASE("unreadable STEP input fails cleanly") {
    CHECK_THROWS_AS(geometry::import_step("/nonexistent/x.step"), Error);

    const auto path = temp_path("garbage.step");
    {
        std::ofstream(path) << "ISO-10303-21; this is not a step file";
    }
    CHECK_THROWS_AS(geometry::import_step(path), Error);
    std::filesystem::remove(path);
}

TEST_CASE("the teeth on the inside of a ring are counted, and a gear's outside is left alone") {
    const double module = 0.8;
    const geometry::ImportedPart ring{"date", geometry::make_ring_gear(31, module, 0.0, 0.4, 15.0)};
    const auto f = geometry::extract_features(ring);
    CHECK(f.inner_teeth == 31);
    CHECK(f.teeth == 0);  // the outside is a plain circle
    CHECK(f.inner_tip_radius == Approx((31 - 2.5) * module / 2).epsilon(0.01));
    CHECK(f.inner_root_radius == Approx((31 + 2) * module / 2).epsilon(0.01));
    CHECK(f.outer_radius == Approx(15.0).epsilon(0.002));

    // A gear with a bore, a plain disc and a solid plate have no internal teeth.
    CHECK(geometry::extract_features({"gear", geometry::make_gear(40, 0.15, 0, 0.8, 0.1)}).inner_teeth == 0);
    CHECK(geometry::extract_features({"plate", geometry::make_plate(0, 0, 30, -1, -0.5)}).inner_teeth == 0);
}
