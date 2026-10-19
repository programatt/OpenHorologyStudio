// SPDX-License-Identifier: Apache-2.0
#include <filesystem>
#include <fstream>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "app/import.hpp"
#include "geometry/pmi.hpp"
#include "manufacturing/tolerance.hpp"
#include "geometry/procedural.hpp"

using namespace ohs;
using Catch::Approx;

TEST_CASE("STEP PMI: dimensions with tolerances survive a write and a read") {
    const auto path = std::filesystem::temp_directory_path() / "ohs_pmi_test.step";
    const auto parts = geometry::make_test_movement();
    REQUIRE(parts.size() > 3);
    const std::string a = parts[1].name, b = parts[2].name;
    const std::vector<geometry::PmiDimension> dims = {{a, "diameter", 0.80, 0.003, 0.001},
                                                      {b, "length", 4.25, 0.020, 0.020},
                                                      {a, "radius", 1.5, 0.004, 0.004}};
    geometry::export_step_with_pmi(parts, dims, path);
    const auto back = geometry::read_step_pmi(path);
    std::filesystem::remove(path);
    REQUIRE(back.size() == dims.size());
    for (const auto& want : dims) {
        const auto it = std::find_if(back.begin(), back.end(), [&](const geometry::PmiDimension& d) { return d.kind == want.kind; });
        REQUIRE(it != back.end());
        CHECK(it->part == want.part);
        CHECK(it->nominal == Approx(want.nominal));
        CHECK(it->plus == Approx(want.plus));
        CHECK(it->minus == Approx(want.minus));
    }
}

TEST_CASE("STEP PMI: tolerances from the file set the tolerance table by part role, the loosest winning") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto plain = dir / "ohs_pmi_plain.step", with_pmi = dir / "ohs_pmi_dims.step";
    const auto parts = geometry::make_test_movement();
    geometry::export_step(parts, plain);
    const model::Movement m = app::import_movement(plain, {});

    std::string wheel, wheel2, plate;
    for (const auto& c : m.components) {
        if (model::is_rotating_role(c.role) && c.role != model::Role::Arbor && c.features.teeth > 0) (wheel.empty() ? wheel : wheel2) = c.features.name;
        if (c.role == model::Role::Plate) plate = c.features.name;
    }
    REQUIRE_FALSE(wheel.empty());
    REQUIRE_FALSE(wheel2.empty());
    REQUIRE_FALSE(plate.empty());
    const std::vector<geometry::PmiDimension> dims = {
        {wheel, "diameter", 0.80, 0.002, 0.002}, {wheel2, "diameter", 0.80, 0.005, 0.001},  // the looser +0.005 wins
        {plate, "diameter", 1.20, 0.008, 0.008}, {plate, "length", 9.0, 0.5, 0.5},           // a length on a plate: not mapped
        {wheel, "radius", 2.0, 0.9, 0.9}};                                                    // radii are not mapped either
    geometry::export_step_with_pmi(parts, dims, with_pmi);

    manufacturing::ToleranceConfig cfg;
    const manufacturing::ToleranceConfig before = cfg;
    const auto applied = manufacturing::apply_pmi(cfg, geometry::read_step_pmi(with_pmi), m);
    std::filesystem::remove(with_pmi);

    // A STEP file without PMI has no dimensions, and changes nothing; a missing file is an error.
    CHECK(geometry::read_step_pmi(plain).empty());
    manufacturing::ToleranceConfig untouched;
    CHECK(manufacturing::apply_pmi(untouched, {}, m).empty());
    CHECK(untouched.pivot_diameter_tol == before.pivot_diameter_tol);
    CHECK_THROWS_AS(geometry::read_step_pmi(dir / "ohs_no_such_file.step"), Error);
    std::filesystem::remove(plain);

    REQUIRE(applied.size() == 2);
    CHECK(cfg.pivot_diameter_tol == Approx(0.005));
    CHECK(cfg.hole_diameter_tol == Approx(0.008));
    CHECK(cfg.arbor_length_tol == before.arbor_length_tol);  // nothing applied: unchanged
    CHECK(cfg.bridge_gap_tol == before.bridge_gap_tol);
    for (const auto& a : applied) CHECK_FALSE(a.source.empty());

}
