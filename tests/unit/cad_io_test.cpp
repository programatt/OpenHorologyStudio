// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "gear/gear.hpp"
#include "geometry/cad_io.hpp"
#include "geometry/features.hpp"
#include "geometry/procedural.hpp"
#include "manufacturing/gear_export.hpp"
#include "report/drawing.hpp"

using namespace ohs;
using Catch::Approx;

namespace {

std::filesystem::path temp_path(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("ohs_cad_" + name);
}

double volume(const TopoDS_Shape& s) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

std::filesystem::path write_dxf(const std::string& name, const std::string& header, const std::string& entities) {
    const auto path = temp_path(name);
    std::ofstream out(path);
    out << "0\nSECTION\n2\nHEADER\n" << header << "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" << entities << "0\nENDSEC\n0\nEOF\n";
    return path;
}

std::string lwpolyline(std::initializer_list<std::array<double, 3>> vertices, bool closed, const std::string& extra = "") {
    std::ostringstream os;
    os << "0\nLWPOLYLINE\n8\n0\n90\n" << vertices.size() << "\n70\n" << (closed ? 1 : 0) << "\n" << extra;
    for (const auto& v : vertices) {
        os << "10\n" << v[0] << "\n20\n" << v[1] << "\n";
        if (v[2] != 0.0) os << "42\n" << v[2] << "\n";
    }
    return os.str();
}

std::string circle(double x, double y, double r) {
    std::ostringstream os;
    os << "0\nCIRCLE\n8\n0\n10\n" << x << "\n20\n" << y << "\n30\n0\n40\n" << r << "\n";
    return os.str();
}

}  // namespace

TEST_CASE("IGES export then import keeps the solids and their volume") {
    const auto path = temp_path("movement.igs");
    const auto original = geometry::make_test_movement();
    geometry::export_iges(original, path);
    const auto imported = geometry::import_iges(path);
    std::filesystem::remove(path);

    double want = 0.0, got = 0.0;
    for (const auto& p : original) want += volume(p.shape);
    for (const auto& p : imported) got += volume(p.shape);
    CHECK(imported.size() == original.size());
    CHECK(got == Approx(want).epsilon(1e-4));
    CHECK(imported.front().name == "part");
    CHECK(imported[1].name == "part#2");
    CHECK_THROWS_AS(geometry::import_iges("/nonexistent/x.igs"), Error);
}

TEST_CASE("unreadable IGES input fails cleanly") {
    const auto path = temp_path("garbage.igs");
    std::ofstream(path) << "this is not an IGES file\n";
    CHECK_THROWS_AS(geometry::import_iges(path), Error);
    std::filesystem::remove(path);
}

TEST_CASE("DXF: a rectangle with a round hole becomes one extruded part") {
    const auto path = write_dxf("plate.dxf", "9\n$INSUNITS\n70\n4\n",
                                lwpolyline({{0, 0, 0}, {20, 0, 0}, {20, 10, 0}, {0, 10, 0}}, true, "39\n0.8\n") + circle(5, 5, 2));
    const auto parts = geometry::import_dxf(path);
    std::filesystem::remove(path);
    REQUIRE(parts.size() == 1);
    CHECK(parts[0].name == "profile");
    CHECK(volume(parts[0].shape) == Approx((200.0 - std::numbers::pi * 4.0) * 0.8).epsilon(1e-6));
}

TEST_CASE("DXF: bulges make arcs, and loose LINE and ARC entities are chained into a contour") {
    // Stadium: two 6 mm straights and two semicircles of radius 2.
    const auto stadium = write_dxf("stadium.dxf", "", lwpolyline({{2, 0, 0}, {8, 0, 1}, {8, 4, 0}, {2, 4, 1}}, true));
    const auto a = geometry::import_dxf(stadium, 2.0);
    std::filesystem::remove(stadium);
    REQUIRE(a.size() == 1);
    CHECK(volume(a[0].shape) == Approx((24.0 + std::numbers::pi * 4.0) * 2.0).epsilon(1e-6));

    // The same shape from LINE and ARC entities, listed out of order.
    const auto loose = write_dxf("loose.dxf", "",
                                 "0\nARC\n8\n0\n10\n2\n20\n2\n40\n2\n50\n90\n51\n270\n"
                                 "0\nLINE\n8\n0\n10\n8\n20\n4\n11\n2\n21\n4\n"
                                 "0\nARC\n8\n0\n10\n8\n20\n2\n40\n2\n50\n270\n51\n90\n"
                                 "0\nLINE\n8\n0\n10\n2\n20\n0\n11\n8\n21\n0\n");
    const auto b = geometry::import_dxf(loose, 2.0);
    std::filesystem::remove(loose);
    REQUIRE(b.size() == 1);
    CHECK(volume(b[0].shape) == Approx((24.0 + std::numbers::pi * 4.0) * 2.0).epsilon(1e-6));
}

TEST_CASE("DXF: old-style POLYLINE, inches, and an island inside a hole") {
    std::string square = "0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n";
    for (auto [x, y] : {std::pair{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}) {
        square += "0\nVERTEX\n8\n0\n10\n" + std::to_string(x) + "\n20\n" + std::to_string(y) + "\n";
    }
    square += "0\nSEQEND\n8\n0\n";
    // A 2 x 2 inch plate with a hole of radius 0.8 inch and, standing in the hole, a pin of radius 0.3 inch.
    const auto path = write_dxf("inch.dxf", "9\n$INSUNITS\n70\n1\n", square + circle(1, 1, 0.8) + circle(1, 1, 0.3));
    const auto parts = geometry::import_dxf(path, 1.0);
    std::filesystem::remove(path);
    REQUIRE(parts.size() == 2);  // the plate with its hole, and the pin standing in the hole
    const double in = 25.4;
    const double plate = (4.0 - std::numbers::pi * 0.64) * in * in;
    const double pin = std::numbers::pi * 0.09 * in * in;
    std::vector<double> vols{volume(parts[0].shape), volume(parts[1].shape)};
    std::sort(vols.begin(), vols.end());
    CHECK(vols[0] == Approx(pin * 1.0).epsilon(1e-6));
    CHECK(vols[1] == Approx(plate * 1.0).epsilon(1e-6));
    CHECK(parts[1].name == "profile#2");
}

TEST_CASE("DXF: bad input is refused with a message") {
    const auto open = write_dxf("open.dxf", "", "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n5\n21\n0\n0\nLINE\n8\n0\n10\n5\n20\n0\n11\n5\n21\n5\n");
    CHECK_THROWS_WITH(geometry::import_dxf(open), Catch::Matchers::ContainsSubstring("open contour"));
    std::filesystem::remove(open);

    const auto empty = write_dxf("empty.dxf", "", "");
    CHECK_THROWS_AS(geometry::import_dxf(empty), Error);
    std::filesystem::remove(empty);

    const auto garbage = temp_path("garbage.dxf");
    std::ofstream(garbage) << "hello\nworld\n";
    CHECK_THROWS_AS(geometry::import_dxf(garbage), Error);
    std::filesystem::remove(garbage);

    CHECK_THROWS_AS(geometry::import_dxf("/nonexistent/x.dxf"), Error);
    CHECK_THROWS_AS(geometry::import_cad("part.obj"), Error);
}

TEST_CASE("DXF: a generated gear profile comes back as a 24-tooth wheel") {
    gear::GearSpec spec;
    spec.teeth = 24;
    spec.module = 0.2;
    spec.bore_radius = 0.3;
    const auto path = temp_path("gear.dxf");
    std::ofstream(path) << report::to_dxf(manufacturing::gear_profile(spec));
    const auto parts = geometry::import_cad(path);
    std::filesystem::remove(path);
    REQUIRE(!parts.empty());
    const auto f = geometry::extract_features(parts[0], {});
    CHECK(f.teeth == 24);
    CHECK(f.outer_radius == Approx(spec.tip_radius()).epsilon(2e-3));
}
