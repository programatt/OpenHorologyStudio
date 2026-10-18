// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "plugins/plugin_loader.hpp"

using namespace ohs;
using namespace ohs::plugins;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

mesh::Ring rect(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

mesh::Ring circle(double cx, double cy, double r, int n = 48) {
    mesh::Ring ring;
    for (int i = 0; i < n; ++i) ring.push_back({cx + r * std::cos(2 * std::numbers::pi * i / n), cy + r * std::sin(2 * std::numbers::pi * i / n)});
    return ring;
}

manufacturing::PartGeometry part_with_hole(double radius) {
    manufacturing::PartGeometry g;
    g.name = "bridge";
    g.role = model::Role::Bridge;
    g.thickness = 0.5;
    g.region = {{rect(0, 0, 10, 6), circle(5, 3, radius)}};
    return g;
}

}  // namespace

TEST_CASE("example plugin: loads, reports its identity and flags small holes") {
    const auto plugin = DfmPlugin::load(OHS_EXAMPLE_PLUGIN);
    CHECK(plugin->name() == "min-hole-example");
    CHECK(plugin->version() == "1.0.0");
    CHECK_FALSE(plugin->description().empty());

    auto findings = plugin->check(part_with_hole(0.15));  // 0.3 mm hole
    REQUIRE(findings.size() == 1);
    CHECK(findings[0].severity == manufacturing::Severity::Error);
    CHECK(findings[0].process == "min-hole-example");
    CHECK(findings[0].part == "bridge");
    CHECK(findings[0].rule == "min_hole");
    CHECK(findings[0].value == Approx(0.3).epsilon(0.01));
    CHECK(findings[0].limit == Approx(0.5));
    CHECK(findings[0].has_location);
    CHECK_THAT(findings[0].message, ContainsSubstring("below the 0.50 mm drill"));
    CHECK_THAT(findings[0].fix, ContainsSubstring("at least 0.50 mm"));

    findings = plugin->check(part_with_hole(0.6));
    REQUIRE(findings.size() == 1);
    CHECK(findings[0].severity == manufacturing::Severity::Info);
    CHECK_FALSE(findings[0].has_location);
}

TEST_CASE("example plugin gets the outline counter-clockwise and the holes clockwise, whatever the order given") {
    const auto plugin = DfmPlugin::load(OHS_EXAMPLE_PLUGIN);
    manufacturing::PartGeometry g = part_with_hole(0.15);
    std::reverse(g.region.rings[0].begin(), g.region.rings[0].end());  // clockwise outline
    std::reverse(g.region.rings[1].begin(), g.region.rings[1].end());
    const auto findings = plugin->check(g);
    REQUIRE(findings.size() == 1);
    CHECK(findings[0].severity == manufacturing::Severity::Error);

    // Several small holes: one finding each.
    g = part_with_hole(0.15);
    g.region.rings.push_back(circle(2, 3, 0.1));
    g.region.rings.push_back(circle(8, 3, 0.2));
    CHECK(plugin->check(g).size() == 3);
}

TEST_CASE("plugins with more findings than the first buffer are asked again with a bigger one") {
    const auto plugin = DfmPlugin::load(OHS_MANY_PLUGIN);
    const auto findings = plugin->check(part_with_hole(0.6));
    REQUIRE(findings.size() == 40);
    CHECK(findings[0].message == "finding number 0");
    CHECK(findings[39].message == "finding number 39");
    CHECK(findings[39].value == Approx(39.0));
    CHECK(findings[1].severity == manufacturing::Severity::Warning);
    CHECK(findings[2].severity == manufacturing::Severity::Error);
    CHECK(findings[0].severity == manufacturing::Severity::Info);
}

TEST_CASE("a plugin's internal error becomes a finding, not a crash") {
    const auto plugin = DfmPlugin::load(OHS_ERROR_PLUGIN);
    const auto findings = plugin->check(part_with_hole(0.6));
    REQUIRE(findings.size() == 1);
    CHECK(findings[0].severity == manufacturing::Severity::Error);
    CHECK_THAT(findings[0].message, ContainsSubstring("internal error (-7)"));
    CHECK(findings[0].rule == "plugin");
}

TEST_CASE("loading refuses what it cannot trust, with the reason") {
    auto reason = [](const std::string& path) {
        try {
            DfmPlugin::load(path);
        } catch (const Error& e) {
            return std::string(e.what());
        }
        return std::string("loaded");
    };
    CHECK_THAT(reason("/nonexistent/libnothing.so"), ContainsSubstring("cannot load plugin"));
    CHECK_THAT(reason(OHS_NO_ENTRY_PLUGIN), ContainsSubstring("does not export ohs_plugin_dfm_rule_set"));
    CHECK_THAT(reason(OHS_INCOMPLETE_PLUGIN), ContainsSubstring("incomplete rule set"));
    const std::string major = reason(OHS_ABI_V2_PLUGIN);
    CHECK_THAT(major, ContainsSubstring("built for plugin ABI 2.0"));
    CHECK_THAT(major, ContainsSubstring("speaks 1.0"));
    const std::string minor = reason(OHS_ABI_NEWER_MINOR_PLUGIN);
    CHECK_THAT(minor, ContainsSubstring("needs plugin ABI 1.9"));
    CHECK_THAT(minor, ContainsSubstring("update OHS"));
}
