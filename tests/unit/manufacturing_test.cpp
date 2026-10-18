// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <filesystem>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "core/error.hpp"
#include "geometry/features.hpp"
#include "geometry/step_io.hpp"
#include "manufacturing/collision.hpp"
#include "manufacturing/dfm.hpp"
#include "manufacturing/gear_export.hpp"
#include "manufacturing/part_drawing.hpp"
#include "manufacturing/tolerance.hpp"
#include "report/drawing.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using namespace ohs::manufacturing;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr double kPi = std::numbers::pi;

double ring_area(const mesh::Ring& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
    return 0.5 * std::abs(a);
}

mesh::Ring rect(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

mesh::Ring circle(double cx, double cy, double r, int n = 64) {
    mesh::Ring ring;
    for (int i = 0; i < n; ++i) ring.push_back({cx + r * std::cos(2 * kPi * i / n), cy + r * std::sin(2 * kPi * i / n)});
    return ring;
}

model::Movement synthetic_movement() {
    rigging::RigOptions opt;
    opt.beat_rate_vph = 25200;
    return rigging::auto_rig("synthetic", test::synthetic_train(), opt);
}

double normal_cdf(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

}  // namespace

// ---- gear generator -----------------------------------------------------------------------------------

TEST_CASE("generated wheel round-trips through STEP with its tooth count, diameters and volume") {
    gear::GearSpec s;
    s.teeth = 20;
    s.module = 0.5;
    s.bore_radius = 1.0;
    const auto path = std::filesystem::temp_directory_path() / "ohs_test_gear.step";
    export_gear_step(s, 0.4, "wheel20", path);
    const auto parts = geometry::import_step(path);
    std::filesystem::remove(path);
    REQUIRE(parts.size() == 1);
    CHECK(parts[0].name == "wheel20");

    const auto f = geometry::extract_features(parts[0]);
    CHECK(f.teeth == 20);
    CHECK(f.outer_radius == Approx(s.tip_radius()).epsilon(0.005));
    CHECK(f.bore_radius == Approx(1.0).epsilon(0.01));
    CHECK(f.thickness() == Approx(0.4).epsilon(1e-6));
    GProp_GProps props;
    BRepGProp::VolumeProperties(gear_solid(s, 0.4), props);
    const double area = ring_area(gear::outline(s));
    CHECK(props.Mass() == Approx((area - kPi * 1.0) * 0.4).epsilon(0.01));
    CHECK_THROWS_AS(gear_solid(s, 0.0), Error);
}

TEST_CASE("gear drawing carries views, layers and the gear data a cutter set-up needs") {
    gear::GearSpec s;
    s.teeth = 24;
    s.module = 0.3;
    s.bore_radius = 1.0;
    DrawingNotes notes;
    notes.title = "wheel24";
    notes.thickness = 0.4;
    notes.material = "brass";
    const report::Drawing d = gear_drawing(s, notes);
    const auto summary = report::summarize_dxf(report::to_dxf(d));
    CHECK(summary.closed_polylines == 2);  // the wheel and the side view
    CHECK(summary.circles >= 3);           // bore, pitch circle, root circle
    for (const char* layer : {"OUTLINE", "DIMENSION", "CENTRE", "HIDDEN"}) {
        CHECK(std::find(summary.layers.begin(), summary.layers.end(), layer) != summary.layers.end());
    }
    const std::string svg = report::to_svg(d);
    CHECK_THAT(svg, ContainsSubstring("tip D 7.800"));
    CHECK_THAT(svg, ContainsSubstring("pitch D 7.200"));
    CHECK_THAT(svg, ContainsSubstring("root D 6.450"));
    CHECK_THAT(svg, ContainsSubstring("bore D 2.000"));
    CHECK_THAT(svg, ContainsSubstring("t 0.400"));
    const auto info = gear_info(s, notes);
    REQUIRE(info.size() == 3);
    CHECK_THAT(info[0], ContainsSubstring("24 teeth, module 0.3000, pressure angle 20.0"));
    CHECK_THAT(info[1], ContainsSubstring("Tip diameter 7.800"));

    // The profile-only drawing is the cutting file: just the outline and the bore.
    const auto bare = report::summarize_dxf(report::to_dxf(gear_profile(s)));
    CHECK(bare.closed_polylines == 1);
    CHECK(bare.circles == 1);
    CHECK(bare.texts == 0);
    CHECK(bare.lines == 0);
}

TEST_CASE("gate: the drawing a user machines from is faithful to the solid") {
    gear::GearSpec s;
    s.teeth = 30;
    s.module = 0.4;
    s.bore_radius = 1.2;
    DrawingNotes notes;
    notes.thickness = 0.5;
    const report::Drawing d = gear_drawing(s, notes);

    // Read the polygon back out of the DXF: its area, its extreme radius and the bore circle
    // are the numbers the machinist measures.
    const std::string dxf = report::to_dxf(d);
    std::vector<report::Pt> poly;
    {
        std::istringstream in(dxf);
        std::string code, value;
        bool in_first = false, done = false;
        double x = 0;
        while (std::getline(in, code) && std::getline(in, value) && !done) {
            if (code == "0" && value == "POLYLINE" && poly.empty()) in_first = true;
            else if (code == "0" && value == "SEQEND" && in_first) done = true;
            else if (in_first && code == "10") x = std::stod(value);
            else if (in_first && code == "20") poly.push_back({x, std::stod(value)});
        }
    }
    REQUIRE(poly.size() > 500);
    double area = 0, rmax = 0, rmin = 1e9;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % poly.size()];
        area += p.x * q.y - q.x * p.y;
        rmax = std::max(rmax, std::hypot(p.x, p.y));
        rmin = std::min(rmin, std::hypot(p.x, p.y));
    }
    CHECK(0.5 * area == Approx(ring_area(gear::outline(s))).epsilon(0.002));
    CHECK(rmax == Approx(s.tip_radius()).epsilon(1e-6));
    CHECK(rmin == Approx(s.root_radius()).epsilon(1e-6));
    // The dimensions printed on the drawing are the ones of the geometry.
    CHECK_THAT(report::to_svg(d), ContainsSubstring("tip D " + std::to_string(2 * s.tip_radius()).substr(0, 5)));
    // And the STEP solid made from the same spec has the same tooth count and tip diameter.
    const auto part = geometry::ImportedPart{"w", gear_solid(s, 0.5)};
    const auto f = geometry::extract_features(part);
    CHECK(f.teeth == 30);
    CHECK(2 * f.outer_radius == Approx(2 * s.tip_radius()).epsilon(0.005));
}

// ---- tolerance ------------------------------------------------------------------------------------------------

TEST_CASE("tolerance Monte Carlo reproduces the analytic spread of backlash and shake") {
    const model::Movement m = synthetic_movement();
    ToleranceConfig c;
    c.samples = 40000;
    const ToleranceResult r = run_tolerance(m, c);
    REQUIRE(r.meshes.size() == 4);
    REQUIRE(r.mobiles.size() == 5);

    const double alpha = 20 * kPi / 180;
    const double sp = c.position_tol / 3, ss = c.tooth_thickness_tol / 3;
    const double sd = std::sqrt(std::pow(2 * std::tan(alpha), 2) * 2 * sp * sp + 2 * ss * ss);
    for (const MeshTolerance& mt : r.meshes) {
        INFO(mt.a << " / " << mt.b);
        CHECK(mt.backlash.mean == Approx(mt.nominal_backlash).margin(4 * sd / std::sqrt(40000.0)));
        CHECK(mt.backlash.sd == Approx(sd).epsilon(0.03));
        CHECK(mt.nominal_backlash == Approx(0.08 * 0.15));
        CHECK(mt.bind_fraction == Approx(normal_cdf(-mt.nominal_backlash / sd)).margin(0.004));
        CHECK(mt.centre_distance.sd == Approx(std::sqrt(2) * sp).epsilon(0.03));
        double share = 0;
        for (const auto& s : mt.backlash_sensitivity) share += s.share;
        CHECK(share == Approx(1.0));
        CHECK(mt.backlash_sensitivity.size() == 6);
        // Sorted, largest first.
        for (std::size_t k = 1; k < mt.backlash_sensitivity.size(); ++k) CHECK(mt.backlash_sensitivity[k - 1].share >= mt.backlash_sensitivity[k].share);
    }

    const double sd_end = std::sqrt(c.bridge_gap_tol * c.bridge_gap_tol + c.plate_flatness_tol * c.plate_flatness_tol + c.arbor_length_tol * c.arbor_length_tol) / 3;
    const double out_end = normal_cdf((c.end_shake_min - c.nominal_end_shake) / sd_end) + (1 - normal_cdf((c.end_shake_max - c.nominal_end_shake) / sd_end));
    for (const MobileTolerance& mt : r.mobiles) {
        CHECK(mt.end_shake.mean == Approx(c.nominal_end_shake).margin(0.0002));
        CHECK(mt.end_shake.sd == Approx(sd_end).epsilon(0.03));
        CHECK(mt.end_shake_out_fraction == Approx(out_end).margin(0.004));
        // The arbor length dominates the end shake: it has the largest tolerance.
        CHECK(mt.end_shake_sensitivity.front().input == "arbor length");
        // The smaller of the two pivots' side shakes: mean of the minimum of two normals is mu - sigma / sqrt(pi).
        const double sd_side = std::sqrt(c.hole_diameter_tol * c.hole_diameter_tol + c.pivot_diameter_tol * c.pivot_diameter_tol) / 3;
        CHECK(mt.side_shake.mean == Approx(c.nominal_side_shake - sd_side / std::sqrt(kPi)).margin(0.0001));
    }
    CHECK(r.failing_fraction > 0);
    CHECK(r.failing_fraction < 0.3);
}

TEST_CASE("tolerance results are deterministic, seeded and respond to the tolerances") {
    const model::Movement m = synthetic_movement();
    ToleranceConfig c;
    c.samples = 2000;
    const auto a = run_tolerance(m, c), b = run_tolerance(m, c);
    CHECK(a.meshes[0].backlash.mean == b.meshes[0].backlash.mean);
    CHECK(a.meshes[0].backlash.sd == b.meshes[0].backlash.sd);
    CHECK(a.failing_fraction == b.failing_fraction);
    ToleranceConfig other = c;
    other.seed = 7;
    CHECK(run_tolerance(m, other).meshes[0].backlash.mean != a.meshes[0].backlash.mean);

    // Tight tolerances: nothing binds, everything is inside its window.
    ToleranceConfig tight = c;
    tight.position_tol = 0.002;
    tight.tooth_thickness_tol = 0.002;
    tight.arbor_length_tol = tight.bridge_gap_tol = tight.plate_flatness_tol = 0.004;
    tight.pivot_diameter_tol = tight.hole_diameter_tol = 0.001;
    const auto t = run_tolerance(m, tight);
    CHECK(t.failing_fraction == 0.0);
    // Loose positions make it worse, and then positions are what drive the backlash.
    ToleranceConfig loose = c;
    loose.position_tol = 0.03;
    const auto l = run_tolerance(m, loose);
    CHECK(l.meshes[0].bind_fraction > a.meshes[0].bind_fraction);
    CHECK(l.meshes[0].backlash_sensitivity.front().input.find("pivot position") != std::string::npos);
    // Uniform distributions have a smaller spread for the same +- tolerance (tol/sqrt 3 against tol/3 is bigger: 0.577 vs 0.333).
    ToleranceConfig uniform = c;
    uniform.uniform = true;
    CHECK(run_tolerance(m, uniform).meshes[0].backlash.sd > a.meshes[0].backlash.sd);
}

TEST_CASE("tolerance configuration: TOML and errors") {
    const ToleranceConfig c = tolerance_from_toml("[tolerance]\nsamples = 500\nseed = 9\nposition_tol = 0.02\nuniform = true\nmin_contact_ratio = 1.3\n");
    CHECK(c.samples == 500);
    CHECK(c.seed == 9);
    CHECK(c.position_tol == Approx(0.02));
    CHECK(c.uniform);
    CHECK(c.min_contact_ratio == Approx(1.3));
    CHECK_THROWS_AS(tolerance_from_toml("[tolerance]\nbogus = 1\n"), Error);
    CHECK_THROWS_AS(tolerance_from_toml("nothing = 1"), Error);
    CHECK_THROWS_AS(tolerance_from_toml("[tolerance]\nposition_tol = 'x'\n"), Error);
    CHECK_THROWS_AS(load_tolerance("/nonexistent.toml"), Error);

    const model::Movement m = synthetic_movement();
    ToleranceConfig few;
    few.samples = 3;
    CHECK_THROWS_AS(run_tolerance(m, few), Error);
    model::Movement no_meshes = m;
    no_meshes.meshes.clear();
    CHECK_THROWS_AS(run_tolerance(no_meshes, ToleranceConfig{}), Error);

    // A very low contact-ratio limit never trips; a very high one always does.
    ToleranceConfig lax;
    lax.samples = 500;
    lax.min_contact_ratio = 0.5;
    lax.position_tol = lax.tooth_thickness_tol = 1e-5;
    for (const auto& mt : run_tolerance(m, lax).meshes) CHECK(mt.under_engage_fraction == 0.0);
    ToleranceConfig strict = lax;
    strict.min_contact_ratio = 3.0;
    for (const auto& mt : run_tolerance(m, strict).meshes) CHECK(mt.under_engage_fraction == 1.0);
}

// ---- DFM ------------------------------------------------------------------------------------------------------------

TEST_CASE("internal corner radius and wall thickness are measured from the outline") {
    // A plate with a round hole of radius 0.15: the hole is an internal feature of that radius.
    mesh::Region plate{{rect(0, 0, 10, 4), circle(3, 2, 0.15, 48)}};
    mesh::P2 at{};
    const double r = min_internal_radius(plate, &at);
    CHECK(r == Approx(0.15).epsilon(0.03));
    CHECK(at.x == Approx(3.0).margin(0.2));
    CHECK(min_internal_radius({{rect(0, 0, 10, 4)}}) > 1e20);  // no internal corner at all

    // An L-shaped plate has one sharp internal corner; a square hole has four.
    mesh::Region l{{{{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}}}};
    CHECK(min_internal_radius(l, &at) == 0.0);
    CHECK(at.x == Approx(2.0));
    CHECK(at.y == Approx(2.0));
    CHECK(min_internal_radius({{rect(0, 0, 10, 4), rect(4, 1, 5, 2)}}) == 0.0);

    // Wall: the strip between the hole and the left edge, and a thin strip.
    mesh::Region wide{{rect(0, 0, 10, 4), circle(1.5, 2, 1.0, 96)}};
    CHECK(min_wall_thickness(wide) == Approx(0.5).epsilon(0.02));
    CHECK(min_wall_thickness({{rect(0, 0, 10, 0.18)}}) == Approx(0.18).epsilon(1e-6));
    CHECK(min_wall_thickness({{rect(0, 0, 10, 4)}}) == Approx(4.0).epsilon(1e-6));
}

TEST_CASE("milling rules flag small corners, thin walls and deep pockets, and pass sound parts") {
    const ProcessRules& mill = find_process("mill");
    auto severity_of = [&](const std::vector<Finding>& fs, const std::string& rule) {
        for (const auto& f : fs) {
            if (f.rule == rule) return f.severity;
        }
        FAIL("no finding for rule " << rule);
        return Severity::Info;
    };
    PartGeometry good;
    good.name = "bridge";
    good.thickness = 0.8;
    good.region = {{rect(0, 0, 10, 4), circle(3, 2, 0.6, 64)}};
    auto fs = run_dfm(mill, good);
    CHECK(severity_of(fs, "internal_radius") == Severity::Info);
    CHECK(severity_of(fs, "min_wall") == Severity::Info);
    CHECK(severity_of(fs, "depth_to_diameter") == Severity::Info);

    PartGeometry small_hole = good;
    small_hole.region = {{rect(0, 0, 10, 4), circle(3, 2, 0.1, 48)}};
    fs = run_dfm(mill, small_hole);
    CHECK(severity_of(fs, "internal_radius") == Severity::Error);
    for (const auto& f : fs) {
        if (f.rule == "internal_radius") {
            CHECK_THAT(f.message, ContainsSubstring("smaller than the smallest tool radius 0.150"));
            CHECK_THAT(f.fix, ContainsSubstring("at least 0.300 mm diameter"));
            CHECK(f.has_location);
            CHECK(f.value == Approx(0.1).epsilon(0.03));
            CHECK(f.limit == Approx(0.15));
        }
    }

    PartGeometry thin = good;
    thin.region = {{rect(0, 0, 10, 0.18)}};
    CHECK(severity_of(run_dfm(mill, thin), "min_wall") == Severity::Error);

    // A 4 mm deep pocket of 0.4 mm width: only a 0.3 mm cutter fits, which is 13:1.
    PartGeometry deep = good;
    deep.thickness = 4.0;
    deep.region = {{rect(0, 0, 10, 4), circle(5, 2, 0.2, 48)}};
    CHECK(severity_of(run_dfm(mill, deep), "depth_to_diameter") == Severity::Warning);

    // Toothed wheels are not milled.
    PartGeometry wheel = good;
    wheel.teeth = 24;
    fs = run_dfm(mill, wheel);
    REQUIRE(fs.size() == 1);
    CHECK(fs[0].rule == "applicability");
}

TEST_CASE("wheel-cutting rules: cutter module, tooth counts, undercut, pinion leaves") {
    const ProcessRules& wc = find_process("wheel-cutting");
    auto find = [](const std::vector<Finding>& fs, const std::string& rule) -> const Finding& {
        for (const auto& f : fs) {
            if (f.rule == rule) return f;
        }
        FAIL("no finding for " << rule);
        return fs.front();
    };
    PartGeometry w;
    w.name = "wheel";
    w.role = model::Role::Wheel;
    w.teeth = 60;
    w.module = 0.15;
    auto fs = run_dfm(wc, w);
    CHECK(find(fs, "module_cutter").severity == Severity::Info);
    CHECK(find(fs, "min_teeth").severity == Severity::Info);
    CHECK(find(fs, "undercut").severity == Severity::Info);

    w.module = 0.137;  // between the 0.12 and 0.15 cutters
    const auto module_findings = run_dfm(wc, w);  // keep the vector alive: find() returns a reference into it
    const Finding& m = find(module_findings, "module_cutter");
    CHECK(m.severity == Severity::Error);
    CHECK_THAT(m.message, ContainsSubstring("nearest 0.150"));

    w.module = 0.1505;  // within 2 % of a cutter
    CHECK(find(run_dfm(wc, w), "module_cutter").severity == Severity::Info);

    w.teeth = 7;
    CHECK(find(run_dfm(wc, w), "min_teeth").severity == Severity::Error);
    w.teeth = 12;
    const auto undercut_findings = run_dfm(wc, w);
    const Finding& u = find(undercut_findings, "undercut");
    CHECK(u.severity == Severity::Warning);
    CHECK(u.limit == Approx(2.0 / std::pow(std::sin(20 * kPi / 180), 2)));
    w.pressure_angle_deg = 25;  // a steeper profile undercuts less
    CHECK(find(run_dfm(wc, w), "undercut").severity == Severity::Info);

    PartGeometry pinion = w;
    pinion.role = model::Role::Pinion;
    pinion.teeth = 5;
    CHECK(find(run_dfm(wc, pinion), "pinion_leaves").severity == Severity::Error);
    pinion.teeth = 8;
    CHECK(find(run_dfm(wc, pinion), "pinion_leaves").severity == Severity::Info);
    // Parts without teeth are outside this process.
    PartGeometry plain;
    plain.region = {{rect(0, 0, 10, 4)}};
    CHECK(run_dfm(wc, plain).empty());
}

TEST_CASE("jewel seats: press-fit interference against the window") {
    const ProcessRules& jw = find_process("jewelling");
    PartGeometry plate;
    plate.name = "plate";
    plate.role = model::Role::Plate;
    plate.thickness = 0.5;
    // 1.99 mm hole for a 2.0 mm jewel: 10 um, inside 4 to 20 um.
    plate.region = {{rect(0, 0, 20, 10), circle(5, 5, 0.995, 120)}};
    auto fs = run_dfm(jw, plate);
    REQUIRE(fs.size() == 1);
    CHECK(fs[0].severity == Severity::Info);
    CHECK(fs[0].value == Approx(0.010).epsilon(0.05));
    // 1.95 mm: 50 um is too tight; the fix names the diameter to ream to.
    plate.region = {{rect(0, 0, 20, 10), circle(5, 5, 0.975, 120)}};
    fs = run_dfm(jw, plate);
    REQUIRE(fs.size() == 1);
    CHECK(fs[0].severity == Severity::Error);
    CHECK_THAT(fs[0].message, ContainsSubstring("um interference"));
    CHECK_THAT(fs[0].fix, ContainsSubstring("ream the seat to 1.988"));
    // 2.01 mm hole: the jewel falls out (negative interference).
    plate.region = {{rect(0, 0, 20, 10), circle(5, 5, 1.005, 120)}};
    CHECK(run_dfm(jw, plate)[0].severity == Severity::Error);
    // A 0.8 mm hole is not near any jewel size: no finding.
    plate.region = {{rect(0, 0, 20, 10), circle(5, 5, 0.4, 60)}};
    CHECK(run_dfm(jw, plate).empty());
}

TEST_CASE("rule sets load from TOML and users can override the built-in ones") {
    const auto& sets = builtin_rule_sets();
    for (const char* name : {"mill", "lathe", "wheel-cutting", "print", "wire-edm", "silicon-drie", "jewelling"}) {
        CHECK_NOTHROW(find_process(name));
    }
    CHECK(sets.size() >= 7);
    CHECK(find_process("mill").limit("min_wall", 0) == Approx(0.25));
    CHECK(find_process("mill").list("cutters").size() == 6);
    CHECK_THROWS_AS(find_process("laser"), Error);

    const auto mine = rule_sets_from_toml("[process.mill]\ndescription='mine'\nrules=['min_wall']\nmin_wall=0.5\n[process.laser]\nrules=['min_wall']\nmin_wall=0.05\n");
    REQUIRE(mine.size() == 2);
    CHECK(find_process("mill", mine).limit("min_wall", 0) == Approx(0.5));  // yours first
    CHECK_NOTHROW(find_process("laser", mine));
    CHECK_THROWS_AS(rule_sets_from_toml("[process.x]\nrules=['telepathy']\n"), Error);
    CHECK_THROWS_AS(rule_sets_from_toml("nothing=1"), Error);
    CHECK_THROWS_AS(rule_sets_from_toml("[process.x]\nrules=[]\nmin_wall='thin'\n"), Error);
    CHECK_THROWS_AS(load_rule_sets("/nonexistent.toml"), Error);

    // Your stricter wall limit changes a verdict.
    PartGeometry p;
    p.name = "strip";
    p.region = {{rect(0, 0, 10, 0.4)}};
    p.thickness = 0.5;
    CHECK(run_dfm(find_process("mill"), p)[1].severity == Severity::Info);
    CHECK(run_dfm(find_process("mill", mine), p)[0].severity == Severity::Error);
}

// ---- collisions --------------------------------------------------------------------------------------------------------

namespace {

struct TwoGears {
    model::Movement movement;
    kinematics::TrainAnalysis train;
    std::vector<CollisionPart> parts;
};

TwoGears two_gears(double centre_distance) {
    gear::GearSpec a, b;
    a.teeth = 24;
    b.teeth = 12;
    a.module = b.module = 0.5;
    TwoGears t;
    for (int k = 0; k < 2; ++k) {
        model::Component c;
        c.features.name = k == 0 ? "wheel" : "pinion";
        c.features.teeth = k == 0 ? 24 : 12;
        c.features.volume = 1;
        t.movement.components.push_back(c);
        model::Mobile mob;
        mob.name = k == 0 ? "M1" : "M2";
        mob.components = {k};
        mob.center = {k == 0 ? 0.0 : centre_distance, 0.0};
        t.movement.mobiles.push_back(mob);
    }
    t.movement.meshes.push_back({0, 1, centre_distance, 0.5, 1.0});
    t.train.mobiles = {{"M1", true, 1.0, 0.0}, {"M2", true, -2.0, 0.0}};

    auto region_of = [](const gear::GearSpec& s, double cx, double half_pitch_turn) {
        mesh::Region r = gear::wheel_region(s);
        for (auto& ring : r.rings) {
            for (auto& p : ring) {
                const double c = std::cos(half_pitch_turn), sn = std::sin(half_pitch_turn);
                p = {cx + p.x * c - p.y * sn, p.x * sn + p.y * c};
            }
        }
        return r;
    };
    CollisionPart pa{"wheel", region_of(a, 0, 0), 0.0, 0.5, 0};
    CollisionPart pb{"pinion", region_of(b, centre_distance, kPi / 12), 0.0, 0.5, 1};  // a gap faces the wheel's tooth
    t.parts = {pa, pb};
    return t;
}

}  // namespace

TEST_CASE("collision sweep: meshing involute wheels at the right centre distance do not overlap") {
    const double c0 = 0.5 * 0.5 * (24 + 12);
    const TwoGears t = two_gears(c0);
    const CollisionReport r = sweep_collisions(t.movement, t.train, t.parts);
    REQUIRE(r.pairs.size() == 1);
    CHECK(r.pairs[0].meshing);
    CHECK(r.pairs[0].steps == 25);
    CHECK(r.pairs[0].overlapping_steps == 0);
    CHECK(r.overlaps == 0);
    CHECK(r.pairs[0].min_clearance > -0.01);  // zero backlash: flanks touch
}

TEST_CASE("collision sweep: a centre distance that is too short overlaps, a long one clears") {
    const double c0 = 0.5 * 0.5 * (24 + 12);
    const CollisionReport tight = sweep_collisions(two_gears(c0 - 0.1).movement, two_gears(c0 - 0.1).train, two_gears(c0 - 0.1).parts);
    REQUIRE(tight.pairs.size() == 1);
    CHECK(tight.overlaps == 1);
    CHECK(tight.pairs[0].overlapping_steps > 0);
    CHECK(tight.pairs[0].min_clearance < -0.02);

    const TwoGears loose = two_gears(c0 + 0.1);
    const CollisionReport r = sweep_collisions(loose.movement, loose.train, loose.parts);
    CHECK(r.overlaps == 0);
    CHECK(r.close_calls == 0);
    CHECK(r.pairs[0].min_clearance > 0.03);
    // Backlash of 0.1 mm of centre distance is about 0.07 mm normal to the flanks.
    CHECK(r.pairs[0].min_clearance < 0.1);
}

TEST_CASE("collision sweep: arbor in a plate hole, and parts at different heights are ignored") {
    model::Movement m;
    model::Component arbor;
    arbor.features.name = "arbor";
    arbor.features.volume = 1;
    m.components.push_back(arbor);
    model::Mobile mob;
    mob.name = "M1";
    mob.components = {0};
    mob.center = {5, 5};
    m.mobiles.push_back(mob);
    kinematics::TrainAnalysis train;
    train.mobiles = {{"M1", true, 1.0, 0.0}};

    CollisionPart plate{"plate", {{rect(0, 0, 10, 10), circle(5, 5, 0.25, 96)}}, 0.0, 0.5, -1};
    CollisionPart fits{"arbor", {{circle(5, 5, 0.2, 96)}}, -0.2, 0.7, 0};
    CollisionReport r = sweep_collisions(m, train, {plate, fits});
    REQUIRE(r.pairs.size() == 1);
    CHECK_FALSE(r.pairs[0].meshing);
    CHECK(r.pairs[0].steps == 121);
    CHECK(r.pairs[0].min_clearance == Approx(0.05).epsilon(0.03));
    CHECK(r.overlaps == 0);
    CHECK(r.close_calls == 0);

    CollisionPart tight{"arbor", {{circle(5, 5, 0.24, 96)}}, -0.2, 0.7, 0};
    r = sweep_collisions(m, train, {plate, tight});
    CHECK(r.close_calls == 1);  // 10 um clearance is under the 20 um warning limit
    CollisionPart jammed{"arbor", {{circle(5, 5, 0.3, 96)}}, -0.2, 0.7, 0};
    r = sweep_collisions(m, train, {plate, jammed});
    CHECK(r.overlaps == 1);
    CHECK(r.pairs[0].min_clearance < -0.04);

    // An arbor above the plate's height range cannot touch it.
    CollisionPart above{"arbor", {{circle(5, 5, 0.3, 96)}}, 0.6, 1.0, 0};
    CHECK(sweep_collisions(m, train, {plate, above}).pairs_checked == 0);
    // Two static parts are never swept.
    CollisionPart other{"bridge", {{rect(0, 0, 10, 10)}}, 0.0, 0.5, -1};
    CHECK(sweep_collisions(m, train, {plate, other}).pairs_checked == 0);
}

// ---- part drawings ------------------------------------------------------------------------------------------------------------

TEST_CASE("part drawing: holes, datum coordinates, overall size and notes") {
    PartDrawingInput in;
    in.title = "bridge";
    in.thickness = 0.8;
    in.material = "brass_leaded";
    in.notes = {"deburr all edges"};
    in.region = {{rect(2, 3, 14, 9), circle(5, 6, 0.5, 64), circle(11, 6, 0.25, 64), rect(7.5, 5.5, 8.5, 6.5)}};
    const PartDrawing pd = make_part_drawing(in);
    CHECK(pd.width == Approx(12.0));
    CHECK(pd.height == Approx(6.0));
    REQUIRE(pd.holes.size() == 3);
    CHECK(pd.holes[0].round);
    CHECK(pd.holes[0].diameter == Approx(1.0).epsilon(0.01));
    CHECK(pd.holes[0].x == Approx(3.0).epsilon(1e-3));  // from the lower left corner of the bounding box
    CHECK(pd.holes[0].y == Approx(3.0).epsilon(1e-3));
    CHECK(pd.holes[1].diameter == Approx(0.5).epsilon(0.01));
    CHECK_FALSE(pd.holes[2].round);  // the square cut-out is drawn as an outline

    const auto s = report::summarize_dxf(report::to_dxf(pd.drawing));
    CHECK(s.circles == 2);
    CHECK(s.closed_polylines == 2);  // the outline and the square
    CHECK_THAT(pd.info[1], ContainsSubstring("Overall 12.000 x 6.000 x 0.800 mm, material brass_leaded"));
    CHECK_THAT(pd.info[2], ContainsSubstring("Hole 1: D 1.000 at x 3.000, y 3.000"));
    CHECK(pd.info.back() == "deburr all edges");
    const std::string svg = report::to_svg(pd.drawing);
    CHECK_THAT(svg, ContainsSubstring("12.000"));
    CHECK_THAT(svg, ContainsSubstring("D 1.000"));
    CHECK_THROWS_AS(make_part_drawing({}), Error);

    PartDrawingInput wheel = in;
    wheel.teeth = 40;
    wheel.module = 0.15;
    CHECK_THAT(make_part_drawing(wheel).info[2], ContainsSubstring("40 teeth, module 0.1500"));
}
