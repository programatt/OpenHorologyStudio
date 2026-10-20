// SPDX-License-Identifier: Apache-2.0
// The pieces added to make import and project opening fast: the shape-grouped parallel loop, the geometry cache, the
// import's progress and cancel hooks, and the fixes found on the way (an out-of-range pressure angle).
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "app/import.hpp"
#include "core/error.hpp"
#include "geometry/features.hpp"
#include "geometry/part_cache.hpp"
#include "geometry/procedural.hpp"
#include "geometry/shape_groups.hpp"
#include "geometry/step_io.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
namespace fs = std::filesystem;

namespace {

fs::path fresh(const char* tag) {
    const fs::path dir = fs::temp_directory_path() / (std::string("ohs-perf-test-") + tag);
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

}  // namespace

TEST_CASE("instances of one shape never run at the same time, and results come out in their own slots") {
    // Twelve placements of three shapes: the instances of a shape share its TShape, as a movement's screws do.
    std::vector<TopoDS_Shape> shapes;
    for (int s = 0; s < 3; ++s) {
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(1.0 + s, 1.0, 1.0).Shape();
        for (int k = 0; k < 4; ++k) {
            gp_Trsf t;
            t.SetTranslation(gp_Vec(10.0 * k, 0, 0));
            shapes.push_back(box.Moved(TopLoc_Location(t)));
        }
    }
    REQUIRE(shapes[0].TShape() == shapes[3].TShape());
    REQUIRE(shapes[0].TShape() != shapes[4].TShape());

    std::mutex m;
    std::map<const void*, int> active;  // instances of each shape running now
    std::atomic<bool> overlapped{false};
    std::vector<int> slot(shapes.size(), -1);
    geometry::parallel_for_shapes(shapes, [&](std::size_t i) {
        const void* key = shapes[i].TShape().get();
        {
            std::lock_guard<std::mutex> lock(m);
            if (++active[key] > 1) overlapped = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        {
            std::lock_guard<std::mutex> lock(m);
            --active[key];
        }
        slot[i] = static_cast<int>(i);
    });
    CHECK_FALSE(overlapped);
    for (std::size_t i = 0; i < slot.size(); ++i) CHECK(slot[i] == static_cast<int>(i));

    // The group form hands over the instances of a shape together, in order.
    std::mutex g;
    std::vector<std::vector<std::size_t>> seen;
    geometry::parallel_for_shape_groups(shapes, [&](const std::vector<std::size_t>& group) {
        std::lock_guard<std::mutex> lock(g);
        seen.push_back(group);
    });
    REQUIRE(seen.size() == 3);
    for (const auto& group : seen) {
        REQUIRE(group.size() == 4);
        for (std::size_t k = 1; k < group.size(); ++k) CHECK(group[k] == group[k - 1] + 1);
    }
}

TEST_CASE("the geometry cache gives the parts back, and goes stale with the CAD file") {
    const fs::path dir = fresh("cache");
    const fs::path step = dir / "movement.step", project = dir / "movement.hsproj";
    geometry::export_step(geometry::make_test_movement(), step);
    const auto parts = geometry::import_step(step);

    CHECK(geometry::cache_dir(project) == dir / "movement.hsproj.ohscache");
    // A job's copy of the project (`movement.job12.hsproj`) shares the project's cache.
    CHECK(geometry::cache_dir(dir / "movement.job12.hsproj") == geometry::cache_dir(project));
    CHECK(geometry::cache_dir(dir / "my.job.hsproj") != geometry::cache_dir(project));

    CHECK_FALSE(fs::exists(geometry::cache_dir(project)));
    const auto first = geometry::load_parts(step, project);  // no cache yet: reads the CAD file and writes it
    CHECK(fs::exists(geometry::cache_dir(project) / "parts.brep"));
    CHECK(fs::exists(geometry::cache_dir(project) / "manifest.txt"));
    REQUIRE(first.size() == parts.size());

    // With the cache the CAD file is not needed: the parts come back, in order, with their names and volumes.
    const auto cached = geometry::load_parts(step, project);
    REQUIRE(cached.size() == parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        CHECK(cached[i].name == parts[i].name);
        CHECK(geometry::extract_features(cached[i], {}).volume == Catch::Approx(geometry::extract_features(parts[i], {}).volume).epsilon(1e-9));
    }

    // A changed CAD file is not served from the cache, and an explicit file (update_cache false) does not replace it.
    const std::string fingerprint = geometry::cad_fingerprint(step);
    REQUIRE_FALSE(fingerprint.empty());
    std::ofstream(step, std::ios::binary | std::ios::app) << "\n/* edited */\n";
    CHECK(geometry::cad_fingerprint(step) != fingerprint);
    const auto stale_manifest = [&] {
        std::ifstream in(geometry::cache_dir(project) / "manifest.txt");
        return std::string(std::istreambuf_iterator<char>(in), {});
    };
    const std::string before = stale_manifest();
    CHECK(geometry::load_parts(step, project, false).size() == parts.size());
    CHECK(stale_manifest() == before);  // read from the file, cache untouched
    CHECK(geometry::load_parts(step, project, true).size() == parts.size());
    CHECK(stale_manifest() != before);  // the cache now describes the edited file

    // Damage is a miss, not an error.
    {
        std::ofstream(geometry::cache_dir(project) / "parts.brep", std::ios::binary | std::ios::trunc) << "not a brep";
    }
    CHECK(geometry::load_parts(step, project).size() == parts.size());
    // And so is a project with nothing beside it, or no project at all.
    CHECK(geometry::load_parts(step, dir / "other" / "x.hsproj", false).size() == parts.size());
    CHECK(geometry::load_parts(step, {}).size() == parts.size());
    CHECK(geometry::is_project_source(step, project, "movement.step"));
    CHECK_FALSE(geometry::is_project_source(step, project, "another.step"));
    fs::remove_all(dir);
}

TEST_CASE("sections are cut once and read back from the cache while the CAD file is unchanged") {
    const fs::path dir = fresh("sections");
    const fs::path step = dir / "movement.step", project = dir / "movement.hsproj";
    geometry::export_step(geometry::make_test_movement(), step);
    const auto parts = geometry::import_step(step);
    const double z = 0.5 * (geometry::extract_features(parts[2], {}).z_min + geometry::extract_features(parts[2], {}).z_max);

    mesh::Region cut;
    {
        geometry::SectionCache cache(project, step);
        cut = cache.get(parts[2].name, '2', parts[2].shape, z, 0.002, 0.002);
        REQUIRE_FALSE(cut.rings.empty());
    }  // saved on destruction
    CHECK(fs::exists(geometry::cache_dir(project) / "sections.bin"));

    // Read back: given a shape that would throw if it were cut (an empty one), the cached section still comes out equal.
    geometry::SectionCache again(project, step);
    const mesh::Region cached = again.get(parts[2].name, '2', TopoDS_Shape{}, z, 0.002, 0.002);
    REQUIRE(cached.rings.size() == cut.rings.size());
    for (std::size_t r = 0; r < cut.rings.size(); ++r) {
        REQUIRE(cached.rings[r].size() == cut.rings[r].size());
        for (std::size_t i = 0; i < cut.rings[r].size(); ++i) {
            CHECK(cached.rings[r][i].x == cut.rings[r][i].x);
            CHECK(cached.rings[r][i].y == cut.rings[r][i].y);
        }
    }
    // Another tolerance is another section; a changed CAD file empties the cache.
    CHECK_THROWS(again.get(parts[2].name, '2', TopoDS_Shape{}, z, 0.001, 0.002));
    again.save();
    std::ofstream(step, std::ios::binary | std::ios::app) << "\n/* edited */\n";
    geometry::SectionCache stale(project, step);
    CHECK_THROWS(stale.get(parts[2].name, '2', TopoDS_Shape{}, z, 0.002, 0.002));
    fs::remove_all(dir);
}

TEST_CASE("one part is read on its own from the cache, and a resident process keeps the parts it has read") {
    const fs::path dir = fresh("onepart");
    const fs::path step = dir / "movement.step", project = dir / "movement.hsproj";
    geometry::export_step(geometry::make_test_movement(), step);
    const auto parts = geometry::load_parts(step, project);  // writes the cache, with a file per part
    REQUIRE(parts.size() > 3);
    CHECK(fs::exists(geometry::cache_dir(project) / "parts" / "2.brep"));

    const std::string name = parts[2].name;
    const geometry::ImportedPart one = geometry::load_part(step, project, name);
    CHECK(one.name == name);
    CHECK(geometry::extract_features(one, {}).volume == Catch::Approx(geometry::extract_features(parts[2], {}).volume).epsilon(1e-9));
    CHECK_THROWS_AS(geometry::load_part(step, project, "no_such_part"), Error);
    // Without a cache (or with a damaged part file) it still finds the part, from the whole read.
    fs::remove(geometry::cache_dir(project) / "parts" / "2.brep");
    CHECK(geometry::load_part(step, project, name).name == name);
    fs::remove_all(geometry::cache_dir(project));
    CHECK(geometry::load_part(step, project, name, false).name == name);

    // The memo: the second read of the same file does not touch the file or the cache.
    geometry::set_parts_memo(true);
    const auto first = geometry::load_parts(step, project);
    fs::remove_all(geometry::cache_dir(project));
    const auto second = geometry::load_parts(step, project);
    REQUIRE(second.size() == first.size());
    CHECK(second[0].shape.TShape() == first[0].shape.TShape());  // the same shapes, shared
    geometry::set_parts_memo(false);
    fs::remove_all(dir);
}

TEST_CASE("an atomic write leaves the old file or the new one") {
    const fs::path dir = fresh("atomic");
    const fs::path file = dir / "project.hsproj";
    REQUIRE(geometry::write_file_atomic(file, "first"));
    REQUIRE(geometry::write_file_atomic(file, "second"));
    std::ifstream in(file);
    std::string text((std::istreambuf_iterator<char>(in)), {});
    CHECK(text == "second");
    CHECK_FALSE(fs::exists(dir / "project.hsproj.tmp"));
    CHECK_FALSE(geometry::write_file_atomic(dir / "no" / "such" / "dir" / "f", "x"));

    // save_rig goes through a temporary file as well.
    const auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    model::save_rig(m, file);
    CHECK_FALSE(fs::exists(dir / "project.hsproj.tmp"));
    CHECK(model::load_rig(file).components.size() == m.components.size());
    fs::remove_all(dir);
}

TEST_CASE("an import reports its progress, stops when asked, and writes the cache") {
    const fs::path dir = fresh("import");
    const fs::path step = dir / "movement.step", project = dir / "movement.hsproj";
    geometry::export_step(geometry::make_test_movement(), step);

    std::atomic<std::size_t> last{0}, total{0};
    app::ImportOptions options;
    options.cache_project = project;
    options.progress = [&](std::size_t done, std::size_t n) {
        last = std::max<std::size_t>(last, done);
        total = n;
    };
    const model::Movement m = app::import_movement(step, options);
    CHECK(total == m.components.size());
    CHECK(last == total);
    CHECK(fs::exists(geometry::cache_dir(project) / "parts.brep"));

    // The same parts however many threads extracted them: the result is deterministic and in part order.
    const model::Movement again = app::import_movement(step, {});
    REQUIRE(again.components.size() == m.components.size());
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        CHECK(again.components[i].features.name == m.components[i].features.name);
        CHECK(again.components[i].features.volume == m.components[i].features.volume);
        CHECK(again.components[i].features.teeth == m.components[i].features.teeth);
        CHECK(again.components[i].role == m.components[i].role);
    }

    std::atomic<bool> stop{true};
    app::ImportOptions cancelled;
    cancelled.cancel = &stop;
    try {
        app::import_movement(step, cancelled);
        FAIL("the cancelled import finished");
    } catch (const Error& e) {
        CHECK(std::string(e.what()).find("cancelled") != std::string::npos);
    }
    fs::remove_all(dir);
}

TEST_CASE("another placement of a shape gets the first one's features, moved; a turned one is extracted in full") {
    const auto parts = geometry::make_test_movement();
    // The arbor placed three ways: the same shape moved by a translation, and turned about the dial axis.
    const TopoDS_Shape base = parts[0].shape;
    auto placed = [&](double dx, double dy, double dz, double turn) {
        gp_Trsf t;
        t.SetRotation(gp_Ax1(gp::Origin(), gp::DZ()), turn);
        gp_Trsf m;
        m.SetTranslation(gp_Vec(dx, dy, dz));
        return geometry::ImportedPart{"copy", base.Moved(TopLoc_Location(m * t))};
    };
    const geometry::ImportedPart first{"first", base.Moved(TopLoc_Location())};
    const geometry::ImportedPart moved = placed(7.0, -3.0, 0.5, 0.0), turned = placed(7.0, -3.0, 0.5, 0.7);
    geometry::FeatureOptions options;
    const model::PartFeatures f = geometry::extract_features(first, options);
    model::PartFeatures out;
    REQUIRE(geometry::translate_features(first, f, moved, options, out));
    const model::PartFeatures full = geometry::extract_features(moved, options);
    CHECK(out.name == "copy");
    CHECK(out.center.x == Catch::Approx(full.center.x).margin(1e-6));
    CHECK(out.center.y == Catch::Approx(full.center.y).margin(1e-6));
    CHECK(out.z_min == Catch::Approx(full.z_min).margin(1e-6));
    CHECK(out.z_max == Catch::Approx(full.z_max).margin(1e-6));
    CHECK(out.teeth == full.teeth);
    CHECK(out.volume == Catch::Approx(full.volume).epsilon(1e-9));
    CHECK(out.fingerprint == full.fingerprint);
    REQUIRE(out.holes.size() == full.holes.size());
    for (std::size_t i = 0; i < out.holes.size(); ++i) {
        CHECK(out.holes[i].x == Catch::Approx(full.holes[i].x).margin(1e-6));
        CHECK(out.holes[i].y == Catch::Approx(full.holes[i].y).margin(1e-6));
    }
    CHECK_FALSE(geometry::translate_features(first, f, turned, options, out));  // turned: not a translation
    CHECK_FALSE(geometry::translate_features(first, f, geometry::ImportedPart{parts[1].name, parts[1].shape}, options, out));  // another shape
}

TEST_CASE("holes are found once per part: a plate keeps the holes it was made with") {
    const auto parts = geometry::make_test_movement();
    int with_holes = 0;
    for (const auto& part : parts) {
        const auto f = geometry::extract_features({part.name, part.shape}, {});
        // A wheel's bore is a hole; a hole is never wider than the part.
        for (const auto& h : f.holes) CHECK(h.radius < f.outer_radius + 1e-9);
        with_holes += f.holes.empty() ? 0 : 1;
    }
    CHECK(with_holes > 0);
}

TEST_CASE("a flank fit with a pressure angle of 45 degrees or more is not kept as a gear fit") {
    auto features = test::synthetic_train();
    for (auto& f : features) {
        if (f.name == "center_wheel") {
            f.flank_rms = 1e-4;
            f.flank_base_radius = 0.05 * f.outer_radius;  // base circle far inside the pitch circle: an angle near 90
        }
        if (f.name == "third_wheel") {
            f.flank_rms = 1e-4;
            f.flank_base_radius = 0.0;
        }
    }
    auto m = rigging::auto_rig("train", features, {});
    const auto& wheel = m.components[static_cast<std::size_t>(m.find_component("center_wheel"))];
    REQUIRE(wheel.gear.has_value());
    CHECK(wheel.gear->pressure_angle_deg == 0.0);
    CHECK(wheel.gear->profile != "involute");
    CHECK_FALSE(model::has_pressure_angle(*wheel.gear));

    // The reading side: a stored angle outside the range is not used, one inside is.
    model::GearFit fit;
    fit.pressure_angle_deg = 57.0;
    CHECK_FALSE(model::has_pressure_angle(fit));
    fit.pressure_angle_deg = 20.0;
    CHECK(model::has_pressure_angle(fit));
    fit.pressure_angle_deg = 45.7;
    CHECK_FALSE(model::has_pressure_angle(fit));
}

TEST_CASE("several roles are set in one edit, all or none") {
    auto m = rigging::auto_rig("train", test::synthetic_train(), {});
    const auto role = [&](const char* name) { return m.components[static_cast<std::size_t>(m.find_component(name))].role; };
    const auto before_center = role("center_wheel");
    REQUIRE(before_center != model::Role::Screw);
    CHECK_THROWS(rigging::set_roles(m, {{"center_wheel", model::Role::Screw}, {"no_such_part", model::Role::Screw}}, {}));
    CHECK(role("center_wheel") == before_center);  // the bad name changed nothing
    rigging::set_roles(m, {{"center_wheel", model::Role::Screw}, {"third_wheel", model::Role::Plate}}, {});
    CHECK(role("center_wheel") == model::Role::Screw);
    CHECK(role("third_wheel") == model::Role::Plate);
    CHECK(m.components[static_cast<std::size_t>(m.find_component("center_wheel"))].role_source == "user");

    // The component to mobile index agrees with the lookup it replaces in loops.
    const auto index = m.mobile_index();
    REQUIRE(index.size() == m.components.size());
    for (std::size_t i = 0; i < index.size(); ++i) CHECK(index[i] == m.mobile_of(static_cast<int>(i)));
}
