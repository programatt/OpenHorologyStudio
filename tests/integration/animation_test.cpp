// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <nlohmann/json.hpp>

#include "animation/gltf_animation.hpp"
#include "app/import.hpp"
#include "core/error.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "kinematics/kinematics.hpp"

using namespace ohs;
using Catch::Approx;
using nlohmann::json;

namespace {

struct Glb {
    json doc;
    std::vector<std::uint8_t> bin;
};

Glb read_glb(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
    REQUIRE(bytes.size() > 28);
    auto u32 = [&](std::size_t off) {
        std::uint32_t v;
        std::memcpy(&v, &bytes[off], 4);
        return v;
    };
    REQUIRE(u32(0) == 0x46546C67);
    REQUIRE(u32(4) == 2);
    REQUIRE(u32(8) == bytes.size());
    const std::uint32_t json_len = u32(12);
    REQUIRE(u32(16) == 0x4E4F534A);
    Glb out;
    out.doc = json::parse(bytes.begin() + 20, bytes.begin() + 20 + json_len);
    const std::size_t bin_off = 20 + json_len;
    const std::uint32_t bin_len = u32(bin_off);
    REQUIRE(u32(bin_off + 4) == 0x004E4942);
    REQUIRE(bin_off + 8 + bin_len == bytes.size());
    out.bin.assign(bytes.begin() + bin_off + 8, bytes.end());
    return out;
}

std::vector<float> floats(const Glb& g, int accessor) {
    const auto& a = g.doc["accessors"][accessor];
    const auto& v = g.doc["bufferViews"][a["bufferView"].get<int>()];
    const int comps = a["type"] == "SCALAR" ? 1 : a["type"] == "VEC3" ? 3 : 4;
    const std::size_t n = a["count"].get<std::size_t>() * comps;
    std::vector<float> out(n);
    std::memcpy(out.data(), &g.bin[v["byteOffset"].get<std::size_t>()], n * sizeof(float));
    return out;
}

gp_Trsf placement(const geometry::ExportPart& p) {
    gp_Trsf rot, tr;
    rot.SetRotation(gp_Ax1(gp::Origin(), gp::DZ()), p.rz);
    tr.SetTranslation(gp_Vec(p.dx, p.dy, p.dz));
    return tr.Multiplied(rot);
}

}  // namespace

TEST_CASE("test movement gears interleave: meshing pairs do not overlap, wrongly phased ones do") {
    const auto parts = geometry::make_test_movement();
    auto find = [&parts](const std::string& name) -> const geometry::ExportPart& {
        for (const auto& p : parts) {
            if (p.name == name) return p;
        }
        FAIL("missing " << name);
        return parts.front();
    };
    auto overlap = [](const TopoDS_Shape& a, const TopoDS_Shape& b) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(BRepAlgoAPI_Common(a, b).Shape(), props);
        return std::abs(props.Mass());
    };
    const std::pair<const char*, const char*> meshes[] = {{"barrel", "center_pinion"},
                                                           {"center_wheel", "third_pinion"},
                                                           {"third_wheel", "fourth_pinion"},
                                                           {"fourth_wheel", "escape_pinion"}};
    for (const auto& [a, b] : meshes) {
        const auto& pa = find(a);
        const auto& pb = find(b);
        const TopoDS_Shape sa = BRepBuilderAPI_Transform(pa.shape, placement(pa), true).Shape();
        const TopoDS_Shape sb = BRepBuilderAPI_Transform(pb.shape, placement(pb), true).Shape();
        INFO(a << " / " << b);
        CHECK(overlap(sa, sb) < 1e-4);

        // Half a tooth pitch of extra rotation puts a tooth where a gap should be.
        auto bad = pb;
        bad.rz += std::numbers::pi / (b == std::string("escape_pinion") ? 6 : 10);
        const TopoDS_Shape sbad = BRepBuilderAPI_Transform(bad.shape, placement(bad), true).Shape();
        CHECK(overlap(sa, sbad) > 1e-4);
    }
}

TEST_CASE("animated GLB: structure, rotations and rates are right") {
    const auto dir = std::filesystem::temp_directory_path() / "ohs_anim_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto step = dir / "train.step";
    geometry::export_step(geometry::make_test_movement(), step);

    app::ImportOptions opt;
    opt.rig.beat_rate_vph = 25200;
    const model::Movement m = app::import_movement(step, opt);
    const auto parts = geometry::import_step(step);
    const auto train = kinematics::analyze_train(m, m.beat_rate_vph);

    animation::AnimationOptions ao;
    ao.duration_seconds = 5.0;
    ao.time_scale = 600.0;  // 3000 s of movement time
    const auto stats = animation::write_animated_glb(m, parts, train, geometry::Axis::Z, ao, dir / "smooth.glb");
    CHECK(stats.animated_nodes == 14);  // every part but the plate
    CHECK(stats.triangles > 5000);
    CHECK(stats.max_degrees_per_keyframe <= 90.0 + 1e-6);

    const Glb g = read_glb(dir / "smooth.glb");
    CHECK(g.doc["asset"]["version"] == "2.0");
    const auto& anim = g.doc["animations"][0];
    CHECK(anim["channels"].size() == 14);
    const auto times = floats(g, anim["samplers"][0]["input"].get<int>());
    CHECK(times.front() == 0.0f);
    CHECK(times.back() == Approx(5.0f));

    // Every accessor lies inside its buffer view, and every view inside the buffer.
    for (const auto& v : g.doc["bufferViews"]) {
        CHECK(v["byteOffset"].get<std::size_t>() + v["byteLength"].get<std::size_t>() <= g.bin.size());
    }

    auto final_angle_deg = [&](const std::string& node_name) {
        for (const auto& ch : anim["channels"]) {
            const auto& node = g.doc["nodes"][ch["target"]["node"].get<int>()];
            if (node["name"] != node_name) continue;
            const auto q = floats(g, anim["samplers"][ch["sampler"].get<int>()]["output"].get<int>());
            double unwrapped = 0.0, prev = 0.0;
            for (std::size_t k = 0; k < q.size() / 4; ++k) {
                double a = 2.0 * std::atan2(q[4 * k + 2], q[4 * k + 3]);
                while (a - prev > std::numbers::pi) a -= 2 * std::numbers::pi;
                while (a - prev < -std::numbers::pi) a += 2 * std::numbers::pi;
                unwrapped = prev = a;
            }
            return unwrapped * 180.0 / std::numbers::pi;
        }
        FAIL("no rotation channel for " << node_name);
        return 0.0;
    };
    CHECK(final_angle_deg("barrel") == Approx(37.5).epsilon(1e-4));            // 3000 s of an 8 h turn
    CHECK(final_angle_deg("fourth_wheel") == Approx(-50 * 360.0).epsilon(1e-4));  // 1 rpm for 50 min
    CHECK(final_angle_deg("escape_wheel") == Approx(700 * 360.0).epsilon(1e-4));  // 14 rpm
    CHECK(final_angle_deg("center_pinion") == Approx(final_angle_deg("center_wheel")));  // one mobile

    // Stepped: at real-time-ish scale the escape wheel moves one 12 degree step per beat.
    ao.duration_seconds = 3.0;
    ao.time_scale = 2.0;
    ao.stepped = true;
    animation::write_animated_glb(m, parts, train, geometry::Axis::Z, ao, dir / "stepped.glb");
    const Glb s = read_glb(dir / "stepped.glb");
    const auto& sa = s.doc["animations"][0];
    for (const auto& ch : sa["channels"]) {
        if (s.doc["nodes"][ch["target"]["node"].get<int>()]["name"] != "escape_wheel") continue;
        const auto q = floats(s, sa["samplers"][ch["sampler"].get<int>()]["output"].get<int>());
        const auto st = floats(s, sa["samplers"][ch["sampler"].get<int>()]["input"].get<int>());
        // 6 s of movement time = 42 beats = 504 degrees.
        double a = 2.0 * std::atan2(q[q.size() - 2], q[q.size() - 1]);
        double deg = std::fmod(a * 180 / std::numbers::pi, 360.0);
        if (deg < 0) deg += 360.0;
        CHECK(deg == Approx(144.0).margin(0.1));  // 504 mod 360
        CHECK(st.back() == Approx(3.0f));
    }

    CHECK_THROWS_AS(animation::write_animated_glb(m, parts, kinematics::analyze_train(m, 0.0), geometry::Axis::Z, ao,
                                                  dir / "x.glb"),
                    Error);
    std::filesystem::remove_all(dir);
}

TEST_CASE("animated USD: same motion as the glTF, in a text file any USD tool reads") {
    const auto dir = std::filesystem::temp_directory_path() / "ohs_usd_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto step = dir / "train.step";
    geometry::export_step(geometry::make_test_movement(), step);
    app::ImportOptions opt;
    opt.rig.beat_rate_vph = 25200;
    const model::Movement m = app::import_movement(step, opt);
    const auto parts = geometry::import_step(step);
    const auto train = kinematics::analyze_train(m, m.beat_rate_vph);

    animation::AnimationOptions ao;
    ao.duration_seconds = 5.0;
    ao.time_scale = 600.0;
    const auto stats = animation::write_animated_usd(m, parts, train, geometry::Axis::Z, ao, dir / "train.usda");
    CHECK(stats.animated_nodes == 14);
    CHECK(stats.triangles > 5000);

    std::ifstream in(dir / "train.usda");
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    REQUIRE(text.rfind("#usda 1.0", 0) == 0);
    CHECK(text.find("metersPerUnit = 0.001") != std::string::npos);
    CHECK(text.find("upAxis = \"Z\"") != std::string::npos);

    auto count = [&](const std::string& needle) {
        std::size_t n = 0;
        for (std::size_t pos = 0; (pos = text.find(needle, pos)) != std::string::npos; pos += needle.size()) ++n;
        return n;
    };
    CHECK(count("xformOp:rotateZ.timeSamples") == 14);
    CHECK(count("def Mesh") == m.components.size());

    // Last time sample of a part's rotation, in degrees.
    auto final_angle_deg = [&](const std::string& doc_name) {
        const std::size_t part = text.find("doc = \"" + doc_name + "\"");
        REQUIRE(part != std::string::npos);
        const std::size_t end = text.find("        }\n", text.find("timeSamples", part));
        const std::size_t colon = text.rfind(": ", end);
        return std::stod(text.substr(colon + 2));
    };
    CHECK(final_angle_deg("barrel") == Approx(37.5).epsilon(1e-4));
    CHECK(final_angle_deg("fourth_wheel") == Approx(-50 * 360.0).epsilon(1e-4));
    CHECK(final_angle_deg("escape_wheel") == Approx(700 * 360.0).epsilon(1e-4));

    CHECK_THROWS_AS(animation::write_animated_usd(m, parts, kinematics::analyze_train(m, 0.0), geometry::Axis::Z, ao, dir / "x.usda"), Error);
    std::filesystem::remove_all(dir);
}
