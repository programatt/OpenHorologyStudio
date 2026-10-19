// SPDX-License-Identifier: Apache-2.0
#include "animation/gltf_animation.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <numbers>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <nlohmann/json.hpp>

#include "animation/scene.hpp"
#include "core/error.hpp"

namespace ohs::animation {

namespace {

static_assert(std::endian::native == std::endian::little, "glTF buffers are little-endian");

using nlohmann::json;

using Vec3 = Vec3f;

class BinaryBuffer {
public:
    // Appends data (4-byte aligned) as a new buffer view and returns its index.
    int add(const void* data, std::size_t bytes, json& views) {
        while (blob_.size() % 4) blob_.push_back(0);
        const std::size_t offset = blob_.size();
        const auto* p = static_cast<const std::uint8_t*>(data);
        blob_.insert(blob_.end(), p, p + bytes);
        views.push_back({{"buffer", 0}, {"byteOffset", offset}, {"byteLength", bytes}});
        return static_cast<int>(views.size()) - 1;
    }
    std::vector<std::uint8_t>& bytes() { return blob_; }

private:
    std::vector<std::uint8_t> blob_;
};

constexpr int kFloat = 5126;

int add_accessor(json& accessors, int view, std::size_t count, const char* type, json min = nullptr, json max = nullptr) {
    json a = {{"bufferView", view}, {"componentType", kFloat}, {"count", count}, {"type", type}};
    if (!min.is_null()) a["min"] = std::move(min);
    if (!max.is_null()) a["max"] = std::move(max);
    accessors.push_back(std::move(a));
    return static_cast<int>(accessors.size()) - 1;
}

int material_index(model::Role role) {
    switch (role) {
        case model::Role::Plate:
        case model::Role::Bridge: return 2;
        case model::Role::Arbor:
        case model::Role::PalletFork:
        case model::Role::Stem:
        case model::Role::Spring:
        case model::Role::Lever:
        case model::Role::Screw: return 1;
        default: return 0;
    }
}

void write_glb(const json& doc, const std::vector<std::uint8_t>& bin, const std::filesystem::path& path) {
    std::string text = doc.dump();
    while (text.size() % 4) text.push_back(' ');
    std::vector<std::uint8_t> padded_bin = bin;
    while (padded_bin.size() % 4) padded_bin.push_back(0);

    auto u32 = [](std::uint32_t v) { return v; };
    const std::uint32_t total = 12 + 8 + static_cast<std::uint32_t>(text.size()) + 8 + static_cast<std::uint32_t>(padded_bin.size());
    std::ofstream out(path, std::ios::binary);
    if (!out) throw Error("cannot write " + path.string());
    auto put = [&out](std::uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
    put(u32(0x46546C67));  // "glTF"
    put(2);
    put(total);
    put(static_cast<std::uint32_t>(text.size()));
    put(u32(0x4E4F534A));  // "JSON"
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    put(static_cast<std::uint32_t>(padded_bin.size()));
    put(u32(0x004E4942));  // "BIN\0"
    out.write(reinterpret_cast<const char*>(padded_bin.data()), static_cast<std::streamsize>(padded_bin.size()));
    if (!out) throw Error("write failed: " + path.string());
}

}  // namespace

AnimationStats write_animated_glb(const model::Movement& m, const std::vector<geometry::ImportedPart>& parts,
                                  const kinematics::TrainAnalysis& train, geometry::Axis dial_axis,
                                  const AnimationOptions& o, const std::filesystem::path& path) {
    const Keyframes kf = make_keyframes(m, train, o);
    const std::size_t frames = kf.times.size();
    std::vector<float> times(frames);
    std::vector<std::vector<float>> quats(m.mobiles.size(), std::vector<float>(frames * 4, 0.f));
    for (std::size_t k = 0; k < frames; ++k) {
        times[k] = static_cast<float>(kf.times[k]);
        for (std::size_t i = 0; i < kf.angles.size(); ++i) {
            quats[i][4 * k + 2] = static_cast<float>(std::sin(kf.angles[i][k] / 2));
            quats[i][4 * k + 3] = static_cast<float>(std::cos(kf.angles[i][k] / 2));
        }
    }
    // A stem, lever or spring turns about an axis of its own: a quaternion about that axis.
    std::vector<std::vector<float>> part_quats(train.articulations.size(), std::vector<float>(frames * 4, 0.f));
    for (std::size_t i = 0; i < part_quats.size(); ++i) {
        const auto& axis = train.articulations[i].axis;
        for (std::size_t k = 0; k < frames; ++k) {
            const double half = kf.part_angles[i][k] / 2;
            for (int c = 0; c < 3; ++c) part_quats[i][4 * k + static_cast<std::size_t>(c)] = static_cast<float>(axis[c] * std::sin(half));
            part_quats[i][4 * k + 3] = static_cast<float>(std::cos(half));
        }
    }
    const double max_step = kf.max_step;

    std::map<std::string, const geometry::ImportedPart*> by_name;
    for (const auto& p : parts) by_name[p.name] = &p;

    json doc;
    doc["asset"] = {{"version", "2.0"}, {"generator", "Open Horology Studio"}};
    doc["scene"] = 0;
    doc["scenes"] = json::array({{{"nodes", json::array({0})}}});
    doc["materials"] = json::array({
        {{"name", "brass"}, {"pbrMetallicRoughness", {{"baseColorFactor", {0.79, 0.64, 0.30, 1.0}}, {"metallicFactor", 1.0}, {"roughnessFactor", 0.35}}}},
        {{"name", "steel"}, {"pbrMetallicRoughness", {{"baseColorFactor", {0.62, 0.64, 0.67, 1.0}}, {"metallicFactor", 1.0}, {"roughnessFactor", 0.30}}}},
        {{"name", "plate"}, {"pbrMetallicRoughness", {{"baseColorFactor", {0.72, 0.70, 0.62, 1.0}}, {"metallicFactor", 0.9}, {"roughnessFactor", 0.55}}}},
    });
    json views = json::array(), accessors = json::array(), meshes = json::array(), nodes = json::array();
    BinaryBuffer bin;

    // Node 0: millimetres -> metres and CAD Z-up -> glTF Y-up (-90 degrees about X).
    const double s = std::sqrt(0.5);
    nodes.push_back({{"name", "movement"}, {"scale", {0.001, 0.001, 0.001}}, {"rotation", {-s, 0.0, 0.0, s}}, {"children", json::array()}});

    const int time_view = bin.add(times.data(), times.size() * sizeof(float), views);
    const int time_acc = add_accessor(accessors, time_view, frames, "SCALAR", json::array({times.front()}), json::array({times.back()}));

    json samplers = json::array(), channels = json::array();
    std::vector<int> sampler_of_mobile(m.mobiles.size(), -1);
    for (std::size_t i = 0; i < m.mobiles.size(); ++i) {
        if (!train.mobiles[i].reached) continue;
        const int view = bin.add(quats[i].data(), quats[i].size() * sizeof(float), views);
        const int acc = add_accessor(accessors, view, frames, "VEC4");
        samplers.push_back({{"input", time_acc}, {"output", acc}, {"interpolation", "LINEAR"}});
        sampler_of_mobile[i] = static_cast<int>(samplers.size()) - 1;
    }

    std::vector<int> sampler_of_part(train.articulations.size(), -1);
    for (std::size_t i = 0; i < part_quats.size(); ++i) {
        const int view = bin.add(part_quats[i].data(), part_quats[i].size() * sizeof(float), views);
        const int acc = add_accessor(accessors, view, frames, "VEC4");
        samplers.push_back({{"input", time_acc}, {"output", acc}, {"interpolation", "LINEAR"}});
        sampler_of_part[i] = static_cast<int>(samplers.size()) - 1;
    }

    AnimationStats stats;
    stats.keyframes = frames;
    stats.max_degrees_per_keyframe = max_step * 180.0 / std::numbers::pi;
    for (std::size_t c = 0; c < m.components.size(); ++c) {
        const auto& comp = m.components[c];
        const auto found = by_name.find(comp.features.name);
        if (found == by_name.end()) throw Error("no imported part named '" + comp.features.name + "'; was the right STEP file given?");

        const int mobile = m.mobile_of(static_cast<int>(c));
        double px = mobile >= 0 ? m.mobiles[mobile].center.x : 0.0;
        double py = mobile >= 0 ? m.mobiles[mobile].center.y : 0.0;
        double pz = 0.0;
        int art = -1;
        for (std::size_t i = 0; i < train.articulations.size(); ++i) {
            if (train.articulations[i].component == static_cast<int>(c)) art = static_cast<int>(i);
        }
        if (art >= 0) {
            const auto& pt = train.articulations[static_cast<std::size_t>(art)].point;
            px = pt[0];
            py = pt[1];
            pz = pt[2];
        }
        const TopoDS_Shape shape = geometry::orient_dial_axis_to_z(found->second->shape, dial_axis);
        const MeshData mesh = tessellate(shape, o.linear_deflection, px, py, pz);
        if (mesh.positions.empty()) continue;
        stats.triangles += mesh.positions.size() / 3;

        Vec3 lo = mesh.positions.front(), hi = mesh.positions.front();
        for (const Vec3& p : mesh.positions) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        const int pv = bin.add(mesh.positions.data(), mesh.positions.size() * sizeof(Vec3), views);
        const int nv = bin.add(mesh.normals.data(), mesh.normals.size() * sizeof(Vec3), views);
        const int pa = add_accessor(accessors, pv, mesh.positions.size(), "VEC3", json::array({lo.x, lo.y, lo.z}), json::array({hi.x, hi.y, hi.z}));
        const int na = add_accessor(accessors, nv, mesh.normals.size(), "VEC3");
        meshes.push_back({{"name", comp.features.name},
                          {"primitives", json::array({{{"attributes", {{"POSITION", pa}, {"NORMAL", na}}}, {"material", material_index(comp.role)}}})}});

        nodes.push_back({{"name", comp.features.name}, {"mesh", meshes.size() - 1}, {"translation", {px, py, pz}}});
        const int node = static_cast<int>(nodes.size()) - 1;
        nodes[0]["children"].push_back(node);
        if (art >= 0) {
            channels.push_back({{"sampler", sampler_of_part[static_cast<std::size_t>(art)]}, {"target", {{"node", node}, {"path", "rotation"}}}});
            ++stats.animated_nodes;
        }
        if (mobile >= 0 && sampler_of_mobile[mobile] >= 0) {
            channels.push_back({{"sampler", sampler_of_mobile[mobile]}, {"target", {{"node", node}, {"path", "rotation"}}}});
            ++stats.animated_nodes;
        }
    }

    doc["nodes"] = std::move(nodes);
    doc["meshes"] = std::move(meshes);
    doc["accessors"] = std::move(accessors);
    doc["bufferViews"] = std::move(views);
    doc["buffers"] = json::array({{{"byteLength", bin.bytes().size()}}});
    doc["animations"] = json::array({{{"name", "going train"}, {"samplers", std::move(samplers)}, {"channels", std::move(channels)}}});
    write_glb(doc, bin.bytes(), path);
    return stats;
}

}  // namespace ohs::animation
