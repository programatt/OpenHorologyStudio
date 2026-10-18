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

#include "core/error.hpp"

namespace ohs::animation {

namespace {

static_assert(std::endian::native == std::endian::little, "glTF buffers are little-endian");

using nlohmann::json;

struct Vec3 {
    float x, y, z;
};

struct MeshData {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
};

// Flat-shaded triangles (three vertices each) keep the sharp tooth edges sharp.
MeshData tessellate(const TopoDS_Shape& shape, double deflection, double pivot_x, double pivot_y) {
    BRepMesh_IncrementalMesh mesher(shape, deflection, false, 0.5, true);
    MeshData out;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        const gp_Trsf trsf = loc.Transformation();
        const bool reversed = face.Orientation() == TopAbs_REVERSED;
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);
            const gp_Pnt p1 = tri->Node(n1).Transformed(trsf);
            const gp_Pnt p2 = tri->Node(n2).Transformed(trsf);
            const gp_Pnt p3 = tri->Node(n3).Transformed(trsf);
            const gp_Vec normal = gp_Vec(p1, p2).Crossed(gp_Vec(p1, p3));
            if (normal.Magnitude() < 1e-12) continue;
            const gp_Vec n = normal.Normalized();
            for (const gp_Pnt& p : {p1, p2, p3}) {
                out.positions.push_back({static_cast<float>(p.X() - pivot_x), static_cast<float>(p.Y() - pivot_y),
                                         static_cast<float>(p.Z())});
                out.normals.push_back({static_cast<float>(n.X()), static_cast<float>(n.Y()), static_cast<float>(n.Z())});
            }
        }
    }
    return out;
}

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
    if (train.root_period_hours <= 0.0) throw Error("animation needs a beat rate: pass --beat-rate or import with one");
    if (o.duration_seconds <= 0 || o.fps <= 0 || o.time_scale <= 0) throw Error("duration, fps and time scale must be positive");
    const kinematics::PoseOptions pose{o.stepped, 0.2};

    // Keyframe spacing: at least fps, and fine enough that no mobile turns more than the cap
    // between keyframes (and that each escapement impulse is sampled in several keyframes).
    constexpr double two_pi = 2.0 * std::numbers::pi;
    double max_rate = 0.0;  // rad per animation second
    for (const auto& mm : train.mobiles) {
        if (mm.reached) max_rate = std::max(max_rate, two_pi * std::abs(mm.ratio) / (train.root_period_hours * 3600.0) * o.time_scale);
    }
    double dt = 1.0 / o.fps;
    if (max_rate > 0) dt = std::min(dt, o.max_keyframe_rotation / max_rate);
    if (o.stepped) {
        const double beat_seconds_anim = 3600.0 / (train.beats_per_root_rev / train.root_period_hours) / o.time_scale;
        dt = std::min(dt, 0.2 * beat_seconds_anim / 8.0);
    }
    const std::size_t frames = static_cast<std::size_t>(std::floor(o.duration_seconds / dt)) + 1;
    if (frames > o.max_keyframes) {
        throw Error("animation needs " + std::to_string(frames) + " keyframes; lower --time-scale or --duration");
    }

    std::vector<float> times(frames);
    std::vector<std::vector<float>> quats(m.mobiles.size(), std::vector<float>(frames * 4, 0.f));
    double max_step = 0.0;
    std::vector<double> prev;
    for (std::size_t k = 0; k < frames; ++k) {
        times[k] = static_cast<float>(k * dt);
        const auto angles = kinematics::mobile_angles(train, k * dt * o.time_scale, pose);
        for (std::size_t i = 0; i < angles.size(); ++i) {
            quats[i][4 * k + 2] = static_cast<float>(std::sin(angles[i] / 2));
            quats[i][4 * k + 3] = static_cast<float>(std::cos(angles[i] / 2));
            if (k > 0) max_step = std::max(max_step, std::abs(angles[i] - prev[i]));
        }
        prev = angles;
    }

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

    AnimationStats stats;
    stats.keyframes = frames;
    stats.max_degrees_per_keyframe = max_step * 180.0 / std::numbers::pi;
    for (std::size_t c = 0; c < m.components.size(); ++c) {
        const auto& comp = m.components[c];
        const auto found = by_name.find(comp.features.name);
        if (found == by_name.end()) throw Error("no imported part named '" + comp.features.name + "'; was the right STEP file given?");

        const int mobile = m.mobile_of(static_cast<int>(c));
        const double px = mobile >= 0 ? m.mobiles[mobile].center.x : 0.0;
        const double py = mobile >= 0 ? m.mobiles[mobile].center.y : 0.0;
        const TopoDS_Shape shape = geometry::orient_dial_axis_to_z(found->second->shape, dial_axis);
        const MeshData mesh = tessellate(shape, o.linear_deflection, px, py);
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

        nodes.push_back({{"name", comp.features.name}, {"mesh", meshes.size() - 1}, {"translation", {px, py, 0.0}}});
        const int node = static_cast<int>(nodes.size()) - 1;
        nodes[0]["children"].push_back(node);
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
