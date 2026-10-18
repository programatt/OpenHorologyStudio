// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <string>

#include "animation/scene.hpp"
#include "core/error.hpp"

namespace ohs::animation {

namespace {

// USD prim names are identifiers: letters, digits and underscores, not starting with a digit.
std::string prim_name(const std::string& name, std::size_t index) {
    std::string out;
    for (char c : name) out.push_back(std::isalnum(static_cast<unsigned char>(c)) ? c : '_');
    if (out.empty() || std::isdigit(static_cast<unsigned char>(out.front()))) out.insert(0, "p_");
    return out + "_" + std::to_string(index);  // the index keeps names that differ only in punctuation apart
}

struct Colour {
    double r, g, b;
};

Colour colour_of(model::Role role) {
    switch (role) {
        case model::Role::Plate:
        case model::Role::Bridge: return {0.72, 0.70, 0.62};
        case model::Role::Arbor:
        case model::Role::PalletFork:
        case model::Role::Screw: return {0.62, 0.64, 0.67};
        default: return {0.79, 0.64, 0.30};
    }
}

}  // namespace

AnimationStats write_animated_usd(const model::Movement& m, const std::vector<geometry::ImportedPart>& parts,
                                  const kinematics::TrainAnalysis& train, geometry::Axis dial_axis,
                                  const AnimationOptions& o, const std::filesystem::path& path) {
    const Keyframes kf = make_keyframes(m, train, o);
    std::map<std::string, const geometry::ImportedPart*> by_name;
    for (const auto& p : parts) by_name[p.name] = &p;

    std::ofstream out(path);
    if (!out) throw Error("cannot write " + path.string());
    out.precision(9);
    out << "#usda 1.0\n(\n    defaultPrim = \"movement\"\n    metersPerUnit = 0.001\n    upAxis = \"Z\"\n"
        << "    startTimeCode = 0\n    endTimeCode = " << kf.times.back() << "\n    timeCodesPerSecond = 1\n"
        << "    doc = \"Open Horology Studio going-train animation\"\n)\n\n"
        << "def Xform \"movement\"\n{\n";

    AnimationStats stats;
    stats.keyframes = kf.times.size();
    stats.max_degrees_per_keyframe = kf.max_step * 180.0 / std::numbers::pi;
    const double to_deg = 180.0 / std::numbers::pi;
    for (std::size_t c = 0; c < m.components.size(); ++c) {
        const auto& comp = m.components[c];
        const auto found = by_name.find(comp.features.name);
        if (found == by_name.end()) throw Error("no imported part named '" + comp.features.name + "'; was the right STEP file given?");

        const int mobile = m.mobile_of(static_cast<int>(c));
        const double px = mobile >= 0 ? m.mobiles[mobile].center.x : 0.0;
        const double py = mobile >= 0 ? m.mobiles[mobile].center.y : 0.0;
        const MeshData mesh = tessellate(geometry::orient_dial_axis_to_z(found->second->shape, dial_axis), o.linear_deflection, px, py);
        if (mesh.positions.empty()) continue;
        stats.triangles += mesh.positions.size() / 3;
        const bool animated = mobile >= 0 && train.mobiles[static_cast<std::size_t>(mobile)].reached;

        out << "    def Xform \"" << prim_name(comp.features.name, c) << "\" (\n        doc = \"" << comp.features.name << "\"\n    )\n    {\n"
            << "        double3 xformOp:translate = (" << px << ", " << py << ", 0)\n";
        if (animated) {
            out << "        double xformOp:rotateZ.timeSamples = {\n";
            for (std::size_t k = 0; k < kf.times.size(); ++k) {
                out << "            " << kf.times[k] << ": " << kf.angles[static_cast<std::size_t>(mobile)][k] * to_deg << ",\n";
            }
            out << "        }\n        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:rotateZ\"]\n";
            ++stats.animated_nodes;
        } else {
            out << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n";
        }
        const Colour col = colour_of(comp.role);
        out << "\n        def Mesh \"mesh\"\n        {\n            uniform bool doubleSided = true\n"
            << "            color3f[] primvars:displayColor = [(" << col.r << ", " << col.g << ", " << col.b << ")]\n"
            << "            int[] faceVertexCounts = [";
        const std::size_t tris = mesh.positions.size() / 3;
        for (std::size_t t = 0; t < tris; ++t) out << (t ? ", 3" : "3");
        out << "]\n            int[] faceVertexIndices = [";
        for (std::size_t i = 0; i < mesh.positions.size(); ++i) out << (i ? ", " : "") << i;
        out << "]\n            point3f[] points = [";
        for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
            const Vec3f& p = mesh.positions[i];
            out << (i ? ", (" : "(") << p.x << ", " << p.y << ", " << p.z << ")";
        }
        out << "]\n            normal3f[] normals = [";
        for (std::size_t i = 0; i < mesh.normals.size(); ++i) {
            const Vec3f& n = mesh.normals[i];
            out << (i ? ", (" : "(") << n.x << ", " << n.y << ", " << n.z << ")";
        }
        out << "] (\n                interpolation = \"faceVarying\"\n            )\n            uniform token subdivisionScheme = \"none\"\n        }\n    }\n";
    }
    out << "}\n";
    if (!out) throw Error("write failed: " + path.string());
    return stats;
}

}  // namespace ohs::animation
