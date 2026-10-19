// SPDX-License-Identifier: Apache-2.0
#include "gui/field.hpp"

#include <algorithm>
#include <cmath>

#include "core/error.hpp"
#include "fea/results.hpp"
#include "mesh/tetmesh.hpp"

namespace ohs::gui {

namespace {

const fea::NodeField* find(const fea::VtuData& data, const std::string& name) {
    for (const auto& f : data.fields) {
        if (f.name == name) return &f;
    }
    return nullptr;
}

}  // namespace

std::vector<std::string> plottable_fields(const std::filesystem::path& vtu) {
    std::vector<std::string> out;
    const fea::VtuData data = fea::read_vtu(vtu);
    if (find(data, "von_mises")) out.push_back("von_mises");
    if (const auto* d = find(data, "displacement"); d && d->components == 3) out.push_back("displacement");
    return out;
}

FieldOverlay field_overlay_from_vtu(const std::filesystem::path& vtu, const std::string& part, const std::string& field, double pivot_x,
                                    double pivot_y) {
    const fea::VtuData data = fea::read_vtu(vtu);
    std::vector<double> node_value(data.mesh.nodes.size());
    FieldOverlay out;
    out.part = part;
    if (field == "von_mises") {
        const auto* f = find(data, "von_mises");
        if (!f || f->components != 1) throw Error("the result has no von Mises stress");
        node_value = f->values;
        out.label = "von Mises stress (MPa)";
    } else if (field == "displacement") {
        const auto* f = find(data, "displacement");
        if (!f || f->components != 3) throw Error("the result has no displacement");
        for (std::size_t n = 0; n < node_value.size(); ++n) {
            node_value[n] = std::hypot(std::hypot(f->values[3 * n], f->values[3 * n + 1]), f->values[3 * n + 2]);
        }
        out.label = "displacement (mm)";
    } else {
        throw Error("unknown field '" + field + "' (von_mises or displacement)");
    }
    out.lo = *std::min_element(node_value.begin(), node_value.end());
    out.hi = *std::max_element(node_value.begin(), node_value.end());
    if (out.hi <= out.lo) out.hi = out.lo + 1e-12;

    for (const mesh::BoundaryFace& face : mesh::boundary_faces(data.mesh)) {
        const auto point = [&](int k) {
            const auto& p = data.mesh.nodes[static_cast<std::size_t>(face.nodes[static_cast<std::size_t>(k)])];
            return Point3{p.x - pivot_x, p.y - pivot_y, p.z};
        };
        out.triangles.push_back({point(0), point(1), point(2)});
        out.values.push_back({node_value[static_cast<std::size_t>(face.nodes[0])], node_value[static_cast<std::size_t>(face.nodes[1])],
                              node_value[static_cast<std::size_t>(face.nodes[2])]});
    }
    if (out.triangles.empty()) throw Error("the result file has no surface");
    return out;
}

std::array<int, 3> field_colour(double t) {
    // Five stops of viridis.
    static const double stops[5][3] = {{68, 1, 84}, {59, 82, 139}, {33, 145, 140}, {94, 201, 98}, {253, 231, 37}};
    t = std::clamp(t, 0.0, 1.0) * 4.0;
    const int i = std::min(3, static_cast<int>(t));
    const double f = t - i;
    std::array<int, 3> c{};
    for (int k = 0; k < 3; ++k) c[static_cast<std::size_t>(k)] = static_cast<int>(std::lround(stops[i][k] + f * (stops[i + 1][k] - stops[i][k])));
    return c;
}

}  // namespace ohs::gui
