// SPDX-License-Identifier: Apache-2.0
#include "gui/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include "core/error.hpp"
#include "geometry/features.hpp"
#include "model/rig_io.hpp"

namespace ohs::gui {

namespace {

std::vector<Triangle> tessellate(const TopoDS_Shape& shape, double deflection, double pivot_x, double pivot_y) {
    BRepMesh_IncrementalMesh mesher(shape, deflection, false, 0.5, true);
    std::vector<Triangle> out;
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
            out.push_back({{p1.X() - pivot_x, p1.Y() - pivot_y, p1.Z()},
                           {p2.X() - pivot_x, p2.Y() - pivot_y, p2.Z()},
                           {p3.X() - pivot_x, p3.Y() - pivot_y, p3.Z()}});
        }
    }
    return out;
}

}  // namespace

Scene Scene::load(const std::filesystem::path& project, const std::filesystem::path& step_override,
                  double beat_rate_vph) {
    Scene s;
    s.movement_ = model::load_rig(project);
    const auto dial_axis = s.movement_.dial_axis == "x" ? geometry::Axis::X
                           : s.movement_.dial_axis == "y" ? geometry::Axis::Y : geometry::Axis::Z;
    const double vph = beat_rate_vph > 0 ? beat_rate_vph : s.movement_.beat_rate_vph;
    try {
        s.train_ = kinematics::analyze_train(s.movement_, vph);
    } catch (const Error& e) {
        s.train_error_ = e.what();  // e.g. no barrel after a role change: still show the parts, just don't animate
    }

    std::filesystem::path step = step_override;
    if (step.empty()) step = project.parent_path() / s.movement_.source_geometry;
    const auto imported = geometry::import_step(step);
    std::map<std::string, const geometry::ImportedPart*> by_name;
    for (const auto& p : imported) by_name[p.name] = &p;

    for (std::size_t c = 0; c < s.movement_.components.size(); ++c) {
        const auto& comp = s.movement_.components[c];
        const auto found = by_name.find(comp.features.name);
        if (found == by_name.end()) throw Error("no imported part named '" + comp.features.name + "'; was the right STEP file given?");
        ScenePart part;
        part.name = comp.features.name;
        part.role = comp.role;
        part.mobile = s.movement_.mobile_of(static_cast<int>(c));
        if (part.mobile >= 0) {
            part.pivot_x = s.movement_.mobiles[part.mobile].center.x;
            part.pivot_y = s.movement_.mobiles[part.mobile].center.y;
        }
        const TopoDS_Shape shape = geometry::orient_dial_axis_to_z(found->second->shape, dial_axis);
        part.triangles = tessellate(shape, 0.02, part.pivot_x, part.pivot_y);
        part.z_min = std::numeric_limits<double>::max();
        part.z_max = std::numeric_limits<double>::lowest();
        for (const auto& t : part.triangles) {
            for (const Point3* p : {&t.a, &t.b, &t.c}) {
                part.z_min = std::min(part.z_min, (*p)[2]);
                part.z_max = std::max(part.z_max, (*p)[2]);
            }
        }
        s.parts_.push_back(std::move(part));
    }
    return s;
}

void Scene::set_visible(const std::string& part, bool visible) {
    const auto it = std::find(hidden_.begin(), hidden_.end(), part);
    if (visible && it != hidden_.end()) hidden_.erase(it);
    if (!visible && it == hidden_.end()) hidden_.push_back(part);
}

bool Scene::visible(const std::string& part) const {
    return std::find(hidden_.begin(), hidden_.end(), part) == hidden_.end();
}

Point3 Scene::centre() const {
    Point3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (const auto& part : parts_) {
        for (const auto& t : part.triangles) {
            for (const Point3* p : {&t.a, &t.b, &t.c}) {
                const double v[3] = {(*p)[0] + part.pivot_x, (*p)[1] + part.pivot_y, (*p)[2]};
                for (int i = 0; i < 3; ++i) {
                    lo[i] = std::min(lo[i], v[i]);
                    hi[i] = std::max(hi[i], v[i]);
                }
            }
        }
    }
    if (lo[0] > hi[0]) return {0, 0, 0};
    return {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
}

double Scene::radius() const {
    const Point3 c = centre();
    double r = 1.0;
    for (const auto& part : parts_) {
        for (const auto& t : part.triangles) {
            for (const Point3* p : {&t.a, &t.b, &t.c}) {
                const double dx = (*p)[0] + part.pivot_x - c[0], dy = (*p)[1] + part.pivot_y - c[1], dz = (*p)[2] - c[2];
                r = std::max(r, std::sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
    return r;
}

Scene::Drawn Scene::draw_list() const {
    std::vector<double> angles(movement_.mobiles.size(), 0.0);
    if (can_animate()) angles = kinematics::mobile_angles(train_, time_, {stepped_, 0.2});
    const double zc = centre()[2];

    Drawn out;
    for (std::size_t i = 0; i < parts_.size(); ++i) {
        const ScenePart& part = parts_[i];
        if (!visible(part.name)) continue;
        const double angle = part.mobile >= 0 && part.mobile < static_cast<int>(angles.size()) ? angles[part.mobile] : 0.0;
        const double ca = std::cos(angle), sa = std::sin(angle);
        const double lift = explode_ * ((part.z_min + part.z_max) / 2 - zc) * 2.0;
        auto place = [&](const Point3& p) -> Point3 {
            return {part.pivot_x + ca * p[0] - sa * p[1], part.pivot_y + sa * p[0] + ca * p[1], p[2] + lift};
        };
        for (const Triangle& t : part.triangles) {
            Triangle w{place(t.a), place(t.b), place(t.c)};
            if (section_.enabled) {
                const auto& n = section_.normal;
                const double cx = (w.a[0] + w.b[0] + w.c[0]) / 3, cy = (w.a[1] + w.b[1] + w.c[1]) / 3,
                             cz = (w.a[2] + w.b[2] + w.c[2]) / 3;
                if (n[0] * cx + n[1] * cy + n[2] * cz > section_.offset) continue;
            }
            out.triangles.push_back(w);
            out.part_index.push_back(static_cast<int>(i));
        }
    }
    return out;
}

}  // namespace ohs::gui
