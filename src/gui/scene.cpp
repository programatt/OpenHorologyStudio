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
#include "dynamics/calibre.hpp"
#include "dynamics/torque_chain.hpp"
#include "geometry/cad_io.hpp"
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
    const auto imported = geometry::import_cad(step);
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
    s.update_bounds();
    return s;
}

bool Scene::reload_rig(const std::filesystem::path& project, double beat_rate_vph) {
    model::Movement m = model::load_rig(project);
    if (m.dial_axis != movement_.dial_axis || m.components.size() != parts_.size()) return false;
    for (std::size_t c = 0; c < m.components.size(); ++c) {
        if (m.components[c].features.name != parts_[c].name) return false;
    }
    movement_ = std::move(m);
    const double vph = beat_rate_vph > 0 ? beat_rate_vph : movement_.beat_rate_vph;
    train_ = {};
    train_error_.clear();
    try {
        train_ = kinematics::analyze_train(movement_, vph);
    } catch (const Error& e) {
        train_error_ = e.what();
    }
    // The triangles are stored relative to the part's pivot, so a part whose mobile moved is shifted, not re-meshed.
    for (std::size_t c = 0; c < parts_.size(); ++c) {
        ScenePart& part = parts_[c];
        const int mobile = movement_.mobile_of(static_cast<int>(c));
        const double px = mobile >= 0 ? movement_.mobiles[static_cast<std::size_t>(mobile)].center.x : 0.0;
        const double py = mobile >= 0 ? movement_.mobiles[static_cast<std::size_t>(mobile)].center.y : 0.0;
        const double dx = part.pivot_x - px, dy = part.pivot_y - py;
        if (dx != 0.0 || dy != 0.0) {
            for (auto& t : part.triangles) {
                for (Point3* p : {&t.a, &t.b, &t.c}) {
                    (*p)[0] += dx;
                    (*p)[1] += dy;
                }
            }
        }
        part.role = movement_.components[c].role;
        part.mobile = mobile;
        part.pivot_x = px;
        part.pivot_y = py;
    }
    update_bounds();
    forces_.clear();
    if (has_field_) clear_field();
    return true;
}

void Scene::set_visible(const std::string& part, bool visible) {
    if (visible) hidden_.erase(part);
    else hidden_.insert(part);
}

bool Scene::visible(const std::string& part) const {
    return hidden_.empty() || !hidden_.contains(part);
}

void Scene::update_bounds() {
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
    centre_ = lo[0] > hi[0] ? Point3{0, 0, 0} : Point3{(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    radius_ = 1.0;
    for (const auto& part : parts_) {
        for (const auto& t : part.triangles) {
            for (const Point3* p : {&t.a, &t.b, &t.c}) {
                const double dx = (*p)[0] + part.pivot_x - centre_[0], dy = (*p)[1] + part.pivot_y - centre_[1],
                             dz = (*p)[2] - centre_[2];
                radius_ = std::max(radius_, std::sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

void Scene::compute_forces(double wind) {
    forces_.clear();
    const double vph = movement_.beat_rate_vph > 0 ? movement_.beat_rate_vph : 25200.0;
    const dynamics::Calibre calibre = dynamics::default_calibre(vph);
    const dynamics::TorqueChain chain = dynamics::build_chain(movement_, train_, calibre);
    std::map<std::string, double> torque;  // N.mm at each mobile of the path
    for (const auto& row : dynamics::torque_table(chain, calibre, wind)) torque[row.mobile] = row.torque_nmm;

    for (const model::MeshJoint& mesh : movement_.meshes) {
        const int ma = movement_.mobile_of(mesh.a), mb = movement_.mobile_of(mesh.b);
        if (ma < 0 || mb < 0) continue;
        const auto ta = torque.find(movement_.mobiles[static_cast<std::size_t>(ma)].name);
        const auto tb = torque.find(movement_.mobiles[static_cast<std::size_t>(mb)].name);
        if (ta == torque.end() || tb == torque.end()) continue;  // a mesh off the path from the barrel to the escape wheel
        // Torque falls along the train, so the mobile with the smaller torque is the one being driven.
        const bool a_driven = ta->second < tb->second;
        const int driver_comp = a_driven ? mesh.b : mesh.a, driven_comp = a_driven ? mesh.a : mesh.b;
        const int driver_mobile = a_driven ? mb : ma, driven_mobile = a_driven ? ma : mb;
        const auto& driver = movement_.components[static_cast<std::size_t>(driver_comp)];
        const auto& driven = movement_.components[static_cast<std::size_t>(driven_comp)];
        const double r_driver = 0.5 * mesh.module * driver.features.teeth, r_driven = 0.5 * mesh.module * driven.features.teeth;
        if (r_driver <= 0 || r_driven <= 0) continue;
        const auto& c1 = movement_.mobiles[static_cast<std::size_t>(driver_mobile)].center;
        const auto& c2 = movement_.mobiles[static_cast<std::size_t>(driven_mobile)].center;
        const double dx = c2.x - c1.x, dy = c2.y - c1.y, dist = std::hypot(dx, dy);
        if (dist <= 0) continue;
        const double t = r_driver / (r_driver + r_driven);  // pitch point along the line of centres
        const double sense = train_.mobiles[static_cast<std::size_t>(driver_mobile)].ratio >= 0 ? 1.0 : -1.0;
        // Surface velocity of the driver's rim at the pitch point: omega x r, with omega along +Z (or -Z).
        const double vx = -sense * dy / dist, vy = sense * dx / dist;
        ForceArrow arrow;
        arrow.driver = movement_.mobiles[static_cast<std::size_t>(driver_mobile)].name;
        arrow.driven = movement_.mobiles[static_cast<std::size_t>(driven_mobile)].name;
        arrow.at = {c1.x + t * dx, c1.y + t * dy, 0.5 * (driver.features.z_min + driver.features.z_max)};
        arrow.direction = {vx, vy, 0.0};
        arrow.newtons = (a_driven ? ta->second : tb->second) / r_driven;
        forces_.push_back(std::move(arrow));
    }
}

std::vector<Scene::Trail> Scene::trails(int samples) const {
    std::vector<Trail> out;
    if (!can_animate() || samples < 2 || trail_seconds_ <= 0) return out;
    const kinematics::PoseOptions pose{stepped_, 0.2};
    for (std::size_t m = 0; m < movement_.mobiles.size(); ++m) {
        if (!train_.mobiles[m].reached) continue;
        // The marker sits where a tooth tip would be, at angle zero of the mobile's own frame.
        double radius = 0.0, z = 0.0;
        for (int c : movement_.mobiles[m].components) {
            const auto& f = movement_.components[static_cast<std::size_t>(c)].features;
            if (f.outer_radius > radius) {
                radius = f.outer_radius;
                z = 0.5 * (f.z_min + f.z_max);
            }
        }
        if (radius <= 0) continue;
        const auto& centre = movement_.mobiles[m].center;
        Trail trail;
        trail.mobile = movement_.mobiles[m].name;
        // The window is the shorter of the requested time and one revolution of this mobile.
        const double omega = 2.0 * 3.14159265358979323846 * std::abs(train_.mobiles[m].ratio) / (train_.root_period_hours * 3600.0);
        const double window = omega > 0 ? std::min(trail_seconds_, 2.0 * 3.14159265358979323846 / omega) : trail_seconds_;
        for (int i = samples - 1; i >= 0; --i) {
            const double t = std::max(0.0, time_ - window * i / (samples - 1));
            const double a = kinematics::mobile_angles(train_, t, pose)[m];
            trail.points.push_back({centre.x + radius * std::cos(a), centre.y + radius * std::sin(a), z});
        }
        if (trail.points.size() >= 2) out.push_back(std::move(trail));
    }
    return out;
}

Scene::Drawn Scene::draw_list() const {
    std::vector<double> angles(movement_.mobiles.size(), 0.0);
    if (can_animate()) angles = kinematics::mobile_angles(train_, time_, {stepped_, 0.2});
    const double zc = centre_[2];

    Drawn out;
    std::size_t most = has_field_ ? field_.triangles.size() : 0;  // built every frame: grow the lists once
    for (const ScenePart& part : parts_) most += part.triangles.size();
    out.triangles.reserve(most);
    out.part_index.reserve(most);
    out.values.reserve(most);
    for (std::size_t i = 0; i < parts_.size(); ++i) {
        const ScenePart& part = parts_[i];
        if (!visible(part.name)) continue;
        const double angle = part.mobile >= 0 && part.mobile < static_cast<int>(angles.size()) ? angles[part.mobile] : 0.0;
        const double ca = std::cos(angle), sa = std::sin(angle);
        const double lift = explode_ * ((part.z_min + part.z_max) / 2 - zc) * 2.0;
        auto place = [&](const Point3& p) -> Point3 {
            return {part.pivot_x + ca * p[0] - sa * p[1], part.pivot_y + sa * p[0] + ca * p[1], p[2] + lift};
        };
        const bool on_field = has_field_ && field_.part == part.name;
        const std::vector<Triangle>& tris = on_field ? field_.triangles : part.triangles;
        for (std::size_t k = 0; k < tris.size(); ++k) {
            const Triangle& t = tris[k];
            Triangle w{place(t.a), place(t.b), place(t.c)};
            if (section_.enabled) {
                const auto& n = section_.normal;
                const double cx = (w.a[0] + w.b[0] + w.c[0]) / 3, cy = (w.a[1] + w.b[1] + w.c[1]) / 3,
                             cz = (w.a[2] + w.b[2] + w.c[2]) / 3;
                if (n[0] * cx + n[1] * cy + n[2] * cz > section_.offset) continue;
            }
            out.triangles.push_back(w);
            out.part_index.push_back(static_cast<int>(i));
            const double nan = std::numeric_limits<double>::quiet_NaN();
            out.values.push_back(on_field ? field_.values[k] : std::array<double, 3>{nan, nan, nan});
        }
    }
    return out;
}

}  // namespace ohs::gui
