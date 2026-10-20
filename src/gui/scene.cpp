// SPDX-License-Identifier: Apache-2.0
#include "gui/scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <system_error>

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
#include "geometry/part_cache.hpp"
#include "geometry/features.hpp"
#include "geometry/shape_groups.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"

namespace ohs::gui {

namespace {

constexpr double kDeflection = 0.02;
constexpr double kCoarseDeflection = 0.1;  ///< the mesh drawn while the view is moving: far fewer triangles
constexpr double kCoarseAngle = 1.2;       ///< radians between facets: a circle is about five segments

// The triangles a shape has been meshed into, in the CAD frame. OCCT writes the triangulation into the shape's
// faces, so two threads must never mesh shapes that share a TShape (see mesh_parts).
std::vector<Triangle> triangles_of(const TopoDS_Shape& shape) {
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
            out.push_back({{p1.X(), p1.Y(), p1.Z()}, {p2.X(), p2.Y(), p2.Z()}, {p3.X(), p3.Y(), p3.Z()}});
        }
    }
    return out;
}

// A part's display mesh in the CAD frame (oriented so the dial axis is Z, not yet moved to its pivot).
struct PartMesh {
    std::string name;
    bool closed{false};
    std::vector<Triangle> triangles;
    std::vector<Triangle> coarse;
};

// Meshes every part. Instances of one shape (screws, jewels and pins in a movement are many parts with one
// TShape) share their triangulation: meshing them on two threads at once corrupts it (it crashed with a
// double free on a 166-part movement). So the parts are grouped by TShape, one thread takes a whole group, and
// groups run in parallel. Each part's output goes in its own slot.
std::vector<PartMesh> mesh_parts(const std::vector<geometry::ImportedPart>& imported, geometry::Axis axis) {
    std::vector<PartMesh> out(imported.size());
    std::vector<TopoDS_Shape> shapes;
    shapes.reserve(imported.size());
    for (const auto& part : imported) shapes.push_back(part.shape);
    std::set<const void*> distinct;
    for (const auto& shape : shapes) distinct.insert(shape.TShape().get());
    const bool in_parallel = distinct.size() < 4;  // a few big shapes: let OCCT split each one instead
    geometry::parallel_for_shape_groups(shapes, [&](const std::vector<std::size_t>& group) {
        std::vector<TopoDS_Shape> placed;
        for (const std::size_t i : group) placed.push_back(geometry::orient_dial_axis_to_z(imported[i].shape, axis));
        // The coarse mesh first: meshing finer afterwards replaces it, where meshing coarser would keep the fine one. The
        // shape is meshed once for the group and each instance reads the triangles in its own placement.
        BRepMesh_IncrementalMesh(placed.front(), kCoarseDeflection, false, kCoarseAngle, in_parallel);
        for (std::size_t k = 0; k < group.size(); ++k) out[group[k]].coarse = triangles_of(placed[k]);
        BRepMesh_IncrementalMesh(placed.front(), kDeflection, false, 0.5, in_parallel);
        for (std::size_t k = 0; k < group.size(); ++k) {
            PartMesh& m = out[group[k]];
            m.name = imported[group[k]].name;
            m.triangles = triangles_of(placed[k]);
            // At least one solid, and no face that is not in a solid (a loose sheet is seen from both sides).
            m.closed = TopExp_Explorer(placed[k], TopAbs_SOLID).More() && !TopExp_Explorer(placed[k], TopAbs_FACE, TopAbs_SOLID).More();
        }
    }, in_parallel ? 1u : 0u);
    return out;
}

// ---- the display mesh cache -------------------------------------------------------------------------------------
// mesh.bin in the project's cache: magic, version, the CAD file's fingerprint, deflection, dial axis, then per part
// the name, closed flag and the triangles as float32 (CAD frame). Reading it replaces both the STEP read and the
// tessellation. Any mismatch or damage means "no cache".
constexpr std::uint32_t kMeshMagic = 0x4d53484f;  // "OHSM"
constexpr std::uint32_t kMeshVersion = 3;

std::filesystem::path mesh_path(const std::filesystem::path& project) { return geometry::cache_dir(project) / "mesh.bin"; }

template <class T>
void put(std::string& out, const T& v) {
    out.append(reinterpret_cast<const char*>(&v), sizeof v);
}

void write_mesh_cache(const std::filesystem::path& project, const std::string& fingerprint, geometry::Axis axis,
                      const std::vector<PartMesh>& meshes) {
    if (fingerprint.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(geometry::cache_dir(project), ec);
    if (ec) return;
    std::string out;
    put(out, kMeshMagic);
    put(out, kMeshVersion);
    put(out, static_cast<std::uint32_t>(fingerprint.size()));
    out.append(fingerprint);
    put(out, kDeflection);
    put(out, static_cast<std::uint8_t>(axis));
    put(out, static_cast<std::uint32_t>(meshes.size()));
    for (const PartMesh& m : meshes) {
        put(out, static_cast<std::uint32_t>(m.name.size()));
        out.append(m.name);
        put(out, static_cast<std::uint8_t>(m.closed));
        for (const std::vector<Triangle>* list : {&m.triangles, &m.coarse}) {
            put(out, static_cast<std::uint32_t>(list->size()));
            for (const Triangle& t : *list) {
                for (const Point3* p : {&t.a, &t.b, &t.c}) {
                    for (int k = 0; k < 3; ++k) put(out, static_cast<float>((*p)[k]));
                }
            }
        }
    }
    geometry::write_file_atomic(mesh_path(project), out);
}

bool read_mesh_cache(const std::filesystem::path& project, const std::string& fingerprint, geometry::Axis axis,
                     std::vector<PartMesh>& meshes) {
    std::ifstream in(mesh_path(project), std::ios::binary | std::ios::ate);
    if (!in) return false;
    std::string data(static_cast<std::size_t>(in.tellg()), '\0');
    in.seekg(0);
    in.read(data.data(), static_cast<std::streamsize>(data.size()));
    if (!in) return false;
    std::size_t at = 0;
    bool ok = true;
    auto take = [&](auto& v) {
        if (!ok || at + sizeof v > data.size()) return ok = false;
        std::memcpy(&v, data.data() + at, sizeof v);
        at += sizeof v;
        return true;
    };
    auto take_string = [&](std::string& v) {
        std::uint32_t n = 0;
        if (!take(n) || at + n > data.size()) return ok = false;
        v.assign(data, at, n);
        at += n;
        return true;
    };
    std::uint32_t magic = 0, version = 0, count = 0;
    std::string stored;
    double deflection = 0;
    std::uint8_t stored_axis = 0;
    if (!take(magic) || magic != kMeshMagic || !take(version) || version != kMeshVersion) return false;
    if (!take_string(stored) || stored != fingerprint || !take(deflection) || deflection != kDeflection) return false;
    if (!take(stored_axis) || stored_axis != static_cast<std::uint8_t>(axis) || !take(count)) return false;
    std::vector<PartMesh> out(count);
    for (PartMesh& m : out) {
        std::uint8_t closed = 0;
        if (!take_string(m.name) || !take(closed)) return false;
        m.closed = closed != 0;
        for (std::vector<Triangle>* list : {&m.triangles, &m.coarse}) {
            std::uint32_t triangles = 0;
            if (!take(triangles) || at + static_cast<std::size_t>(triangles) * 9 * sizeof(float) > data.size()) return false;
            list->resize(triangles);
            for (Triangle& t : *list) {
                for (Point3* p : {&t.a, &t.b, &t.c}) {
                    for (int k = 0; k < 3; ++k) {
                        float v = 0;
                        take(v);
                        (*p)[k] = v;
                    }
                }
            }
        }
    }
    if (!ok) return false;
    meshes = std::move(out);
    return true;
}

// The articulation (a stem, lever or spring that moves) of a component, or -1.
int articulation_of(const kinematics::TrainAnalysis& train, int component) {
    for (std::size_t i = 0; i < train.articulations.size(); ++i) {
        if (train.articulations[i].component == component) return static_cast<int>(i);
    }
    return -1;
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
    // The project's cache describes the CAD file it was imported from: it is used for any file it matches (the
    // fingerprint says so) but only written from that file.
    const std::string fingerprint = geometry::cad_fingerprint(step);
    const bool owns_cache = step_override.empty() || geometry::is_project_source(step, project, s.movement_.source_geometry);

    std::vector<PartMesh> meshes;
    bool cached = !fingerprint.empty() && read_mesh_cache(project, fingerprint, dial_axis, meshes);
    if (cached) {
        std::map<std::string, int> have;
        for (const PartMesh& m : meshes) have[m.name] = 1;
        for (const auto& comp : s.movement_.components) cached = cached && have.contains(comp.features.name);
    }
    if (!cached) {
        meshes = mesh_parts(geometry::load_parts(step, project, owns_cache), dial_axis);
        if (owns_cache && !fingerprint.empty()) write_mesh_cache(project, fingerprint, dial_axis, meshes);
    }
    std::map<std::string, const PartMesh*> by_name;
    for (const PartMesh& m : meshes) by_name[m.name] = &m;

    const std::vector<int> mobile_at = s.movement_.mobile_index();
    for (std::size_t c = 0; c < s.movement_.components.size(); ++c) {
        const auto& comp = s.movement_.components[c];
        const auto found = by_name.find(comp.features.name);
        if (found == by_name.end()) throw Error("no imported part named '" + comp.features.name + "'; was the right STEP file given?");
        ScenePart part;
        part.name = comp.features.name;
        part.role = comp.role;
        part.mobile = mobile_at[c];
        part.articulation = articulation_of(s.train_, static_cast<int>(c));
        if (part.mobile >= 0) {
            part.pivot_x = s.movement_.mobiles[part.mobile].center.x;
            part.pivot_y = s.movement_.mobiles[part.mobile].center.y;
        }
        part.closed = found->second->closed;
        part.triangles = found->second->triangles;
        part.coarse = found->second->coarse;
        part.z_min = std::numeric_limits<double>::max();
        part.z_max = std::numeric_limits<double>::lowest();
        for (auto& t : part.triangles) {
            for (Point3* p : {&t.a, &t.b, &t.c}) {
                (*p)[0] -= part.pivot_x;
                (*p)[1] -= part.pivot_y;
                part.z_min = std::min(part.z_min, (*p)[2]);
                part.z_max = std::max(part.z_max, (*p)[2]);
            }
        }
        for (auto& t : part.coarse) {
            for (Point3* p : {&t.a, &t.b, &t.c}) {
                (*p)[0] -= part.pivot_x;
                (*p)[1] -= part.pivot_y;
            }
        }
        s.parts_.push_back(std::move(part));
    }
    s.update_bounds();
    s.update_review();
    return s;
}

void Scene::update_review() {
    review_.issues = rigging::check_rig(movement_);
    review_.chain = rigging::train_chain(movement_);
    review_.suggestions = rigging::suggest_meshes(movement_, {});
    review_.walk = rigging::walk_train(movement_, review_.chain, review_.suggestions);
}

bool Scene::reload_rig(const std::filesystem::path& project, double beat_rate_vph) {
    return reload_rig(model::load_rig(project), beat_rate_vph);
}

bool Scene::reload_rig(model::Movement m, double beat_rate_vph) {
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
    const std::vector<int> mobile_at = movement_.mobile_index();
    for (std::size_t c = 0; c < parts_.size(); ++c) {
        ScenePart& part = parts_[c];
        const int mobile = mobile_at[c];
        const double px = mobile >= 0 ? movement_.mobiles[static_cast<std::size_t>(mobile)].center.x : 0.0;
        const double py = mobile >= 0 ? movement_.mobiles[static_cast<std::size_t>(mobile)].center.y : 0.0;
        const double dx = part.pivot_x - px, dy = part.pivot_y - py;
        if (dx != 0.0 || dy != 0.0) {
            ++geometry_revision_;
            for (auto* list : {&part.triangles, &part.coarse}) {
                for (auto& t : *list) {
                    for (Point3* p : {&t.a, &t.b, &t.c}) {
                        (*p)[0] += dx;
                        (*p)[1] += dy;
                    }
                }
            }
        }
        part.role = movement_.components[c].role;
        part.mobile = mobile;
        part.articulation = articulation_of(train_, static_cast<int>(c));
        part.pivot_x = px;
        part.pivot_y = py;
    }
    update_bounds();
    update_review();
    ++revision_;
    forces_.clear();
    if (has_field_) clear_field();
    return true;
}

void Scene::set_visible(const std::string& part, bool visible) {
    if (visible) hidden_.erase(part);
    else hidden_.insert(part);
    ++revision_;
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

std::vector<Scene::PartPose> Scene::part_poses() const {
    std::vector<double> angles(movement_.mobiles.size(), 0.0);
    std::vector<double> part_angles(train_.articulations.size(), 0.0);
    if (can_animate()) {
        angles = kinematics::mobile_angles(train_, time_, {stepped_, 0.2});
        part_angles = kinematics::articulation_angles(train_, angles);
    }
    const double zc = centre_[2];

    std::vector<PartPose> out(parts_.size());
    for (std::size_t i = 0; i < parts_.size(); ++i) {
        const ScenePart& part = parts_[i];
        PartPose& pose = out[i];
        pose.visible = visible(part.name);
        const double lift = explode_ * ((part.z_min + part.z_max) / 2 - zc) * 2.0;
        const kinematics::Articulation* art =
            part.articulation >= 0 ? &train_.articulations[static_cast<std::size_t>(part.articulation)] : nullptr;
        if (art) {
            // A stem, lever or spring turns about a line of its own: Rodrigues' rotation about the axis k through the
            // point P is R = cos I + sin [k]x + (1 - cos) k kT, and q = P + R (p - P).
            const double turn = part_angles[static_cast<std::size_t>(part.articulation)];
            const double ct = std::cos(turn), st = std::sin(turn);
            const double* k = art->axis;
            const double kx[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
            for (int r = 0; r < 3; ++r) {
                double t = art->point[r];
                for (int c = 0; c < 3; ++c) {
                    const double R = (r == c ? ct : 0.0) + st * kx[r][c] + (1.0 - ct) * k[r] * k[c];
                    pose.m[static_cast<std::size_t>(4 * r + c)] = R;
                    t -= R * art->point[c];
                }
                pose.m[static_cast<std::size_t>(4 * r + 3)] = t + (r == 2 ? lift : 0.0);
            }
        } else {
            const double angle = part.mobile >= 0 && part.mobile < static_cast<int>(angles.size()) ? angles[part.mobile] : 0.0;
            const double ca = std::cos(angle), sa = std::sin(angle);
            pose.m = {ca, -sa, 0, part.pivot_x, sa, ca, 0, part.pivot_y, 0, 0, 1, lift};
        }
    }
    return out;
}

Scene::Drawn Scene::draw_list(bool coarse) const {
    const std::vector<PartPose> poses = part_poses();

    Drawn out;
    std::size_t most = has_field_ ? field_.triangles.size() : 0;  // built every frame: grow the lists once
    for (const ScenePart& part : parts_) most += part.triangles.size();
    out.triangles.reserve(most);
    out.part_index.reserve(most);
    out.values.reserve(most);
    for (std::size_t i = 0; i < parts_.size(); ++i) {
        const ScenePart& part = parts_[i];
        const PartPose& pose = poses[i];
        if (!pose.visible) continue;
        const double* m = pose.m.data();
        auto place = [m](const Point3& p) -> Point3 {
            return {m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3], m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
                    m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11]};
        };
        const bool on_field = has_field_ && field_.part == part.name;
        const std::vector<Triangle>& tris = on_field ? field_.triangles : coarse && !part.coarse.empty() ? part.coarse : part.triangles;
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
