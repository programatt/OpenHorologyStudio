// SPDX-License-Identifier: Apache-2.0
#include "animation/scene.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include "core/error.hpp"

namespace ohs::animation {

Keyframes make_keyframes(const model::Movement& m, const kinematics::TrainAnalysis& train, const AnimationOptions& o) {
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
    // A lever or spring goes over a tooth of what it rides on in the time that tooth takes to pass (a date ring
    // that steps is sampled finely enough below). The stem turns at a few times the barrel.
    double tooth_dt = 1e300;
    for (const auto& art : train.articulations) {
        if (art.kind == kinematics::Articulation::Kind::Spin) {
            max_rate = std::max(max_rate, std::abs(art.gain) * two_pi / (train.root_period_hours * 3600.0) * o.time_scale);
            continue;
        }
        const auto& mm = train.mobiles[static_cast<std::size_t>(art.driver)];
        if (mm.index_teeth > 0 || mm.revs_per_hour <= 0 || art.teeth <= 0) continue;
        tooth_dt = std::min(tooth_dt, 3600.0 / (mm.revs_per_hour * art.teeth) / o.time_scale / 16.0);
    }
    double dt = std::min(1.0 / o.fps, tooth_dt);
    if (max_rate > 0) dt = std::min(dt, o.max_keyframe_rotation / max_rate);
    if (o.stepped) {
        const double beat_seconds_anim = 3600.0 / (train.beats_per_root_rev / train.root_period_hours) / o.time_scale;
        dt = std::min(dt, 0.2 * beat_seconds_anim / 8.0);
    }
    // A date ring moves only while its finger is in contact: sample that stretch of the day several times.
    for (const auto& mm : train.mobiles) {
        if (!mm.reached || mm.index_teeth <= 0 || mm.revs_per_hour <= 0) continue;
        const double driver_period = 3600.0 / (mm.revs_per_hour * mm.index_teeth);  // s
        dt = std::min(dt, pose.date_engage_fraction * driver_period / o.time_scale / 8.0);
    }
    const std::size_t frames = static_cast<std::size_t>(std::floor(o.duration_seconds / dt)) + 1;
    if (frames > o.max_keyframes) {
        throw Error("animation needs " + std::to_string(frames) + " keyframes; lower --time-scale or --duration");
    }

    Keyframes out;
    out.times.resize(frames);
    out.angles.assign(m.mobiles.size(), std::vector<double>(frames, 0.0));
    out.part_angles.assign(train.articulations.size(), std::vector<double>(frames, 0.0));
    std::vector<double> prev, prev_parts;
    for (std::size_t k = 0; k < frames; ++k) {
        out.times[k] = static_cast<double>(k) * dt;
        const auto angles = kinematics::mobile_angles(train, out.times[k] * o.time_scale, pose);
        for (std::size_t i = 0; i < angles.size(); ++i) {
            out.angles[i][k] = angles[i];
            if (k > 0) out.max_step = std::max(out.max_step, std::abs(angles[i] - prev[i]));
        }
        prev = angles;
        const auto parts = kinematics::articulation_angles(train, angles);
        for (std::size_t i = 0; i < parts.size(); ++i) {
            out.part_angles[i][k] = parts[i];
            if (k > 0) out.max_step = std::max(out.max_step, std::abs(parts[i] - prev_parts[i]));
        }
        prev_parts = parts;
    }
    return out;
}

// Flat-shaded triangles (three vertices each) keep the sharp tooth edges sharp.
MeshData tessellate(const TopoDS_Shape& shape, double deflection, double pivot_x, double pivot_y, double pivot_z) {
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
                                         static_cast<float>(p.Z() - pivot_z)});
                out.normals.push_back({static_cast<float>(n.X()), static_cast<float>(n.Y()), static_cast<float>(n.Z())});
            }
        }
    }
    return out;
}


}  // namespace ohs::animation
