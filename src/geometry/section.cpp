// SPDX-License-Identifier: Apache-2.0
#include "geometry/section.hpp"

#include <algorithm>
#include <cmath>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pln.hxx>

#include "core/error.hpp"

namespace ohs::geometry {

namespace {

double ring_area(const mesh::Ring& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.size(); ++i) {
        const auto& p = r[i];
        const auto& q = r[(i + 1) % r.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

double point_segment_distance(const mesh::P2& p, const mesh::P2& a, const mesh::P2& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    const double t = len2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

void douglas_peucker(const std::vector<mesh::P2>& pts, std::size_t first, std::size_t last, double tol, std::vector<char>& keep) {
    if (last <= first + 1) return;
    double worst = 0;
    std::size_t at = first;
    for (std::size_t i = first + 1; i < last; ++i) {
        const double d = point_segment_distance(pts[i], pts[first], pts[last]);
        if (d > worst) {
            worst = d;
            at = i;
        }
    }
    if (worst > tol) {
        keep[at] = 1;
        douglas_peucker(pts, first, at, tol, keep);
        douglas_peucker(pts, at, last, tol, keep);
    }
}

/// Thins a closed polygon so that it stays within `tol` of the original.
mesh::Ring simplify_ring(const mesh::Ring& ring, double tol) {
    if (tol <= 0 || ring.size() < 8) return ring;
    // Split the loop at point 0 and its farthest point so both halves are open chains.
    std::size_t far = 0;
    double best = 0;
    for (std::size_t i = 1; i < ring.size(); ++i) {
        const double d = std::hypot(ring[i].x - ring[0].x, ring[i].y - ring[0].y);
        if (d > best) {
            best = d;
            far = i;
        }
    }
    std::vector<char> keep(ring.size(), 0);
    keep[0] = keep[far] = 1;
    douglas_peucker(ring, 0, far, tol, keep);
    std::vector<mesh::P2> second(ring.begin() + static_cast<std::ptrdiff_t>(far), ring.end());
    second.push_back(ring[0]);
    std::vector<char> keep2(second.size(), 0);
    keep2.front() = keep2.back() = 1;
    douglas_peucker(second, 0, second.size() - 1, tol, keep2);
    mesh::Ring out;
    for (std::size_t i = 0; i < far; ++i) {
        if (keep[i]) out.push_back(ring[i]);
    }
    for (std::size_t i = 0; i + 1 < second.size(); ++i) {
        if (keep2[i]) out.push_back(second[i]);
    }
    return out.size() >= 3 ? out : ring;
}

bool inside(const mesh::Ring& ring, const mesh::P2& p) {
    bool in = false;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const auto& a = ring[i];
        const auto& b = ring[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

/// Parts that are straight extrusions along Z (plates, bridges and wheels from most CAD tools) have
/// the same section at every height: the outline of the bottom face. Finding it from the topology
/// takes microseconds, where intersecting a plane with a solid of thousands of faces takes seconds.
/// Returns false (leaving `wires` empty) when the part is not such a prism.
bool prism_outline(const TopoDS_Shape& shape, double z, std::vector<TopoDS_Wire>& wires) {
    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    if (z <= z0 + 1e-9 || z >= z1 - 1e-9) return false;
    const double slack = 1e-5 * std::max(1.0, z1 - z0);  // the bounding box is padded by the shape's tolerance
    double area = 0, top_area = 0;
    std::vector<TopoDS_Face> bottoms;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() == GeomAbs_Plane) {
            const gp_Pln plane = surface.Plane();
            const double nz = std::abs(plane.Axis().Direction().Z());
            if (nz < 1e-6) continue;                    // a vertical plane
            if (nz < 1.0 - 1e-9) return false;          // slanted
            const double at = plane.Location().Z();
            const bool bottom = std::abs(at - z0) <= slack, top = std::abs(at - z1) <= slack;
            if (!bottom && !top) return false;          // a step: not a prism
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            (bottom ? area : top_area) += props.Mass();
            if (bottom) bottoms.push_back(face);
        } else {
            // A curved side face must be vertical: its normal has no Z component anywhere on it.
            double u0, u1, v0, v1;
            u0 = surface.FirstUParameter();
            u1 = surface.LastUParameter();
            v0 = surface.FirstVParameter();
            v1 = surface.LastVParameter();
            if (std::isinf(u0) || std::isinf(u1) || std::isinf(v0) || std::isinf(v1)) return false;
            for (int i = 0; i <= 2; ++i) {
                for (int j = 0; j <= 2; ++j) {
                    BRepLProp_SLProps props(surface, u0 + (u1 - u0) * i / 2.0, v0 + (v1 - v0) * j / 2.0, 1, 1e-9);
                    if (!props.IsNormalDefined()) continue;
                    if (std::abs(props.Normal().Z()) > 1e-4) return false;
                }
            }
        }
    }
    // An extrusion has equal bottom and top faces.
    if (bottoms.empty() || std::abs(area - top_area) > 1e-7 * area) return false;
    for (const TopoDS_Face& face : bottoms) {
        for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next()) wires.push_back(TopoDS::Wire(ex.Current()));
    }
    return true;
}

}  // namespace

mesh::Region section_region(const TopoDS_Shape& shape, double z, double deflection, double simplify) {
    std::vector<TopoDS_Wire> wire_list;
    if (!prism_outline(shape, z, wire_list)) {
        BRepAlgoAPI_Section section(shape, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)), false);
        section.SetRunParallel(true);
        section.Approximation(false);
        section.ComputePCurveOn1(false);
        section.Build();
        if (!section.IsDone()) throw Error("the section plane could not be intersected with the part");

        Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape();
        for (TopExp_Explorer ex(section.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) edges->Append(ex.Current());
        if (edges->IsEmpty()) throw Error("the section plane does not cut the part");
        Handle(TopTools_HSequenceOfShape) wires;
        ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, 1e-6, false, wires);
        for (int w = 1; w <= wires->Length(); ++w) wire_list.push_back(TopoDS::Wire(wires->Value(w)));
    }

    std::vector<mesh::Ring> rings;
    for (const TopoDS_Wire& wire : wire_list) {
        mesh::Ring ring;
        for (BRepTools_WireExplorer we(wire); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            BRepAdaptor_Curve curve(edge);
            GCPnts_QuasiUniformDeflection sampler(curve, deflection);
            std::vector<mesh::P2> pts;
            if (sampler.IsDone()) {
                for (int i = 1; i <= sampler.NbPoints(); ++i) pts.push_back({sampler.Value(i).X(), sampler.Value(i).Y()});
            } else {
                pts.push_back({curve.Value(curve.FirstParameter()).X(), curve.Value(curve.FirstParameter()).Y()});
                pts.push_back({curve.Value(curve.LastParameter()).X(), curve.Value(curve.LastParameter()).Y()});
            }
            if (edge.Orientation() == TopAbs_REVERSED) std::reverse(pts.begin(), pts.end());
            for (const auto& p : pts) {
                if (ring.empty() || std::hypot(ring.back().x - p.x, ring.back().y - p.y) > 1e-9) ring.push_back(p);
            }
        }
        while (ring.size() > 1 && std::hypot(ring.front().x - ring.back().x, ring.front().y - ring.back().y) < 1e-9) ring.pop_back();
        if (ring.size() >= 3) rings.push_back(simplify_ring(ring, simplify));
    }
    if (rings.empty()) throw Error("the section of the part does not form closed outlines");

    std::sort(rings.begin(), rings.end(), [](const mesh::Ring& a, const mesh::Ring& b) { return std::abs(ring_area(a)) > std::abs(ring_area(b)); });
    mesh::Region region;
    region.rings.push_back(rings.front());
    for (std::size_t i = 1; i < rings.size(); ++i) {
        if (inside(rings.front(), rings[i].front())) region.rings.push_back(rings[i]);
    }
    return region;
}

}  // namespace ohs::geometry

