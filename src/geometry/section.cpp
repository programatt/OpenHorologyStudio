// SPDX-License-Identifier: Apache-2.0
#include "geometry/section.hpp"

#include <algorithm>
#include <cmath>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
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

}  // namespace

mesh::Region section_region(const TopoDS_Shape& shape, double z, double deflection, double simplify) {
    BRepAlgoAPI_Section section(shape, gp_Pln(gp_Pnt(0, 0, z), gp_Dir(0, 0, 1)));
    if (!section.IsDone()) throw Error("the section plane could not be intersected with the part");

    Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape();
    for (TopExp_Explorer ex(section.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) edges->Append(ex.Current());
    if (edges->IsEmpty()) throw Error("the section plane does not cut the part");
    Handle(TopTools_HSequenceOfShape) wires;
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, 1e-6, false, wires);

    std::vector<mesh::Ring> rings;
    for (int w = 1; w <= wires->Length(); ++w) {
        mesh::Ring ring;
        for (BRepTools_WireExplorer we(TopoDS::Wire(wires->Value(w))); we.More(); we.Next()) {
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
