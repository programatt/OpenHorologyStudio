// SPDX-License-Identifier: Apache-2.0
#include "mesh/tetmesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "core/error.hpp"

namespace ohs::mesh {

namespace {

V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 cross(const V3& a, const V3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

double signed_volume(const std::vector<V3>& n, const std::array<int, 10>& e) {
    return dot(cross(sub(n[e[1]], n[e[0]]), sub(n[e[2]], n[e[0]])), sub(n[e[3]], n[e[0]])) / 6.0;
}

}  // namespace

int tet_edge(int i, int j) {
    if (i > j) std::swap(i, j);
    if (i == 0 && j == 1) return 0;
    if (i == 1 && j == 2) return 1;
    if (i == 0 && j == 2) return 2;
    if (i == 0 && j == 3) return 3;
    if (i == 1 && j == 3) return 4;
    return 5;  // (2,3)
}

Mesh2d structured_rect(double x0, double y0, double x1, double y1, int nx, int ny) {
    Mesh2d m;
    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i <= nx; ++i) m.nodes.push_back({x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny});
    }
    auto id = [&](int i, int j) { return j * (nx + 1) + i; };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            m.triangles.push_back({id(i, j), id(i + 1, j), id(i + 1, j + 1)});
            m.triangles.push_back({id(i, j), id(i + 1, j + 1), id(i, j + 1)});
        }
    }
    return m;
}

TetMesh extrude(const Mesh2d& s, double z0, double z1, int layers, int order) {
    if (layers < 1) throw Error("extrude: need at least one layer");
    if (z1 <= z0) throw Error("extrude: the top must be above the bottom");
    TetMesh m;
    const int n2 = static_cast<int>(s.nodes.size());
    for (int k = 0; k <= layers; ++k) {
        const double z = z0 + (z1 - z0) * k / layers;
        for (const P2& p : s.nodes) m.nodes.push_back({p.x, p.y, z});
    }
    for (int k = 0; k < layers; ++k) {
        const int lo = k * n2, hi = (k + 1) * n2;
        for (const auto& t : s.triangles) {
            std::array<int, 3> v = t;
            std::sort(v.begin(), v.end());
            const int a = v[0], b = v[1], c = v[2];
            const std::array<std::array<int, 4>, 3> tets = {{{lo + a, lo + b, lo + c, hi + a},
                                                              {lo + b, lo + c, hi + a, hi + b},
                                                              {lo + c, hi + a, hi + b, hi + c}}};
            for (const auto& tt : tets) {
                std::array<int, 10> e{tt[0], tt[1], tt[2], tt[3], -1, -1, -1, -1, -1, -1};
                if (signed_volume(m.nodes, e) < 0) std::swap(e[2], e[3]);
                m.elems.push_back(e);
            }
        }
    }
    if (order == 2) promote_to_quadratic(m);
    return m;
}

void promote_to_quadratic(TetMesh& m) {
    if (m.order == 2) return;
    std::map<std::pair<int, int>, int> mids;
    auto mid = [&](int a, int b) {
        const auto key = std::make_pair(std::min(a, b), std::max(a, b));
        auto it = mids.find(key);
        if (it != mids.end()) return it->second;
        const V3& p = m.nodes[a];
        const V3& q = m.nodes[b];
        m.nodes.push_back({0.5 * (p.x + q.x), 0.5 * (p.y + q.y), 0.5 * (p.z + q.z)});
        return mids[key] = static_cast<int>(m.nodes.size()) - 1;
    };
    static const int pairs[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (auto& e : m.elems) {
        for (int k = 0; k < 6; ++k) e[4 + k] = mid(e[pairs[k][0]], e[pairs[k][1]]);
    }
    m.order = 2;
}

double volume(const TetMesh& m) {
    double v = 0;
    for (const auto& e : m.elems) v += signed_volume(m.nodes, e);
    return v;
}

std::vector<BoundaryFace> boundary_faces(const TetMesh& m) {
    static const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};  // outward
    struct Entry {
        BoundaryFace face;
        int count;
    };
    std::map<std::array<int, 3>, Entry> seen;
    for (std::size_t ei = 0; ei < m.elems.size(); ++ei) {
        const auto& e = m.elems[ei];
        for (const auto& f : faces) {
            std::array<int, 3> key{e[f[0]], e[f[1]], e[f[2]]};
            std::sort(key.begin(), key.end());
            auto [it, fresh] = seen.try_emplace(key);
            ++it->second.count;
            if (fresh) {
                BoundaryFace bf;
                bf.elem = static_cast<int>(ei);
                bf.nodes = {e[f[0]], e[f[1]], e[f[2]], -1, -1, -1};
                if (m.order == 2) {
                    bf.nodes[3] = e[4 + tet_edge(f[0], f[1])];
                    bf.nodes[4] = e[4 + tet_edge(f[1], f[2])];
                    bf.nodes[5] = e[4 + tet_edge(f[2], f[0])];
                }
                it->second.face = bf;
            }
        }
    }
    std::vector<BoundaryFace> out;
    for (const auto& [key, entry] : seen) {
        if (entry.count == 1) out.push_back(entry.face);
    }
    std::sort(out.begin(), out.end(), [](const BoundaryFace& a, const BoundaryFace& b) { return a.nodes < b.nodes; });
    return out;
}

V3 face_centroid(const TetMesh& m, const BoundaryFace& f) {
    const V3& a = m.nodes[f.nodes[0]];
    const V3& b = m.nodes[f.nodes[1]];
    const V3& c = m.nodes[f.nodes[2]];
    return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3};
}

V3 face_normal(const TetMesh& m, const BoundaryFace& f) {
    const V3 n = cross(sub(m.nodes[f.nodes[1]], m.nodes[f.nodes[0]]), sub(m.nodes[f.nodes[2]], m.nodes[f.nodes[0]]));
    const double l = std::sqrt(dot(n, n));
    return {n.x / l, n.y / l, n.z / l};
}

double face_area(const TetMesh& m, const BoundaryFace& f) {
    const V3 n = cross(sub(m.nodes[f.nodes[1]], m.nodes[f.nodes[0]]), sub(m.nodes[f.nodes[2]], m.nodes[f.nodes[0]]));
    return 0.5 * std::sqrt(dot(n, n));
}

std::vector<int> nodes_where(const TetMesh& m, const std::function<bool(const V3&)>& pred) {
    std::vector<int> out;
    for (std::size_t i = 0; i < m.nodes.size(); ++i) {
        if (pred(m.nodes[i])) out.push_back(static_cast<int>(i));
    }
    return out;
}

std::vector<BoundaryFace> faces_where(const TetMesh& m, const std::vector<BoundaryFace>& faces,
                                      const std::function<bool(const V3&, const V3&)>& pred) {
    std::vector<BoundaryFace> out;
    for (const BoundaryFace& f : faces) {
        if (pred(face_centroid(m, f), face_normal(m, f))) out.push_back(f);
    }
    return out;
}

double min_quality(const TetMesh& m) {
    double worst = 1e300;
    static const int pairs[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (const auto& e : m.elems) {
        double l2 = 0;
        for (const auto& p : pairs) {
            const V3 d = sub(m.nodes[e[p[0]]], m.nodes[e[p[1]]]);
            l2 += dot(d, d);
        }
        const double lrms = std::sqrt(l2 / 6.0);
        worst = std::min(worst, 6.0 * std::sqrt(2.0) * signed_volume(m.nodes, e) / (lrms * lrms * lrms));
    }
    return worst;
}

}  // namespace ohs::mesh
