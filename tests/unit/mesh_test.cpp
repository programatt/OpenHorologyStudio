// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <map>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "core/error.hpp"
#include "mesh/mesh2d.hpp"

using namespace ohs;
using namespace ohs::mesh;
using Catch::Approx;

namespace {

Ring circle(double cx, double cy, double r, int n) {
    Ring ring;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * std::numbers::pi * i / n;
        ring.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return ring;
}

Ring rect(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

// Every interior edge is shared by two triangles, every boundary edge by one, and no triangle
// is inverted: the mesh is a valid, conforming surface.
void check_valid(const Mesh2d& m) {
    std::map<std::pair<int, int>, int> uses;
    for (const auto& t : m.triangles) {
        const P2& a = m.nodes[t[0]];
        const P2& b = m.nodes[t[1]];
        const P2& c = m.nodes[t[2]];
        REQUIRE((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0);
        for (int k = 0; k < 3; ++k) {
            const int u = t[k], v = t[(k + 1) % 3];
            ++uses[{std::min(u, v), std::max(u, v)}];
        }
    }
    std::map<std::pair<int, int>, int> bnd;
    for (const auto& s : m.boundary) ++bnd[{std::min(s[0], s[1]), std::max(s[0], s[1])}];
    for (const auto& [edge, n] : uses) {
        REQUIRE(n <= 2);
        if (n == 1) REQUIRE(bnd.count(edge) == 1);
        else REQUIRE(bnd.count(edge) == 0);
    }
    for (const auto& [edge, n] : bnd) REQUIRE(uses.count(edge) == 1);
}

}  // namespace

TEST_CASE("mesh2d: a square is covered exactly with good triangles") {
    Mesh2dOptions o;
    o.size = 0.1;
    const Mesh2d m = triangulate({{rect(0, 0, 1, 1)}}, o);
    check_valid(m);
    CHECK(area(m) == Approx(1.0).epsilon(1e-12));
    CHECK(m.triangles.size() > 150);
    CHECK(m.triangles.size() < 700);
    CHECK(angle_range(m)[0] > 20.0);
    CHECK(angle_range(m)[1] < 125.0);
}

TEST_CASE("mesh2d: a ring with a hole keeps the hole and the area of the polygons") {
    Mesh2dOptions o;
    o.size = 0.2;
    const Ring outer = circle(0, 0, 2, 120), hole = circle(0, 0, 0.8, 60);
    const Mesh2d m = triangulate({{outer, hole}}, o);
    check_valid(m);
    const double polygons = 0.5 * 120 * 4 * std::sin(2 * std::numbers::pi / 120) - 0.5 * 60 * 0.64 * std::sin(2 * std::numbers::pi / 60);
    CHECK(area(m) == Approx(polygons).epsilon(1e-9));
    CHECK(angle_range(m)[0] > 19.0);
    for (const P2& p : m.nodes) CHECK(std::hypot(p.x, p.y) > 0.8 * std::cos(std::numbers::pi / 60) - 1e-9);
}

TEST_CASE("mesh2d: a size field grades the mesh towards a feature") {
    Mesh2dOptions o;
    o.size = 0.5;
    o.size_at = [](P2 p) { return 0.05 + 0.2 * std::hypot(p.x - 1, p.y - 0.5); };
    const Mesh2d m = triangulate({{rect(0, 0, 4, 1)}}, o);
    check_valid(m);
    int near = 0, far = 0;
    for (const auto& t : m.triangles) {
        const P2 c{(m.nodes[t[0]].x + m.nodes[t[1]].x + m.nodes[t[2]].x) / 3, (m.nodes[t[0]].y + m.nodes[t[1]].y + m.nodes[t[2]].y) / 3};
        (std::hypot(c.x - 1, c.y - 0.5) < 0.5 ? near : far) += 1;
    }
    CHECK(near > 100);
    // Triangles are much smaller near the feature than at the far end.
    double h_near = 0, h_far = 0;
    int n_near = 0, n_far = 0;
    for (const auto& t : m.triangles) {
        const P2& a = m.nodes[t[0]];
        const P2& b = m.nodes[t[1]];
        const P2 c{(a.x + b.x + m.nodes[t[2]].x) / 3, (a.y + b.y + m.nodes[t[2]].y) / 3};
        const double e = std::hypot(a.x - b.x, a.y - b.y);
        if (std::hypot(c.x - 1, c.y - 0.5) < 0.3) { h_near += e; ++n_near; }
        if (c.x > 3.5) { h_far += e; ++n_far; }
    }
    CHECK(h_near / n_near < 0.5 * (h_far / n_far));
}

TEST_CASE("mesh2d: a concave outline with a re-entrant corner") {
    const Ring l{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};
    Mesh2dOptions o;
    o.size = 0.15;
    const Mesh2d m = triangulate({{l}}, o);
    check_valid(m);
    CHECK(area(m) == Approx(3.0).epsilon(1e-12));
    CHECK(angle_range(m)[0] > 20.0);
}

TEST_CASE("mesh2d: input errors and the node budget are reported") {
    Mesh2dOptions o;
    o.size = 0.1;
    CHECK_THROWS_AS(triangulate({}, o), Error);
    CHECK_THROWS_AS(triangulate({{Ring{{0, 0}, {1, 0}}}}, o), Error);
    o.size = 0;
    CHECK_THROWS_AS(triangulate({{rect(0, 0, 1, 1)}}, o), Error);
    o.size = 0.001;
    o.max_nodes = 500;
    CHECK_THROWS_AS(triangulate({{rect(0, 0, 1, 1)}}, o), Error);
}

TEST_CASE("mesh2d: the same input gives the same mesh") {
    Mesh2dOptions o;
    o.size = 0.2;
    const Mesh2d a = triangulate({{circle(0, 0, 1, 50)}}, o);
    const Mesh2d b = triangulate({{circle(0, 0, 1, 50)}}, o);
    REQUIRE(a.nodes.size() == b.nodes.size());
    REQUIRE(a.triangles == b.triangles);
}
