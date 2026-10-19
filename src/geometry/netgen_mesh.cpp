// SPDX-License-Identifier: Apache-2.0
#include "geometry/netgen_mesh.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "core/error.hpp"
#include "geometry/step_io.hpp"

namespace ohs::geometry {

namespace {

// Writes "N", N lines of x y z, "E" and E lines of four 0-based corner indices.
const char* const kScript = R"PY(import sys
from netgen.occ import OCCGeometry

step, size, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
mesh = OCCGeometry(step).GenerateMesh(maxh=size)
points = list(mesh.Points())
tets = list(mesh.Elements3D())
with open(out, "w") as f:
    f.write("%d\n" % len(points))
    for p in points:
        x, y, z = p.p
        f.write("%r %r %r\n" % (x, y, z))
    f.write("%d\n" % len(tets))
    for t in tets:
        f.write("%d %d %d %d\n" % tuple(v.nr - 1 for v in t.vertices))
)PY";

std::string quote(const std::filesystem::path& p) {
    std::string out = "'";
    for (char c : p.string()) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    return out + "'";
}

bool imports_netgen(const std::filesystem::path& python) {
    const std::string cmd = quote(python) + " -c 'import netgen.occ' > /dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
}

}  // namespace

std::optional<std::filesystem::path> find_netgen_python() {
    static const std::optional<std::filesystem::path> found = [] () -> std::optional<std::filesystem::path> {
        if (const char* env = std::getenv("OHS_NETGEN_PYTHON")) {
            if (std::filesystem::exists(env) && imports_netgen(env)) return std::filesystem::path(env);
            return std::nullopt;  // an explicit choice that does not work is not silently replaced
        }
        if (const char* path = std::getenv("PATH")) {
            std::istringstream ss(path);
            std::string dir;
            while (std::getline(ss, dir, ':')) {
                const auto candidate = std::filesystem::path(dir) / "python3";
                if (std::filesystem::exists(candidate)) return imports_netgen(candidate) ? std::optional(candidate) : std::nullopt;
            }
        }
        return std::nullopt;
    }();
    return found;
}

mesh::TetMesh mesh_solid_with_netgen(const TopoDS_Shape& shape, double max_size, int order, const std::filesystem::path& work_dir) {
    if (max_size <= 0) throw Error("the element size must be positive");
    if (order != 1 && order != 2) throw Error("the element order must be 1 or 2");
    const auto python = find_netgen_python();
    if (!python) {
        throw Error("Netgen is not available: install it with 'pip install netgen-mesher' (into the python3 on PATH, or set OHS_NETGEN_PYTHON to the interpreter that has it)");
    }
    std::filesystem::create_directories(work_dir);
    geometry::ExportPart part;
    part.name = "part";
    part.shape = shape;
    export_step({part}, work_dir / "part.step");
    {
        std::ofstream script(work_dir / "mesh.py");
        script << kScript;
    }
    const auto result = work_dir / "mesh.txt";
    std::filesystem::remove(result);
    const std::string cmd = "cd " + quote(work_dir) + " && " + quote(*python) + " mesh.py part.step " + std::to_string(max_size) + " mesh.txt > netgen.log 2>&1";
    if (std::system(cmd.c_str()) != 0 || !std::filesystem::exists(result)) {
        throw Error("Netgen failed to mesh the part; see " + (work_dir / "netgen.log").string());
    }

    std::ifstream in(result);
    std::size_t n = 0, e = 0;
    if (!(in >> n) || n == 0) throw Error("Netgen returned no nodes");
    mesh::TetMesh out;
    out.order = 1;
    out.nodes.resize(n);
    for (auto& p : out.nodes) in >> p.x >> p.y >> p.z;
    if (!(in >> e) || e == 0) throw Error("Netgen returned no tetrahedra");
    out.elems.resize(e);
    for (auto& el : out.elems) {
        el.fill(-1);
        in >> el[0] >> el[1] >> el[2] >> el[3];
        if (!in) throw Error("the Netgen result file is truncated");
        for (int k = 0; k < 4; ++k) {
            if (el[k] < 0 || static_cast<std::size_t>(el[k]) >= n) throw Error("the Netgen result file has a bad node index");
        }
        const auto& a = out.nodes[static_cast<std::size_t>(el[0])];
        const auto& b = out.nodes[static_cast<std::size_t>(el[1])];
        const auto& c = out.nodes[static_cast<std::size_t>(el[2])];
        const auto& d = out.nodes[static_cast<std::size_t>(el[3])];
        const double det = (b.x - a.x) * ((c.y - a.y) * (d.z - a.z) - (c.z - a.z) * (d.y - a.y)) -
                           (b.y - a.y) * ((c.x - a.x) * (d.z - a.z) - (c.z - a.z) * (d.x - a.x)) +
                           (b.z - a.z) * ((c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x));
        if (det < 0) std::swap(el[0], el[1]);  // Netgen orients its tetrahedra the other way
    }
    if (order == 2) mesh::promote_to_quadratic(out);
    return out;
}

}  // namespace ohs::geometry
