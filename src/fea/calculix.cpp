// SPDX-License-Identifier: Apache-2.0
#include "fea/calculix.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "core/error.hpp"

namespace ohs::fea {

std::string write_calculix_input(const Model& m, const Vec& load, const std::string& title) {
    if (!m.materials.empty() || !m.element_material.empty()) throw Error("the CalculiX export supports a single material");
    if (!m.ties.empty()) throw Error("the CalculiX export does not support ties");
    const int npe = m.mesh.nodes_per_element();
    std::ostringstream os;
    os << std::setprecision(12);
    os << "** " << title << ": written by Open Horology Studio. Units: mm, N, MPa, tonne, s.\n*NODE\n";
    for (std::size_t i = 0; i < m.mesh.nodes.size(); ++i) {
        const auto& p = m.mesh.nodes[i];
        os << i + 1 << ", " << p.x << ", " << p.y << ", " << p.z << '\n';
    }
    os << "*ELEMENT, TYPE=" << (npe == 4 ? "C3D4" : "C3D10") << ", ELSET=EALL\n";
    for (std::size_t e = 0; e < m.mesh.elems.size(); ++e) {
        os << e + 1;
        for (int k = 0; k < npe; ++k) os << ", " << m.mesh.elems[e][k] + 1;
        os << '\n';
    }
    os << "*MATERIAL, NAME=MAT\n*ELASTIC\n" << m.material.youngs_mpa() << ", " << m.material.poisson
       << "\n*DENSITY\n" << m.material.density_t_mm3() << "\n*SOLID SECTION, ELSET=EALL, MATERIAL=MAT\n";
    os << "*STEP\n*STATIC\n";
    if (!m.fixed.empty()) {
        os << "*BOUNDARY\n";
        for (const Fixed& c : m.fixed) os << c.node + 1 << ", " << c.comp + 1 << ", " << c.comp + 1 << ", " << c.value << '\n';
    }
    bool any = false;
    for (Eigen::Index i = 0; i < load.size(); ++i) {
        if (load[i] == 0.0) continue;
        if (!any) os << "*CLOAD\n";
        any = true;
        os << i / 3 + 1 << ", " << i % 3 + 1 << ", " << load[i] << '\n';
    }
    os << "*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n";
    return os.str();
}

Vec parse_calculix_displacements(const std::string& frd, std::size_t node_count) {
    Vec u = Vec::Zero(static_cast<Eigen::Index>(3 * node_count));
    std::istringstream in(frd);
    std::string line;
    bool in_block = false;
    std::size_t found = 0;
    while (std::getline(in, line)) {
        if (line.size() >= 3 && line.compare(0, 3, " -4") == 0) {
            in_block = line.find("DISP") != std::string::npos;
            continue;
        }
        if (line.size() >= 3 && line.compare(0, 3, " -3") == 0) {
            in_block = false;
            continue;
        }
        if (!in_block || line.size() < 13 || line.compare(0, 3, " -1") != 0) continue;
        // Fixed format: " -1" + node number (10 chars) + 12 chars per value; negative values touch.
        const long node = std::stol(line.substr(3, 10));
        if (node < 1 || static_cast<std::size_t>(node) > node_count) throw Error("the CalculiX result refers to a node that does not exist");
        for (int c = 0; c < 3; ++c) {
            const std::size_t pos = 13 + 12 * static_cast<std::size_t>(c);
            if (line.size() < pos + 12) throw Error("the CalculiX result has a short displacement line");
            u[3 * (node - 1) + c] = std::stod(line.substr(pos, 12));
        }
        ++found;
    }
    if (found == 0) throw Error("no displacement block found in the CalculiX result");
    return u;
}

std::optional<std::filesystem::path> find_calculix() {
    if (const char* env = std::getenv("OHS_CCX")) {
        if (std::filesystem::exists(env)) return std::filesystem::path(env);
    }
    if (const char* path = std::getenv("PATH")) {
        std::istringstream ss(path);
        std::string dir;
        while (std::getline(ss, dir, ':')) {
            for (const char* name : {"ccx", "ccx_2.21", "ccx_2.20"}) {
                const auto candidate = std::filesystem::path(dir) / name;
                if (std::filesystem::exists(candidate)) return candidate;
            }
        }
    }
    return std::nullopt;
}

CrossCheck run_calculix_crosscheck(const Model& model, const Vec& load, const StaticResult& r, const std::filesystem::path& dir) {
    CrossCheck out;
    const auto ccx = find_calculix();
    if (!ccx) {
        out.note = "CalculiX (ccx) was not found on PATH; set OHS_CCX to its location to enable the cross-check";
        return out;
    }
    out.available = true;
    std::filesystem::create_directories(dir);
    {
        std::ofstream f(dir / "job.inp");
        f << write_calculix_input(model, load);
    }
    const std::string cmd = "cd '" + dir.string() + "' && '" + ccx->string() + "' -i job > ccx.log 2>&1";
    if (std::system(cmd.c_str()) != 0 || !std::filesystem::exists(dir / "job.frd")) {
        out.note = "CalculiX failed; see " + (dir / "ccx.log").string();
        out.available = false;
        return out;
    }
    std::ifstream frd(dir / "job.frd");
    std::stringstream buf;
    buf << frd.rdbuf();
    const Vec u = parse_calculix_displacements(buf.str(), model.mesh.nodes.size());
    out.ohs_max_displacement = r.u.cwiseAbs().maxCoeff();
    out.ccx_max_displacement = u.cwiseAbs().maxCoeff();
    out.max_displacement_diff = (u - r.u).cwiseAbs().maxCoeff() / std::max(out.ohs_max_displacement, 1e-300);
    return out;
}

}  // namespace ohs::fea
