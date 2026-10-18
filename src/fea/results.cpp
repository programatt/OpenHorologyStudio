// SPDX-License-Identifier: Apache-2.0
#include "fea/results.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

#include "core/error.hpp"

namespace ohs::fea {

std::string to_vtu(const mesh::TetMesh& m, const std::vector<NodeField>& fields) {
    const int npe = m.nodes_per_element();
    std::ostringstream os;
    os << std::setprecision(10);
    os << "<?xml version=\"1.0\"?>\n<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n"
       << "<UnstructuredGrid>\n<Piece NumberOfPoints=\"" << m.nodes.size() << "\" NumberOfCells=\"" << m.elems.size() << "\">\n";
    os << "<PointData>\n";
    for (const NodeField& f : fields) {
        if (f.values.size() != m.nodes.size() * static_cast<std::size_t>(f.components)) {
            throw Error("field '" + f.name + "' does not match the mesh");
        }
        os << "<DataArray type=\"Float64\" Name=\"" << f.name << "\" NumberOfComponents=\"" << f.components << "\" format=\"ascii\">\n";
        for (std::size_t i = 0; i < f.values.size(); ++i) os << f.values[i] << ((i + 1) % 6 == 0 ? "\n" : " ");
        os << "\n</DataArray>\n";
    }
    os << "</PointData>\n<Points>\n<DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& p : m.nodes) os << p.x << ' ' << p.y << ' ' << p.z << '\n';
    os << "</DataArray>\n</Points>\n<Cells>\n<DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (const auto& e : m.elems) {
        for (int k = 0; k < npe; ++k) os << e[k] << ' ';
        os << '\n';
    }
    os << "</DataArray>\n<DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (std::size_t i = 1; i <= m.elems.size(); ++i) os << i * static_cast<std::size_t>(npe) << ' ';
    os << "\n</DataArray>\n<DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (std::size_t i = 0; i < m.elems.size(); ++i) os << (npe == 4 ? 10 : 24) << ' ';
    os << "\n</DataArray>\n</Cells>\n</Piece>\n</UnstructuredGrid>\n</VTKFile>\n";
    return os.str();
}

void write_vtu(const std::filesystem::path& path, const mesh::TetMesh& m, const std::vector<NodeField>& fields) {
    std::ofstream f(path);
    if (!f) throw Error("cannot write " + path.string());
    f << to_vtu(m, fields);
}

std::vector<NodeField> static_fields(const StaticResult& r) {
    std::vector<NodeField> out;
    out.push_back({"displacement", std::vector<double>(r.u.data(), r.u.data() + r.u.size()), 3});
    out.push_back({"von_mises", r.von_mises, 1});
    NodeField s{"stress", {}, 6};
    for (const Stress& st : r.stress) s.values.insert(s.values.end(), st.begin(), st.end());
    out.push_back(std::move(s));
    return out;
}

NodeField mode_field(const Vec& mode, const std::string& name) {
    return {name, std::vector<double>(mode.data(), mode.data() + mode.size()), 3};
}

}  // namespace ohs::fea
