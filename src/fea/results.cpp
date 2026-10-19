// SPDX-License-Identifier: Apache-2.0
#include "fea/results.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

#include "core/error.hpp"

#ifdef OHS_WITH_HDF5
#include <hdf5.h>
#endif

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

#ifdef OHS_WITH_HDF5
namespace {

/// Owns an HDF5 identifier and closes it with `close`.
struct Handle {
    hid_t id;
    herr_t (*close)(hid_t);
    Handle(hid_t i, herr_t (*c)(hid_t), const char* what) : id(i), close(c) {
        if (id < 0) throw Error(std::string("HDF5: cannot create ") + what);
    }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    ~Handle() { close(id); }
};

void put_dataset(hid_t where, const std::string& name, hid_t type, std::vector<hsize_t> dims, const void* data) {
    Handle space(H5Screate_simple(static_cast<int>(dims.size()), dims.data(), nullptr), H5Sclose, "dataspace");
    Handle ds(H5Dcreate2(where, name.c_str(), type, space.id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT), H5Dclose, name.c_str());
    if (H5Dwrite(ds.id, type == H5T_NATIVE_INT ? H5T_NATIVE_INT : H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data) < 0) {
        throw Error("HDF5: cannot write dataset " + name);
    }
}

}  // namespace
#endif

bool hdf5_available() {
#ifdef OHS_WITH_HDF5
    return true;
#else
    return false;
#endif
}

void write_hdf5([[maybe_unused]] const std::filesystem::path& path, [[maybe_unused]] const mesh::TetMesh& m,
                [[maybe_unused]] const std::vector<NodeField>& fields) {
#ifdef OHS_WITH_HDF5
    for (const NodeField& f : fields) {
        if (f.values.size() != m.nodes.size() * static_cast<std::size_t>(f.components)) {
            throw Error("field '" + f.name + "' does not match the mesh");
        }
    }
    Handle file(H5Fcreate(path.string().c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose, path.string().c_str());
    Handle mesh_group(H5Gcreate2(file.id, "/mesh", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT), H5Gclose, "/mesh");
    std::vector<double> xyz;
    xyz.reserve(m.nodes.size() * 3);
    for (const auto& p : m.nodes) xyz.insert(xyz.end(), {p.x, p.y, p.z});
    put_dataset(mesh_group.id, "nodes", H5T_NATIVE_DOUBLE, {m.nodes.size(), 3}, xyz.data());
    const int npe = m.nodes_per_element();
    std::vector<int> conn;
    conn.reserve(m.elems.size() * static_cast<std::size_t>(npe));
    for (const auto& e : m.elems) conn.insert(conn.end(), e.begin(), e.begin() + npe);
    put_dataset(mesh_group.id, "elements", H5T_NATIVE_INT, {m.elems.size(), static_cast<hsize_t>(npe)}, conn.data());
    {
        Handle scalar(H5Screate(H5S_SCALAR), H5Sclose, "dataspace");
        Handle attr(H5Acreate2(mesh_group.id, "order", H5T_NATIVE_INT, scalar.id, H5P_DEFAULT, H5P_DEFAULT), H5Aclose, "attribute");
        if (H5Awrite(attr.id, H5T_NATIVE_INT, &m.order) < 0) throw Error("HDF5: cannot write the element order");
    }
    Handle field_group(H5Gcreate2(file.id, "/fields", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT), H5Gclose, "/fields");
    for (const NodeField& f : fields) {
        put_dataset(field_group.id, f.name, H5T_NATIVE_DOUBLE, {m.nodes.size(), static_cast<hsize_t>(f.components)}, f.values.data());
    }
#else
    throw Error("this build of ohs has no HDF5 support; install libhdf5-dev and reconfigure, or use --vtu");
#endif
}

namespace {

struct Array {
    std::string name;
    int components{1};
    std::vector<double> values;
};

// The text between the quotes after `key=`, or empty.
std::string attribute(const std::string& tag, const std::string& key) {
    const std::size_t at = tag.find(key + "=\"");
    if (at == std::string::npos) return {};
    const std::size_t from = at + key.size() + 2;
    return tag.substr(from, tag.find('"', from) - from);
}

}  // namespace

VtuData parse_vtu(const std::string& text) {
    if (text.find("<VTKFile") == std::string::npos || text.find("UnstructuredGrid") == std::string::npos) throw Error("not a VTU file");
    // Every DataArray in file order, tagged with the section it sits in.
    std::vector<std::pair<std::string, Array>> arrays;
    std::string section;
    for (std::size_t pos = 0; pos < text.size();) {
        const std::size_t open = text.find('<', pos);
        if (open == std::string::npos) break;
        const std::size_t close = text.find('>', open);
        if (close == std::string::npos) throw Error("the VTU file is truncated");
        const std::string tag = text.substr(open, close - open + 1);
        pos = close + 1;
        for (const char* s : {"PointData", "Points", "Cells", "CellData"}) {
            if (tag == std::string("<") + s + ">") section = s;
        }
        if (tag.rfind("<DataArray", 0) != 0) continue;
        if (attribute(tag, "format") != "ascii") throw Error("only ASCII VTU files can be read");
        const std::size_t end = text.find("</DataArray>", pos);
        if (end == std::string::npos) throw Error("the VTU file is truncated");
        Array a;
        a.name = attribute(tag, "Name");
        const std::string nc = attribute(tag, "NumberOfComponents");
        a.components = nc.empty() ? 1 : std::stoi(nc);
        std::istringstream in(text.substr(pos, end - pos));
        double v;
        while (in >> v) a.values.push_back(v);
        arrays.emplace_back(section, std::move(a));
        pos = end;
    }

    VtuData out;
    const Array *points = nullptr, *connectivity = nullptr, *offsets = nullptr, *types = nullptr;
    for (const auto& [where, a] : arrays) {
        if (where == "Points") points = &a;
        else if (where == "Cells" && a.name == "connectivity") connectivity = &a;
        else if (where == "Cells" && a.name == "offsets") offsets = &a;
        else if (where == "Cells" && a.name == "types") types = &a;
        else if (where == "PointData") out.fields.push_back({a.name, a.values, a.components});
    }
    if (!points || !connectivity || !offsets || !types || points->components != 3 || points->values.size() % 3 != 0) {
        throw Error("the VTU file has no tetrahedral mesh");
    }
    const std::size_t nodes = points->values.size() / 3, cells = types->values.size();
    if (cells == 0 || offsets->values.size() != cells) throw Error("the VTU file has no cells");
    const int type = static_cast<int>(types->values[0]);
    if (type != 10 && type != 24) throw Error("the VTU file holds cells other than tetrahedra");
    const int npe = type == 10 ? 4 : 10;
    if (connectivity->values.size() != cells * static_cast<std::size_t>(npe)) throw Error("the VTU file has mixed or malformed cells");
    out.mesh.order = npe == 4 ? 1 : 2;
    out.mesh.nodes.resize(nodes);
    for (std::size_t i = 0; i < nodes; ++i) out.mesh.nodes[i] = {points->values[3 * i], points->values[3 * i + 1], points->values[3 * i + 2]};
    out.mesh.elems.resize(cells);
    for (std::size_t c = 0; c < cells; ++c) {
        out.mesh.elems[c].fill(-1);
        for (int k = 0; k < npe; ++k) {
            const int id = static_cast<int>(connectivity->values[c * static_cast<std::size_t>(npe) + static_cast<std::size_t>(k)]);
            if (id < 0 || static_cast<std::size_t>(id) >= nodes) throw Error("the VTU file has a bad node index");
            out.mesh.elems[c][static_cast<std::size_t>(k)] = id;
        }
    }
    for (const NodeField& f : out.fields) {
        if (f.values.size() != nodes * static_cast<std::size_t>(f.components)) throw Error("field '" + f.name + "' does not match the mesh");
    }
    return out;
}

VtuData read_vtu(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw Error("cannot read " + path.string());
    std::stringstream buffer;
    buffer << in.rdbuf();
    return parse_vtu(buffer.str());
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
