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
