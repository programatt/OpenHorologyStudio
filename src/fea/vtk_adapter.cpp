// SPDX-License-Identifier: Apache-2.0
#include "fea/vtk_adapter.hpp"

#include "core/error.hpp"

#ifdef OHS_WITH_VTK
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>
#endif

namespace ohs::fea {

bool vtk_available() {
#ifdef OHS_WITH_VTK
    return true;
#else
    return false;
#endif
}

#ifdef OHS_WITH_VTK

void write_vtu_compressed(const std::filesystem::path& path, const mesh::TetMesh& m, const std::vector<NodeField>& fields) {
    for (const NodeField& f : fields) {
        if (f.values.size() != m.nodes.size() * static_cast<std::size_t>(f.components)) throw Error("field '" + f.name + "' does not match the mesh");
    }
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    for (const auto& p : m.nodes) points->InsertNextPoint(p.x, p.y, p.z);
    vtkNew<vtkUnstructuredGrid> grid;
    grid->SetPoints(points);
    const int npe = m.nodes_per_element();
    // VTK's tetrahedron (type 10) and quadratic tetrahedron (type 24) number their nodes as OHS does: corners,
    // then the midsides of edges 01, 12, 20, 03, 13, 23.
    const int cell_type = npe == 4 ? 10 : 24;
    for (const auto& e : m.elems) {
        vtkIdType ids[10];
        for (int k = 0; k < npe; ++k) ids[k] = e[static_cast<std::size_t>(k)];
        grid->InsertNextCell(cell_type, npe, ids);
    }
    for (const NodeField& f : fields) {
        vtkNew<vtkDoubleArray> array;
        array->SetName(f.name.c_str());
        array->SetNumberOfComponents(f.components);
        array->SetNumberOfTuples(static_cast<vtkIdType>(m.nodes.size()));
        for (std::size_t i = 0; i < f.values.size(); ++i) array->SetValue(static_cast<vtkIdType>(i), f.values[i]);
        grid->GetPointData()->AddArray(array);
    }
    vtkNew<vtkXMLUnstructuredGridWriter> writer;
    writer->SetFileName(path.string().c_str());
    writer->SetInputData(grid);
    // Inline base64 (not appended) data: VTK 9.1 with newer libexpat cannot parse the appended-data block it wrote itself.
    writer->SetDataModeToBinary();
    writer->SetCompressorTypeToZLib();
    if (writer->Write() != 1) throw Error("VTK could not write " + path.string());
}

VtkSummary inspect_with_vtk(const std::filesystem::path& path) {
    VtkSummary out;
    vtkNew<vtkXMLUnstructuredGridReader> reader;
    if (!reader->CanReadFile(path.string().c_str())) {
        out.error = "VTK cannot read " + path.string();
        return out;
    }
    reader->SetFileName(path.string().c_str());
    reader->Update();
    vtkUnstructuredGrid* grid = reader->GetOutput();
    if (!grid || grid->GetNumberOfPoints() == 0) {
        out.error = "VTK read no points from " + path.string();
        return out;
    }
    out.points = static_cast<std::size_t>(grid->GetNumberOfPoints());
    out.cells = static_cast<std::size_t>(grid->GetNumberOfCells());
    out.cell_type = out.cells > 0 ? grid->GetCellType(0) : 0;
    vtkPointData* data = grid->GetPointData();
    for (int i = 0; i < data->GetNumberOfArrays(); ++i) out.arrays.emplace_back(data->GetArrayName(i));
    grid->GetBounds(out.bounds);
    out.ok = true;
    return out;
}

#else

void write_vtu_compressed(const std::filesystem::path&, const mesh::TetMesh&, const std::vector<NodeField>&) {
    throw Error("this build of ohs has no VTK support; install libvtk9-dev and reconfigure");
}

VtkSummary inspect_with_vtk(const std::filesystem::path&) {
    throw Error("this build of ohs has no VTK support; install libvtk9-dev and reconfigure");
}

#endif

}  // namespace ohs::fea
