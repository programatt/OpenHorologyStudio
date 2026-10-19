// SPDX-License-Identifier: Apache-2.0
#include "geometry/pmi.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include <BinXCAFDrivers.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDimTolObjects_DimensionObject.hxx>
#include <XCAFDimTolObjects_DimensionType.hxx>
#include <XCAFDoc_DimTolTool.hxx>
#include <XCAFDoc_Dimension.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "core/error.hpp"

namespace ohs::geometry {

namespace {

Handle(TDocStd_Application) make_application() {
    Handle(TDocStd_Application) app = new TDocStd_Application();
    BinXCAFDrivers::DefineFormat(app);
    return app;
}

std::string name_of(const TDF_Label& label) {
    Handle(TDataStd_Name) attr;
    if (label.FindAttribute(TDataStd_Name::GetID(), attr)) return TCollection_AsciiString(attr->Get()).ToCString();
    return {};
}

// The name of the part a shape label belongs to: its own name, else that of the product it refers to, else
// of the label above it (a face of a part).
std::string part_name(TDF_Label label) {
    for (int depth = 0; depth < 6 && !label.IsNull(); ++depth) {
        if (XCAFDoc_ShapeTool::IsReference(label)) {
            TDF_Label referred;
            if (XCAFDoc_ShapeTool::GetReferredShape(label, referred)) {
                if (const std::string n = name_of(label); !n.empty()) return n;
                label = referred;
            }
        }
        if (const std::string n = name_of(label); !n.empty()) return n;
        label = label.Father();
    }
    return {};
}

std::string kind_of(XCAFDimTolObjects_DimensionType t) {
    switch (t) {
        case XCAFDimTolObjects_DimensionType_Size_Diameter:
        case XCAFDimTolObjects_DimensionType_Size_SphericalDiameter:
        case XCAFDimTolObjects_DimensionType_Size_ToroidalMinorDiameter:
        case XCAFDimTolObjects_DimensionType_Size_ToroidalMajorDiameter: return "diameter";
        case XCAFDimTolObjects_DimensionType_Size_Radius:
        case XCAFDimTolObjects_DimensionType_Size_SphericalRadius: return "radius";
        case XCAFDimTolObjects_DimensionType_Size_CurveLength:
        case XCAFDimTolObjects_DimensionType_Size_Thickness:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromCenterToOuter:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromCenterToInner:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromOuterToCenter:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromOuterToOuter:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromOuterToInner:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromInnerToCenter:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromInnerToOuter:
        case XCAFDimTolObjects_DimensionType_Location_LinearDistance_FromInnerToInner: return "length";
        default: return "other";
    }
}

XCAFDimTolObjects_DimensionType type_of(const std::string& kind) {
    if (kind == "diameter") return XCAFDimTolObjects_DimensionType_Size_Diameter;
    if (kind == "radius") return XCAFDimTolObjects_DimensionType_Size_Radius;
    if (kind == "length") return XCAFDimTolObjects_DimensionType_Location_LinearDistance;
    throw Error("PMI dimension kind must be diameter, radius or length");
}

}  // namespace

std::vector<PmiDimension> read_step_pmi(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) throw Error("file not found: " + path.string());
    Handle(TDocStd_Application) app = make_application();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);

    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    reader.SetGDTMode(true);
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone) {
        app->Close(doc);
        throw Error("cannot read STEP file: " + path.string());
    }
    if (!reader.Transfer(doc)) {
        app->Close(doc);
        throw Error("STEP transfer failed: " + path.string());
    }

    std::vector<PmiDimension> out;
    Handle(XCAFDoc_DimTolTool) tool = XCAFDoc_DocumentTool::DimTolTool(doc->Main());
    TDF_LabelSequence labels;
    tool->GetDimensionLabels(labels);
    for (const TDF_Label& label : labels) {
        Handle(XCAFDoc_Dimension) attr;
        if (!label.FindAttribute(XCAFDoc_Dimension::GetID(), attr)) continue;
        const Handle(XCAFDimTolObjects_DimensionObject) obj = attr->GetObject();
        if (obj.IsNull()) continue;
        PmiDimension d;
        d.kind = kind_of(obj->GetType());
        d.nominal = obj->GetValue();
        d.plus = std::abs(obj->GetUpperTolValue());
        d.minus = std::abs(obj->GetLowerTolValue());
        TDF_LabelSequence first, second;
        if (tool->GetRefShapeLabel(label, first, second) && first.Length() > 0) d.part = part_name(first.First());
        out.push_back(std::move(d));
    }
    app->Close(doc);
    return out;
}

void export_step_with_pmi(const std::vector<ExportPart>& parts, const std::vector<PmiDimension>& dimensions,
                          const std::filesystem::path& path) {
    Handle(TDocStd_Application) app = make_application();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);
    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_DimTolTool) tolerances = XCAFDoc_DocumentTool::DimTolTool(doc->Main());

    const TDF_Label assembly = shapes->NewShape();
    TDataStd_Name::Set(assembly, TCollection_ExtendedString("movement"));
    std::map<std::string, TDF_Label> product_of;
    for (const ExportPart& p : parts) {
        const TDF_Label product = shapes->NewShape();
        shapes->SetShape(product, p.shape);
        TDataStd_Name::Set(product, TCollection_ExtendedString(p.name.c_str(), true));
        product_of[p.name] = product;
        gp_Trsf rotation;
        rotation.SetRotation(gp_Ax1(gp::Origin(), gp::DZ()), p.rz);
        gp_Trsf translation;
        translation.SetTranslation(gp_Vec(p.dx, p.dy, p.dz));
        const TDF_Label instance = shapes->AddComponent(assembly, product, TopLoc_Location(translation.Multiplied(rotation)));
        TDataStd_Name::Set(instance, TCollection_ExtendedString(p.name.c_str(), true));
    }
    shapes->UpdateAssemblies();

    for (const PmiDimension& d : dimensions) {
        const auto it = product_of.find(d.part);
        if (it == product_of.end()) throw Error("PMI dimension on '" + d.part + "', which is not one of the parts");
        Handle(XCAFDimTolObjects_DimensionObject) obj = new XCAFDimTolObjects_DimensionObject();
        obj->SetType(type_of(d.kind));
        obj->SetValue(d.nominal);
        obj->SetUpperTolValue(d.plus);
        obj->SetLowerTolValue(-d.minus);
        const TDF_Label dimension = tolerances->AddDimension();
        Handle(XCAFDoc_Dimension) attr = XCAFDoc_Dimension::Set(dimension);
        attr->SetObject(obj);
        // AP242 attaches a dimension to geometry, not to a whole product: use faces of the part (a diameter or
        // radius to the first face, a length between its first two).
        const ExportPart* owner = nullptr;
        for (const ExportPart& p : parts) {
            if (p.name == d.part) owner = &p;
        }
        std::vector<TopoDS_Face> faces;
        for (TopExp_Explorer ex(owner->shape, TopAbs_FACE); ex.More() && faces.size() < 2; ex.Next()) faces.push_back(TopoDS::Face(ex.Current()));
        if (faces.empty()) throw Error("part '" + d.part + "' has no face to carry a dimension");
        const TDF_Label first = shapes->AddSubShape(it->second, faces[0]);
        if (d.kind == "length" && faces.size() > 1) {
            tolerances->SetDimension(first, shapes->AddSubShape(it->second, faces[1]), dimension);
        } else {
            tolerances->SetDimension(first, dimension);
        }
    }

    STEPControl_Controller::Init();  // registers the write.step.schema parameter
    if (!Interface_Static::SetCVal("write.step.schema", "AP242DIS")) throw Error("this OpenCASCADE cannot write STEP AP242");
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    writer.SetDimTolMode(true);
    const bool ok = writer.Transfer(doc, STEPControl_AsIs) && writer.Write(path.string().c_str()) == IFSelect_RetDone;
    app->Close(doc);
    if (!ok) throw Error("cannot write STEP file: " + path.string());
}

}  // namespace ohs::geometry
