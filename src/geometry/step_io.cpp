// SPDX-License-Identifier: Apache-2.0
#include "geometry/step_io.hpp"

#include <map>

#include <BinXCAFDrivers.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <XCAFApp_Application.hxx>
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

std::string label_name(const TDF_Label& label) {
    Handle(TDataStd_Name) attr;
    if (label.FindAttribute(TDataStd_Name::GetID(), attr)) {
        return TCollection_AsciiString(attr->Get()).ToCString();
    }
    return {};
}

void collect(const TDF_Label& label, const TopLoc_Location& parent, std::vector<ImportedPart>& out) {
    TDF_Label referred = label;
    if (XCAFDoc_ShapeTool::IsReference(label)) XCAFDoc_ShapeTool::GetReferredShape(label, referred);
    const TopLoc_Location loc = parent * XCAFDoc_ShapeTool::GetLocation(label);

    if (XCAFDoc_ShapeTool::IsAssembly(referred)) {
        TDF_LabelSequence components;
        XCAFDoc_ShapeTool::GetComponents(referred, components);
        for (const TDF_Label& c : components) collect(c, loc, out);
    } else if (XCAFDoc_ShapeTool::IsSimpleShape(referred)) {
        std::string name = label_name(label);
        if (name.empty()) name = label_name(referred);
        if (name.empty()) name = "part";
        out.push_back({std::move(name), XCAFDoc_ShapeTool::GetShape(referred).Moved(loc)});
    }
}

}  // namespace

std::vector<ImportedPart> import_step(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) throw Error("file not found: " + path.string());

    Handle(TDocStd_Application) app = make_application();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);

    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone) {
        app->Close(doc);
        throw Error("cannot read STEP file: " + path.string());
    }
    if (!reader.Transfer(doc)) {
        app->Close(doc);
        throw Error("STEP transfer failed: " + path.string());
    }

    Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence roots;
    tool->GetFreeShapes(roots);

    std::vector<ImportedPart> parts;
    for (const TDF_Label& root : roots) collect(root, TopLoc_Location(), parts);
    app->Close(doc);

    std::map<std::string, int> seen;
    for (ImportedPart& p : parts) {
        const int n = ++seen[p.name];
        if (n > 1) p.name += "#" + std::to_string(n);
    }
    if (parts.empty()) throw Error("no parts found in " + path.string());
    return parts;
}

void export_step(const std::vector<ExportPart>& parts, const std::filesystem::path& path) {
    Handle(TDocStd_Application) app = make_application();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);
    Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

    const TDF_Label assembly = tool->NewShape();
    TDataStd_Name::Set(assembly, TCollection_ExtendedString("movement"));
    for (const ExportPart& p : parts) {
        const TDF_Label part = tool->NewShape();
        tool->SetShape(part, p.shape);
        TDataStd_Name::Set(part, TCollection_ExtendedString(p.name.c_str(), true));
        gp_Trsf rotation;
        rotation.SetRotation(gp_Ax1(gp::Origin(), gp::DZ()), p.rz);
        gp_Trsf translation;
        translation.SetTranslation(gp_Vec(p.dx, p.dy, p.dz));
        const TDF_Label instance =
            tool->AddComponent(assembly, part, TopLoc_Location(translation.Multiplied(rotation)));
        TDataStd_Name::Set(instance, TCollection_ExtendedString(p.name.c_str(), true));
    }
    tool->UpdateAssemblies();

    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    const bool ok = writer.Transfer(doc, STEPControl_AsIs) &&
                    writer.Write(path.string().c_str()) == IFSelect_RetDone;
    app->Close(doc);
    if (!ok) throw Error("cannot write STEP file: " + path.string());
}

}  // namespace ohs::geometry
