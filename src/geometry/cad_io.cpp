// SPDX-License-Identifier: Apache-2.0
#include "geometry/cad_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <sstream>
#include <string>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeFix_Face.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "core/error.hpp"

namespace ohs::geometry {

namespace {

void number_duplicates(std::vector<ImportedPart>& parts) {
    std::map<std::string, int> seen;
    for (ImportedPart& p : parts) {
        const int n = ++seen[p.name];
        if (n > 1) p.name += "#" + std::to_string(n);
    }
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// ---- DXF ---------------------------------------------------------------------------------------------

struct Entity {
    std::string type;
    std::vector<std::pair<int, std::string>> codes;

    // First value with this group code, or `fallback`.
    double number(int code, double fallback = 0.0) const {
        for (const auto& [c, v] : codes) {
            if (c == code) return std::stod(v);
        }
        return fallback;
    }
};

struct Vertex2 {
    double x{}, y{}, bulge{};
};

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

double unit_scale(int insunits) {
    switch (insunits) {
        case 1: return 25.4;       // inches
        case 2: return 304.8;      // feet
        case 5: return 10.0;       // centimetres
        case 6: return 1000.0;     // metres
        default: return 1.0;       // 0 (unitless), 4 (millimetres): taken as millimetres
    }
}

TopoDS_Edge line_edge(double x0, double y0, double x1, double y1) {
    if (std::hypot(x1 - x0, y1 - y0) < 1e-9) return {};
    return BRepBuilderAPI_MakeEdge(gp_Pnt(x0, y0, 0), gp_Pnt(x1, y1, 0)).Edge();
}

// Segment from (x0, y0) to (x1, y1) with a DXF bulge b = tan(included angle / 4); positive is counter-clockwise.
TopoDS_Edge bulge_edge(double x0, double y0, double x1, double y1, double b) {
    if (std::abs(b) < 1e-12) return line_edge(x0, y0, x1, y1);
    const double len = std::hypot(x1 - x0, y1 - y0);
    if (len < 1e-9) return {};
    const double sagitta = b * len / 2.0;
    const double mx = (x0 + x1) / 2.0 + (y1 - y0) / len * sagitta;  // the arc bulges to the right of p0 -> p1 for b > 0
    const double my = (y0 + y1) / 2.0 - (x1 - x0) / len * sagitta;
    GC_MakeArcOfCircle arc(gp_Pnt(x0, y0, 0), gp_Pnt(mx, my, 0), gp_Pnt(x1, y1, 0));
    if (!arc.IsDone()) throw Error("DXF: cannot build an arc from a polyline bulge");
    return BRepBuilderAPI_MakeEdge(arc.Value()).Edge();
}

void add_polyline(const std::vector<Vertex2>& v, bool closed, double scale, std::vector<TopoDS_Shape>& edges) {
    const std::size_t n = v.size();
    for (std::size_t i = 0; i + (closed ? 0 : 1) < n; ++i) {
        const Vertex2& a = v[i];
        const Vertex2& b = v[(i + 1) % n];
        const TopoDS_Edge e = bulge_edge(a.x * scale, a.y * scale, b.x * scale, b.y * scale, a.bulge);
        if (!e.IsNull()) edges.push_back(e);
    }
}

double wire_area(const TopoDS_Wire& w) {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY()), w, true).Face(), props);
    return std::abs(props.Mass());
}

}  // namespace

std::vector<ImportedPart> import_dxf(const std::filesystem::path& path, double default_thickness) {
    std::ifstream in(path);
    if (!in) throw Error("file not found: " + path.string());

    // Group code / value pairs.
    std::vector<std::pair<int, std::string>> pairs;
    std::string code_line, value_line;
    while (std::getline(in, code_line)) {
        if (!std::getline(in, value_line)) break;
        try {
            std::size_t used = 0;
            const int code = std::stoi(code_line, &used);
            if (trim(code_line.substr(used)).size()) throw std::invalid_argument("trailing text");
            pairs.emplace_back(code, trim(value_line));
        } catch (const std::exception&) {
            throw Error("not a DXF file: unreadable group code '" + trim(code_line) + "'");
        }
    }

    // Units from the header, entities from the ENTITIES section (or the whole file when it has no sections).
    double scale = 1.0;
    std::vector<Entity> entities;
    std::string section;
    bool any_section = false;
    for (std::size_t i = 0; i < pairs.size(); ++i) {
        const auto& [code, value] = pairs[i];
        if (code == 9 && value == "$INSUNITS" && i + 1 < pairs.size()) {
            scale = unit_scale(std::stoi(pairs[i + 1].second));
        } else if (code == 0 && value == "SECTION") {
            any_section = true;
            section = i + 1 < pairs.size() && pairs[i + 1].first == 2 ? pairs[i + 1].second : "";
        } else if (code == 0 && value == "ENDSEC") {
            section.clear();
        } else if (code == 0 && (section == "ENTITIES" || !any_section)) {
            entities.push_back({value, {}});
        } else if (!entities.empty() && (section == "ENTITIES" || !any_section)) {
            entities.back().codes.emplace_back(code, value);
        }
    }

    std::vector<TopoDS_Shape> edges;
    double thickness = 0.0;
    for (std::size_t i = 0; i < entities.size(); ++i) {
        const Entity& e = entities[i];
        if (const double t = e.number(39); t != 0.0 && thickness == 0.0) thickness = std::abs(t) * scale;
        if (e.type == "LINE") {
            const TopoDS_Edge edge = line_edge(e.number(10) * scale, e.number(20) * scale, e.number(11) * scale, e.number(21) * scale);
            if (!edge.IsNull()) edges.push_back(edge);
        } else if (e.type == "CIRCLE") {
            const double r = e.number(40) * scale;
            if (r <= 0) throw Error("DXF: circle without a radius");
            edges.push_back(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(e.number(10) * scale, e.number(20) * scale, 0), gp::DZ()), r)).Edge());
        } else if (e.type == "ARC") {
            const double r = e.number(40) * scale;
            if (r <= 0) throw Error("DXF: arc without a radius");
            const double a0 = e.number(50) * std::numbers::pi / 180.0;
            double a1 = e.number(51) * std::numbers::pi / 180.0;
            while (a1 <= a0) a1 += 2.0 * std::numbers::pi;
            edges.push_back(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(e.number(10) * scale, e.number(20) * scale, 0), gp::DZ()), r), a0, a1).Edge());
        } else if (e.type == "LWPOLYLINE") {
            std::vector<Vertex2> v;
            for (const auto& [code, value] : e.codes) {
                if (code == 10) v.push_back({std::stod(value), 0, 0});
                else if (code == 20 && !v.empty()) v.back().y = std::stod(value);
                else if (code == 42 && !v.empty()) v.back().bulge = std::stod(value);
            }
            add_polyline(v, (static_cast<int>(e.number(70)) & 1) != 0, scale, edges);
        } else if (e.type == "POLYLINE") {
            std::vector<Vertex2> v;
            std::size_t j = i + 1;
            for (; j < entities.size() && entities[j].type == "VERTEX"; ++j) {
                v.push_back({entities[j].number(10), entities[j].number(20), entities[j].number(42)});
            }
            add_polyline(v, (static_cast<int>(e.number(70)) & 1) != 0, scale, edges);
            i = j - 1;
        }
    }
    if (edges.empty()) throw Error("DXF: no lines, arcs, circles or polylines in " + path.string());

    // Chain the edges into contours.
    Handle(TopTools_HSequenceOfShape) edge_seq = new TopTools_HSequenceOfShape();
    for (const TopoDS_Shape& e : edges) edge_seq->Append(e);
    Handle(TopTools_HSequenceOfShape) wire_seq;
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edge_seq, 1e-3, false, wire_seq);
    std::vector<TopoDS_Wire> wires;
    for (int k = 1; k <= wire_seq->Length(); ++k) {
        const TopoDS_Wire w = TopoDS::Wire(wire_seq->Value(k));
        if (!BRep_Tool::IsClosed(w)) throw Error("DXF: an open contour in " + path.string() + "; close it in the drawing");
        wires.push_back(w);
    }

    // Nesting: a contour inside another is a hole in it; one inside a hole is a new part.
    const std::size_t n = wires.size();
    std::vector<double> area(n);
    std::vector<TopoDS_Face> faces(n);
    for (std::size_t k = 0; k < n; ++k) {
        area[k] = wire_area(wires[k]);
        faces[k] = BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY()), wires[k], true).Face();
    }
    auto inside = [&](std::size_t inner, std::size_t outer) {
        if (area[inner] >= area[outer]) return false;
        TopExp_Explorer ex(wires[inner], TopAbs_VERTEX);
        const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(ex.Current()));
        BRepClass_FaceClassifier cls(faces[outer], p, 1e-6);
        return cls.State() == TopAbs_IN;
    };
    std::vector<int> depth(n, 0);
    std::vector<int> parent(n, -1);
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t j = 0; j < n; ++j) {
            if (j == k || !inside(k, j)) continue;
            ++depth[k];
            if (parent[k] < 0 || area[j] < area[static_cast<std::size_t>(parent[k])]) parent[k] = static_cast<int>(j);
        }
    }

    const double t = thickness > 0 ? thickness : default_thickness;
    if (t <= 0) throw Error("DXF: the thickness must be positive");
    std::vector<ImportedPart> parts;
    for (std::size_t k = 0; k < n; ++k) {
        if (depth[k] % 2 != 0) continue;
        BRepBuilderAPI_MakeFace mf(gp_Pln(gp::XOY()), wires[k], true);
        for (std::size_t h = 0; h < n; ++h) {
            if (depth[h] % 2 == 1 && parent[h] == static_cast<int>(k)) mf.Add(wires[h]);
        }
        ShapeFix_Face fix(mf.Face());
        fix.Perform();
        fix.FixOrientation();
        parts.push_back({"profile", BRepPrimAPI_MakePrism(fix.Face(), gp_Vec(0, 0, t)).Shape()});
    }
    if (parts.empty()) throw Error("DXF: no closed contour in " + path.string());
    number_duplicates(parts);
    return parts;
}

// ---- IGES --------------------------------------------------------------------------------------------

std::vector<ImportedPart> import_iges(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) throw Error("file not found: " + path.string());
    IGESControl_Controller::Init();
    IGESControl_Reader reader;
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone) throw Error("cannot read IGES file: " + path.string());
    reader.TransferRoots();

    std::vector<ImportedPart> parts;
    for (int i = 1; i <= reader.NbShapes(); ++i) {
        const TopoDS_Shape root = reader.Shape(i);
        if (root.IsNull()) continue;
        bool any_solid = false;
        for (TopExp_Explorer ex(root, TopAbs_SOLID); ex.More(); ex.Next()) {
            parts.push_back({"part", ex.Current()});
            any_solid = true;
        }
        if (!any_solid) parts.push_back({"part", root});
    }
    if (parts.empty()) throw Error("no geometry found in " + path.string());
    number_duplicates(parts);
    return parts;
}

void export_iges(const std::vector<ExportPart>& parts, const std::filesystem::path& path) {
    IGESControl_Controller::Init();
    IGESControl_Writer writer("MM", 0);
    for (const ExportPart& p : parts) {
        gp_Trsf rotation;
        rotation.SetRotation(gp_Ax1(gp::Origin(), gp::DZ()), p.rz);
        gp_Trsf translation;
        translation.SetTranslation(gp_Vec(p.dx, p.dy, p.dz));
        writer.AddShape(BRepBuilderAPI_Transform(p.shape, translation.Multiplied(rotation), true).Shape());
    }
    writer.ComputeModel();
    if (!writer.Write(path.string().c_str())) throw Error("cannot write IGES file: " + path.string());
}

std::vector<ImportedPart> import_cad(const std::filesystem::path& path) {
    const std::string ext = lower(path.extension().string());
    if (ext == ".dxf") return import_dxf(path);
    if (ext == ".iges" || ext == ".igs") return import_iges(path);
    if (ext == ".step" || ext == ".stp" || ext.empty()) return import_step(path);
    throw Error("unsupported CAD format '" + ext + "' (use .step, .stp, .iges, .igs or .dxf)");
}

}  // namespace ohs::geometry
