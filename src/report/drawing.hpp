// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

namespace ohs::report {

struct Pt {
    double x{};
    double y{};
};

enum class Layer { Outline, Dimension, Centre, Text, Hidden, Axis, Data0, Data1, Data2, Data3 };

const char* layer_name(Layer layer);

struct Primitive {
    enum class Kind { Line, Polyline, Circle, Text } kind{Kind::Line};
    Layer layer{Layer::Outline};
    std::vector<Pt> points;   ///< Line: 2 points; Polyline: n points; Circle: 1 point (centre); Text: 1 point (anchor)
    bool closed{false};       ///< Polyline
    double radius{};          ///< Circle
    double height{};          ///< Text height in drawing units
    std::string text;
    int anchor{0};            ///< Text: 0 left, 1 centre, 2 right
};

/// Vector drawing in millimetres, y up. The same description is written as SVG, DXF and PDF
/// vector content, so drawings and charts need one description only.
struct Drawing {
    std::string title;
    std::vector<Primitive> items;

    void line(Pt a, Pt b, Layer l = Layer::Outline);
    void polyline(std::vector<Pt> pts, bool closed, Layer l = Layer::Outline);
    void circle(Pt c, double r, Layer l = Layer::Outline);
    void text(Pt at, std::string s, double height, Layer l = Layer::Text, int anchor = 0);

    /// Bounding box over all geometry (circles by radius); text counts by its anchor.
    void bounds(Pt& lo, Pt& hi) const;
};

/// SVG with a viewBox fitted to the drawing plus `margin` mm; strokes use CSS classes per layer
/// and system colours that follow the page's light or dark scheme.
std::string to_svg(const Drawing& drawing, double margin = 4.0);

/// ASCII DXF (R12 entities: POLYLINE, LINE, CIRCLE, TEXT), one DXF layer per drawing layer,
/// units millimetres. Readable by every CAD and CAM tool.
std::string to_dxf(const Drawing& drawing);

struct DxfSummary {
    int polylines{};
    int lines{};
    int circles{};
    int texts{};
    int closed_polylines{};
    int vertices{};
    std::vector<std::string> layers;
};

/// Reads back the entities of an ASCII DXF written by to_dxf (and simple files from other tools).
/// Throws ohs::Error when the file is not a DXF.
DxfSummary summarize_dxf(const std::string& text);

/// Line chart: axes with ticks, a polyline per series and a legend, as a Drawing.
struct Series {
    std::string name;
    std::vector<Pt> points;
};
Drawing line_chart(const std::string& title, const std::string& x_label, const std::string& y_label,
                   const std::vector<Series>& series, double width = 160, double height = 80);

}  // namespace ohs::report
