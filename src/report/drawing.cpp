// SPDX-License-Identifier: Apache-2.0
#include "report/drawing.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

#include "core/error.hpp"

namespace ohs::report {

const char* layer_name(Layer l) {
    switch (l) {
        case Layer::Outline: return "OUTLINE";
        case Layer::Dimension: return "DIMENSION";
        case Layer::Centre: return "CENTRE";
        case Layer::Text: return "TEXT";
        case Layer::Hidden: return "HIDDEN";
        case Layer::Axis: return "AXIS";
        case Layer::Data0: return "DATA0";
        case Layer::Data1: return "DATA1";
        case Layer::Data2: return "DATA2";
        case Layer::Data3: return "DATA3";
    }
    return "OUTLINE";
}

void Drawing::line(Pt a, Pt b, Layer l) {
    Primitive p;
    p.kind = Primitive::Kind::Line;
    p.layer = l;
    p.points = {a, b};
    items.push_back(std::move(p));
}

void Drawing::polyline(std::vector<Pt> pts, bool closed, Layer l) {
    Primitive p;
    p.kind = Primitive::Kind::Polyline;
    p.layer = l;
    p.points = std::move(pts);
    p.closed = closed;
    items.push_back(std::move(p));
}

void Drawing::circle(Pt c, double r, Layer l) {
    Primitive p;
    p.kind = Primitive::Kind::Circle;
    p.layer = l;
    p.points = {c};
    p.radius = r;
    items.push_back(std::move(p));
}

void Drawing::text(Pt at, std::string s, double height, Layer l, int anchor) {
    Primitive p;
    p.kind = Primitive::Kind::Text;
    p.layer = l;
    p.points = {at};
    p.height = height;
    p.text = std::move(s);
    p.anchor = anchor;
    items.push_back(std::move(p));
}

void Drawing::bounds(Pt& lo, Pt& hi) const {
    lo = {1e300, 1e300};
    hi = {-1e300, -1e300};
    auto add = [&](double x, double y) {
        lo.x = std::min(lo.x, x);
        lo.y = std::min(lo.y, y);
        hi.x = std::max(hi.x, x);
        hi.y = std::max(hi.y, y);
    };
    for (const Primitive& p : items) {
        if (p.kind == Primitive::Kind::Circle) {
            add(p.points[0].x - p.radius, p.points[0].y - p.radius);
            add(p.points[0].x + p.radius, p.points[0].y + p.radius);
        } else if (p.kind == Primitive::Kind::Text) {
            const double w = 0.55 * p.height * static_cast<double>(p.text.size());
            const double x0 = p.anchor == 0 ? p.points[0].x : (p.anchor == 1 ? p.points[0].x - w / 2 : p.points[0].x - w);
            add(x0, p.points[0].y - 0.2 * p.height);
            add(x0 + w, p.points[0].y + p.height);
        } else {
            for (const Pt& q : p.points) add(q.x, q.y);
        }
    }
    if (items.empty()) lo = hi = {0, 0};
}

namespace {

std::string escape_xml(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            default: out += c;
        }
    }
    return out;
}

const char* css_class(Layer l) {
    switch (l) {
        case Layer::Outline: return "outline";
        case Layer::Dimension: return "dim";
        case Layer::Centre: return "centre";
        case Layer::Text: return "txt";
        case Layer::Hidden: return "hidden";
        case Layer::Axis: return "axis";
        case Layer::Data0: return "d0";
        case Layer::Data1: return "d1";
        case Layer::Data2: return "d2";
        case Layer::Data3: return "d3";
    }
    return "outline";
}

double nice_step(double range, int target) {
    const double raw = range / target;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double f = raw / mag;
    return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * mag;
}

std::string number(double v) {
    std::ostringstream os;
    os << std::setprecision(6) << v;
    return os.str();
}

}  // namespace

std::string to_svg(const Drawing& d, double margin) {
    Pt lo, hi;
    d.bounds(lo, hi);
    const double w = hi.x - lo.x + 2 * margin, h = hi.y - lo.y + 2 * margin;
    auto X = [&](double x) { return x - lo.x + margin; };
    auto Y = [&](double y) { return hi.y - y + margin; };
    std::ostringstream os;
    os << std::fixed << std::setprecision(4);
    os << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << w << ' ' << h << "\" width=\"" << w << "mm\" height=\"" << h << "mm\">\n";
    if (!d.title.empty()) os << "<title>" << escape_xml(d.title) << "</title>\n";
    os << "<style>\n"
          "svg{color:#1d1d1f;--dim:#1f6fbf;--d0:#1f6fbf;--d1:#c2501f;--d2:#2f8f55;--d3:#7c55c0;--grey:#8a8a8a}\n"
          "@media (prefers-color-scheme:dark){svg{color:#ececec;--dim:#6aaaf0;--d0:#6aaaf0;--d1:#f08a5c;--d2:#5cc88a;--d3:#b394ee;--grey:#9a9a9a}}\n"
          ".outline{stroke:currentColor;stroke-width:.25;fill:none;stroke-linejoin:round}\n"
          ".dim{stroke:var(--dim);stroke-width:.15;fill:none}text.dim{fill:var(--dim);stroke:none}\n"
          ".centre{stroke:var(--grey);stroke-width:.12;fill:none;stroke-dasharray:3 1 .5 1}\n"
          ".hidden{stroke:var(--grey);stroke-width:.15;fill:none;stroke-dasharray:1.5 1}\n"
          ".axis{stroke:var(--grey);stroke-width:.2;fill:none}text.axis{fill:currentColor;stroke:none}\n"
          ".d0{stroke:var(--d0);stroke-width:.45;fill:none}.d1{stroke:var(--d1);stroke-width:.45;fill:none}\n"
          ".d2{stroke:var(--d2);stroke-width:.45;fill:none}.d3{stroke:var(--d3);stroke-width:.45;fill:none}\n"
          "text{font-family:sans-serif;fill:currentColor;stroke:none}\n</style>\n";
    for (const Primitive& p : d.items) {
        const char* cls = css_class(p.layer);
        switch (p.kind) {
            case Primitive::Kind::Line:
                os << "<line class=\"" << cls << "\" x1=\"" << X(p.points[0].x) << "\" y1=\"" << Y(p.points[0].y) << "\" x2=\"" << X(p.points[1].x)
                   << "\" y2=\"" << Y(p.points[1].y) << "\"/>\n";
                break;
            case Primitive::Kind::Polyline:
                os << '<' << (p.closed ? "polygon" : "polyline") << " class=\"" << cls << "\" points=\"";
                for (const Pt& q : p.points) os << X(q.x) << ',' << Y(q.y) << ' ';
                os << "\"/>\n";
                break;
            case Primitive::Kind::Circle:
                os << "<circle class=\"" << cls << "\" cx=\"" << X(p.points[0].x) << "\" cy=\"" << Y(p.points[0].y) << "\" r=\"" << p.radius << "\"/>\n";
                break;
            case Primitive::Kind::Text:
                os << "<text class=\"" << cls << "\" x=\"" << X(p.points[0].x) << "\" y=\"" << Y(p.points[0].y) << "\" font-size=\"" << p.height
                   << "\" text-anchor=\"" << (p.anchor == 0 ? "start" : p.anchor == 1 ? "middle" : "end") << "\">" << escape_xml(p.text) << "</text>\n";
                break;
        }
    }
    os << "</svg>\n";
    return os.str();
}

std::string to_dxf(const Drawing& d) {
    std::ostringstream os;
    os << std::setprecision(8);
    auto pair = [&](int code, const auto& value) { os << code << '\n' << value << '\n'; };
    pair(0, "SECTION");
    pair(2, "HEADER");
    pair(9, "$INSUNITS");
    pair(70, 4);  // millimetres
    pair(0, "ENDSEC");
    pair(0, "SECTION");
    pair(2, "ENTITIES");
    for (const Primitive& p : d.items) {
        const char* layer = layer_name(p.layer);
        switch (p.kind) {
            case Primitive::Kind::Line:
                pair(0, "LINE");
                pair(8, layer);
                pair(10, p.points[0].x);
                pair(20, p.points[0].y);
                pair(30, 0);
                pair(11, p.points[1].x);
                pair(21, p.points[1].y);
                pair(31, 0);
                break;
            case Primitive::Kind::Polyline:
                pair(0, "POLYLINE");
                pair(8, layer);
                pair(66, 1);
                pair(70, p.closed ? 1 : 0);
                for (const Pt& q : p.points) {
                    pair(0, "VERTEX");
                    pair(8, layer);
                    pair(10, q.x);
                    pair(20, q.y);
                    pair(30, 0);
                }
                pair(0, "SEQEND");
                pair(8, layer);
                break;
            case Primitive::Kind::Circle:
                pair(0, "CIRCLE");
                pair(8, layer);
                pair(10, p.points[0].x);
                pair(20, p.points[0].y);
                pair(30, 0);
                pair(40, p.radius);
                break;
            case Primitive::Kind::Text:
                pair(0, "TEXT");
                pair(8, layer);
                pair(10, p.points[0].x);
                pair(20, p.points[0].y);
                pair(30, 0);
                pair(40, p.height);
                pair(1, p.text);
                if (p.anchor != 0) {
                    pair(72, p.anchor == 1 ? 1 : 2);
                    pair(11, p.points[0].x);
                    pair(21, p.points[0].y);
                    pair(31, 0);
                }
                break;
        }
    }
    pair(0, "ENDSEC");
    pair(0, "EOF");
    return os.str();
}

DxfSummary summarize_dxf(const std::string& text) {
    std::istringstream in(text);
    std::string code_line, value;
    DxfSummary s;
    std::set<std::string> layers;
    bool saw_section = false, saw_eof = false, in_polyline = false, first_vertex_seen = false;
    std::string entity;
    while (std::getline(in, code_line) && std::getline(in, value)) {
        auto trim = [](std::string& v) {
            while (!v.empty() && (v.back() == '\r' || v.back() == ' ')) v.pop_back();
            while (!v.empty() && v.front() == ' ') v.erase(v.begin());
        };
        trim(code_line);
        trim(value);
        int code = 0;
        try {
            code = std::stoi(code_line);
        } catch (const std::exception&) {
            throw Error("not a DXF file: unreadable group code '" + code_line + "'");
        }
        if (code == 0) {
            entity = value;
            if (value == "SECTION") saw_section = true;
            if (value == "EOF") saw_eof = true;
            if (value == "POLYLINE") {
                ++s.polylines;
                in_polyline = true;
                first_vertex_seen = false;
            } else if (value == "VERTEX") {
                ++s.vertices;
                first_vertex_seen = true;
            } else if (value == "SEQEND") {
                in_polyline = false;
            } else if (value == "LINE") {
                ++s.lines;
            } else if (value == "CIRCLE") {
                ++s.circles;
            } else if (value == "TEXT") {
                ++s.texts;
            }
        } else if (code == 8) {
            layers.insert(value);
        } else if (code == 70 && entity == "POLYLINE" && in_polyline && !first_vertex_seen) {
            if (std::stoi(value) & 1) ++s.closed_polylines;
        }
    }
    if (!saw_section || !saw_eof) throw Error("not a DXF file: missing SECTION or EOF");
    s.layers.assign(layers.begin(), layers.end());
    return s;
}

Drawing line_chart(const std::string& title, const std::string& x_label, const std::string& y_label, const std::vector<Series>& series,
                   double width, double height) {
    Drawing d;
    d.title = title;
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const Series& s : series) {
        for (const Pt& p : s.points) {
            x0 = std::min(x0, p.x);
            x1 = std::max(x1, p.x);
            y0 = std::min(y0, p.y);
            y1 = std::max(y1, p.y);
        }
    }
    if (x0 > x1) {
        x0 = 0;
        x1 = 1;
        y0 = 0;
        y1 = 1;
    }
    if (x1 - x0 < 1e-12) x1 = x0 + 1;
    if (y1 - y0 < 1e-12) {
        y0 -= 0.5;
        y1 += 0.5;
    }
    const double pad = 0.05 * (y1 - y0);
    const bool non_negative = y0 >= 0;
    y0 -= pad;
    y1 += pad;
    if (non_negative) y0 = std::max(y0, 0.0);  // no ticks below zero for quantities that cannot be negative
    const double sx = nice_step(x1 - x0, 6), sy = nice_step(y1 - y0, 5);
    x0 = std::floor(x0 / sx) * sx;
    x1 = std::ceil(x1 / sx) * sx;
    y0 = std::floor(y0 / sy) * sy;
    y1 = std::ceil(y1 / sy) * sy;

    const double left = 20, bottom = 14, top = 9, right = 4;
    const double pw = width - left - right, ph = height - bottom - top;
    auto X = [&](double x) { return left + (x - x0) / (x1 - x0) * pw; };
    auto Y = [&](double y) { return bottom + (y - y0) / (y1 - y0) * ph; };

    d.text({left + pw / 2, height - 4.2}, title, 3.4, Layer::Text, 1);
    d.polyline({{left, bottom}, {left + pw, bottom}, {left + pw, bottom + ph}, {left, bottom + ph}}, true, Layer::Axis);
    for (double x = std::ceil(x0 / sx) * sx; x <= x1 + 1e-9 * sx; x += sx) {
        d.line({X(x), bottom}, {X(x), bottom - 1.2}, Layer::Axis);
        d.text({X(x), bottom - 5}, number(x), 2.6, Layer::Axis, 1);
    }
    for (double y = std::ceil(y0 / sy) * sy; y <= y1 + 1e-9 * sy; y += sy) {
        d.line({left, Y(y)}, {left - 1.2, Y(y)}, Layer::Axis);
        d.text({left - 2.2, Y(y) - 0.9}, number(y), 2.6, Layer::Axis, 2);
    }
    d.text({left + pw / 2, 1.5}, x_label, 2.8, Layer::Axis, 1);
    d.text({left, bottom + ph + 1.6}, y_label, 2.6, Layer::Axis, 0);
    const Layer layers[] = {Layer::Data0, Layer::Data1, Layer::Data2, Layer::Data3};
    for (std::size_t k = 0; k < series.size(); ++k) {
        std::vector<Pt> pts;
        for (const Pt& p : series[k].points) pts.push_back({X(p.x), Y(p.y)});
        const Layer l = layers[k % 4];
        if (pts.size() > 1) d.polyline(pts, false, l);
        for (const Pt& p : pts) d.circle(p, 0.5, l);
        const double ly = bottom + ph - 3.5 - 3.5 * static_cast<double>(k);
        d.line({left + pw - 38, ly + 1}, {left + pw - 32, ly + 1}, l);
        d.text({left + pw - 30.5, ly}, series[k].name, 2.6, Layer::Text, 0);
    }
    return d;
}

}  // namespace ohs::report
