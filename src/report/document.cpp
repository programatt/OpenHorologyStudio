// SPDX-License-Identifier: Apache-2.0
#include "report/document.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ohs::report {

const char* status_word(Status s) {
    switch (s) {
        case Status::Pass: return "PASS";
        case Status::Warn: return "WARN";
        case Status::Fail: return "FAIL";
        case Status::None: return "";
    }
    return "";
}

void Document::heading(std::string text, int level) {
    Block b;
    b.kind = Block::Kind::Heading;
    b.level = level;
    b.text = std::move(text);
    blocks.push_back(std::move(b));
}

void Document::paragraph(std::string text) {
    Block b;
    b.kind = Block::Kind::Paragraph;
    b.text = std::move(text);
    blocks.push_back(std::move(b));
}

void Document::key_values(std::vector<std::pair<std::string, std::string>> pairs) {
    Block b;
    b.kind = Block::Kind::KeyValues;
    b.pairs = std::move(pairs);
    blocks.push_back(std::move(b));
}

void Document::table(std::vector<std::string> header, std::vector<std::vector<std::string>> rows, std::vector<Status> status) {
    Block b;
    b.kind = Block::Kind::Table;
    b.header = std::move(header);
    b.rows = std::move(rows);
    b.row_status = std::move(status);
    blocks.push_back(std::move(b));
}

void Document::figure(Drawing drawing, std::string caption) {
    Block b;
    b.kind = Block::Kind::Figure;
    b.figure = std::move(drawing);
    b.text = std::move(caption);
    blocks.push_back(std::move(b));
}

void Document::callout(Status status, std::string text) {
    Block b;
    b.kind = Block::Kind::Callout;
    b.status = status;
    b.text = std::move(text);
    blocks.push_back(std::move(b));
}

Status Document::overall() const {
    Status worst = Status::None;
    auto bump = [&](Status s) {
        if (static_cast<int>(s) > static_cast<int>(worst)) worst = s;
    };
    for (const Block& b : blocks) {
        if (b.kind == Block::Kind::Callout) bump(b.status);
        for (Status s : b.row_status) bump(s);
    }
    return worst;
}

// ---- HTML ---------------------------------------------------------------------------------------------

namespace {

std::string esc(const std::string& s) {
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

const char* status_class(Status s) {
    return s == Status::Pass ? "pass" : s == Status::Warn ? "warn" : s == Status::Fail ? "fail" : "none";
}

}  // namespace

std::string to_html(const Document& d) {
    std::ostringstream os;
    os << "<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
       << "<title>" << esc(d.title) << "</title>\n<style>\n"
       << ":root{color-scheme:light dark;--bg:#fbfbfa;--fg:#1d1d1f;--muted:#5f6368;--line:#d8d8d4;--zebra:#f2f2ef;--pass:#1f7a45;--warn:#9a6700;--fail:#b3261e}\n"
       << "@media (prefers-color-scheme:dark){:root{--bg:#161618;--fg:#ececec;--muted:#a0a4a8;--line:#3a3a3e;--zebra:#1f1f22;--pass:#5cc88a;--warn:#e0b04a;--fail:#f28b82}}\n"
       << "body{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0;padding:0 16px 48px}\n"
       << "main{max-width:880px;margin:0 auto}h1{font-size:1.7rem;margin:1.6rem 0 .2rem}h2{font-size:1.25rem;margin:2rem 0 .5rem;border-bottom:1px solid var(--line);padding-bottom:.2rem}\n"
       << "h3{font-size:1.05rem;margin:1.4rem 0 .4rem}p.sub{color:var(--muted);margin:0 0 1rem}table{border-collapse:collapse;width:100%;margin:.6rem 0;font-size:.92rem}\n"
       << "th,td{text-align:left;padding:.3rem .55rem;border-bottom:1px solid var(--line);vertical-align:top}th{font-weight:600}tbody tr:nth-child(even){background:var(--zebra)}\n"
       << "dl{display:grid;grid-template-columns:minmax(10rem,18rem) 1fr;gap:.2rem 1rem;margin:.6rem 0}dt{font-weight:600}dd{margin:0}\n"
       << ".badge{display:inline-block;font-weight:700;font-size:.78rem;border:1.5px solid currentColor;border-radius:4px;padding:0 .35rem}\n"
       << ".pass{color:var(--pass)}.warn{color:var(--warn)}.fail{color:var(--fail)}\n"
       << ".callout{border-left:4px solid currentColor;padding:.4rem .8rem;margin:.8rem 0;background:var(--zebra)}.callout span.t{color:var(--fg)}\n"
       << "figure{margin:1rem 0}figure svg{max-width:100%;height:auto}figcaption{color:var(--muted);font-size:.88rem}.notice{color:var(--muted);font-size:.85rem;margin-top:2.5rem}\n"
       << "@media (max-width:600px){dl{grid-template-columns:1fr}}\n</style>\n</head>\n<body>\n<main>\n";
    os << "<h1>" << esc(d.title) << "</h1>\n";
    if (!d.subtitle.empty()) os << "<p class=\"sub\">" << esc(d.subtitle) << "</p>\n";
    for (const Block& b : d.blocks) {
        switch (b.kind) {
            case Block::Kind::Heading: os << "<h" << b.level << '>' << esc(b.text) << "</h" << b.level << ">\n"; break;
            case Block::Kind::Paragraph: os << "<p>" << esc(b.text) << "</p>\n"; break;
            case Block::Kind::KeyValues:
                os << "<dl>\n";
                for (const auto& [k, v] : b.pairs) os << "<dt>" << esc(k) << "</dt><dd>" << esc(v) << "</dd>\n";
                os << "</dl>\n";
                break;
            case Block::Kind::Table: {
                const bool has_status = !b.row_status.empty();
                os << "<table>\n<thead><tr>";
                for (const auto& h : b.header) os << "<th>" << esc(h) << "</th>";
                if (has_status) os << "<th>Result</th>";
                os << "</tr></thead>\n<tbody>\n";
                for (std::size_t i = 0; i < b.rows.size(); ++i) {
                    os << "<tr>";
                    for (const auto& c : b.rows[i]) os << "<td>" << esc(c) << "</td>";
                    if (has_status && i < b.row_status.size()) {
                        const Status s = b.row_status[i];
                        os << "<td>" << (s == Status::None ? "" : std::string("<span class=\"badge ") + status_class(s) + "\">" + status_word(s) + "</span>") << "</td>";
                    }
                    os << "</tr>\n";
                }
                os << "</tbody>\n</table>\n";
                break;
            }
            case Block::Kind::Figure: os << "<figure>\n" << to_svg(b.figure, 2.0) << "<figcaption>" << esc(b.text) << "</figcaption>\n</figure>\n"; break;
            case Block::Kind::Callout:
                os << "<div class=\"callout " << status_class(b.status) << "\"><span class=\"badge\">" << status_word(b.status) << "</span> <span class=\"t\">" << esc(b.text)
                   << "</span></div>\n";
                break;
        }
    }
    if (!d.notice.empty()) os << "<p class=\"notice\">" << esc(d.notice) << "</p>\n";
    os << "</main>\n</body>\n</html>\n";
    return os.str();
}

// ---- PDF -----------------------------------------------------------------------------------------------------

std::string to_winansi(const std::string& s) {
    std::string out;
    for (std::size_t i = 0; i < s.size();) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c < 0x80) {
            out += static_cast<char>(c);
            ++i;
            continue;
        }
        unsigned cp = 0;
        int len = 1;
        if ((c & 0xE0) == 0xC0) {
            cp = c & 0x1F;
            len = 2;
        } else if ((c & 0xF0) == 0xE0) {
            cp = c & 0x0F;
            len = 3;
        } else if ((c & 0xF8) == 0xF0) {
            cp = c & 0x07;
            len = 4;
        }
        for (int k = 1; k < len && i + static_cast<std::size_t>(k) < s.size(); ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[i + static_cast<std::size_t>(k)]) & 0x3F);
        i += static_cast<std::size_t>(len);
        if (cp >= 0xA0 && cp <= 0xFF) out += static_cast<char>(cp);
        else switch (cp) {
            case 0x2013: out += '\x96'; break;
            case 0x2014: out += '\x97'; break;
            case 0x2018: out += '\x91'; break;
            case 0x2019: out += '\x92'; break;
            case 0x201C: out += '\x93'; break;
            case 0x201D: out += '\x94'; break;
            case 0x2022: out += '\x95'; break;
            case 0x2026: out += '\x85'; break;
            case 0x2265: out += ">="; break;
            case 0x2264: out += "<="; break;
            case 0x2248: out += "~"; break;
            case 0x2212: out += "-"; break;
            case 0x2192: out += "->"; break;
            case 0x03C0: out += "pi"; break;
            case 0x0394: out += "d"; break;
            case 0x03B1: out += "a"; break;
            case 0x03C3: out += "s"; break;
            default: out += '?';
        }
    }
    return out;
}

namespace {

std::string pdf_string(const std::string& utf8) {
    const std::string s = to_winansi(utf8);
    std::string out = "(";
    for (unsigned char c : s) {
        if (c == '(' || c == ')' || c == '\\') {
            out += '\\';
            out += static_cast<char>(c);
        } else if (c < 0x20 || c >= 0x7F) {
            std::ostringstream o;
            o << '\\' << std::oct << std::setw(3) << std::setfill('0') << static_cast<int>(c);
            out += o.str();
        } else {
            out += static_cast<char>(c);
        }
    }
    return out + ")";
}

// Approximate Helvetica advance widths in em.
double char_width(char c, bool bold) {
    double w;
    if (std::string("ijl.,:;'|!").find(c) != std::string::npos) w = 0.26;
    else if (std::string("ft rI()[]-").find(c) != std::string::npos) w = 0.32;
    else if (std::string("mwMW%@").find(c) != std::string::npos) w = 0.84;
    else if (c >= 'A' && c <= 'Z') w = 0.68;
    else if (c >= '0' && c <= '9') w = 0.556;
    else w = 0.54;
    return bold ? w * 1.07 : w;
}

double text_width(const std::string& utf8, double size, bool bold = false) {
    const std::string s = to_winansi(utf8);
    double w = 0;
    for (char c : s) w += char_width(c, bold);
    return w * size;
}

std::vector<std::string> wrap(const std::string& text, double size, double width, bool bold = false) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string para;
    // Honour explicit newlines, wrap words inside each.
    std::string all = text;
    std::size_t start = 0;
    while (start <= all.size()) {
        const std::size_t nl = all.find('\n', start);
        para = all.substr(start, nl == std::string::npos ? std::string::npos : nl - start);
        std::istringstream words(para);
        std::string word, line;
        while (words >> word) {
            const std::string candidate = line.empty() ? word : line + " " + word;
            if (!line.empty() && text_width(candidate, size, bold) > width) {
                lines.push_back(line);
                line = word;
            } else {
                line = candidate;
            }
        }
        lines.push_back(line);
        if (nl == std::string::npos) break;
        start = nl + 1;
    }
    return lines;
}

struct Rgb {
    double r, g, b;
};

Rgb layer_colour(Layer l) {
    switch (l) {
        case Layer::Dimension: return {0.12, 0.44, 0.75};
        case Layer::Centre:
        case Layer::Hidden:
        case Layer::Axis: return {0.45, 0.45, 0.45};
        case Layer::Data0: return {0.12, 0.44, 0.75};
        case Layer::Data1: return {0.76, 0.31, 0.12};
        case Layer::Data2: return {0.18, 0.56, 0.33};
        case Layer::Data3: return {0.49, 0.33, 0.75};
        default: return {0, 0, 0};
    }
}

class Pdf {
public:
    Pdf(double w, double h, double margin) : w_(w), h_(h), margin_(margin) { new_page(); }

    double width() const { return w_; }
    double height() const { return h_; }
    double margin() const { return margin_; }
    double y() const { return y_; }
    void set_y(double y) { y_ = y; }
    double content_width() const { return w_ - 2 * margin_; }

    void new_page() {
        pages_.emplace_back();
        y_ = h_ - margin_;
    }
    void ensure(double need) {
        if (y_ - need < margin_ + 18) new_page();
    }
    std::string& stream() { return pages_.back(); }

    void text(double x, double y, double size, bool bold, const std::string& s, Rgb c = {0, 0, 0}) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << c.r << ' ' << c.g << ' ' << c.b << " rg BT /" << (bold ? "F2" : "F1") << ' ' << size << " Tf " << x << ' ' << y << " Td "
          << pdf_string(s) << " Tj ET\n";
        stream() += o.str();
    }
    void line(double x0, double y0, double x1, double y1, double width = 0.5, Rgb c = {0.6, 0.6, 0.6}) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << c.r << ' ' << c.g << ' ' << c.b << " RG " << width << " w " << x0 << ' ' << y0 << " m " << x1 << ' ' << y1 << " l S\n";
        stream() += o.str();
    }
    void rect(double x, double y, double w, double h, Rgb c = {0, 0, 0}, double lw = 0.8) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << c.r << ' ' << c.g << ' ' << c.b << " RG " << lw << " w " << x << ' ' << y << ' ' << w << ' ' << h << " re S\n";
        stream() += o.str();
    }

    void emit_drawing(const Drawing& d, double ox, double oy, double scale) {
        Pt lo, hi;
        d.bounds(lo, hi);
        auto X = [&](double x) { return ox + (x - lo.x) * scale; };
        auto Y = [&](double y) { return oy + (y - lo.y) * scale; };
        for (const Primitive& p : d.items) {
            const Rgb c = layer_colour(p.layer);
            std::ostringstream o;
            o << std::fixed << std::setprecision(3);
            const bool dashed = p.layer == Layer::Centre || p.layer == Layer::Hidden;
            const double lw = (p.layer >= Layer::Data0 ? 0.45 : p.layer == Layer::Outline ? 0.35 : 0.18) * scale / 2.8346;
            o << c.r << ' ' << c.g << ' ' << c.b << " RG " << std::max(lw, 0.25) << " w " << (dashed ? "[3 2] 0 d " : "[] 0 d ");
            switch (p.kind) {
                case Primitive::Kind::Line:
                    o << X(p.points[0].x) << ' ' << Y(p.points[0].y) << " m " << X(p.points[1].x) << ' ' << Y(p.points[1].y) << " l S\n";
                    break;
                case Primitive::Kind::Polyline:
                    for (std::size_t i = 0; i < p.points.size(); ++i) o << X(p.points[i].x) << ' ' << Y(p.points[i].y) << (i == 0 ? " m " : " l ");
                    o << (p.closed ? "h S\n" : "S\n");
                    break;
                case Primitive::Kind::Circle: {
                    const double cx = X(p.points[0].x), cy = Y(p.points[0].y), r = p.radius * scale, k = 0.5523 * r;
                    o << cx + r << ' ' << cy << " m " << cx + r << ' ' << cy + k << ' ' << cx + k << ' ' << cy + r << ' ' << cx << ' ' << cy + r << " c " << cx - k << ' ' << cy + r
                      << ' ' << cx - r << ' ' << cy + k << ' ' << cx - r << ' ' << cy << " c " << cx - r << ' ' << cy - k << ' ' << cx - k << ' ' << cy - r << ' ' << cx << ' ' << cy - r
                      << " c " << cx + k << ' ' << cy - r << ' ' << cx + r << ' ' << cy - k << ' ' << cx + r << ' ' << cy << " c S\n";
                    break;
                }
                case Primitive::Kind::Text: {
                    const double size = p.height * scale;
                    const double wd = text_width(p.text, size);
                    const double x = X(p.points[0].x) - (p.anchor == 1 ? wd / 2 : p.anchor == 2 ? wd : 0);
                    stream() += o.str();
                    o.str("");
                    text(x, Y(p.points[0].y), size, false, p.text, p.layer == Layer::Dimension ? c : Rgb{0, 0, 0});
                    continue;
                }
            }
            stream() += o.str();
        }
    }

    std::string build(const std::string& title, const std::string& footer) {
        // Footer on every page.
        for (std::size_t i = 0; i < pages_.size(); ++i) {
            std::ostringstream o;
            o << "0.4 0.4 0.4 rg BT /F1 8 Tf " << margin_ << " 24 Td " << pdf_string(footer) << " Tj ET\n";
            o << "BT /F1 8 Tf " << w_ - margin_ - 40 << " 24 Td " << pdf_string("page " + std::to_string(i + 1) + " of " + std::to_string(pages_.size())) << " Tj ET\n";
            pages_[i] += o.str();
        }
        std::vector<std::string> objects;
        auto add = [&](std::string body) {
            objects.push_back(std::move(body));
            return static_cast<int>(objects.size());
        };
        const int catalog = add("");
        const int pages_obj = add("");
        const int f1 = add("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
        const int f2 = add("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>");
        std::vector<int> page_ids;
        for (const std::string& content : pages_) {
            std::ostringstream stream;
            stream << "<< /Length " << content.size() << " >>\nstream\n" << content << "endstream";
            const int c = add(stream.str());
            std::ostringstream page;
            page << "<< /Type /Page /Parent " << pages_obj << " 0 R /MediaBox [0 0 " << w_ << ' ' << h_ << "] /Resources << /Font << /F1 " << f1 << " 0 R /F2 " << f2
                 << " 0 R >> >> /Contents " << c << " 0 R >>";
            page_ids.push_back(add(page.str()));
        }
        const int info = add("<< /Title " + pdf_string(title) + " /Producer (Open Horology Studio) >>");
        objects[static_cast<std::size_t>(catalog - 1)] = "<< /Type /Catalog /Pages " + std::to_string(pages_obj) + " 0 R >>";
        {
            std::ostringstream kids;
            kids << "<< /Type /Pages /Count " << page_ids.size() << " /Kids [";
            for (int id : page_ids) kids << id << " 0 R ";
            kids << "] >>";
            objects[static_cast<std::size_t>(pages_obj - 1)] = kids.str();
        }
        std::string out = "%PDF-1.4\n";
        std::vector<std::size_t> offsets;
        for (std::size_t i = 0; i < objects.size(); ++i) {
            offsets.push_back(out.size());
            out += std::to_string(i + 1) + " 0 obj\n" + objects[i] + "\nendobj\n";
        }
        const std::size_t xref = out.size();
        out += "xref\n0 " + std::to_string(objects.size() + 1) + "\n0000000000 65535 f \n";
        for (std::size_t off : offsets) {
            std::ostringstream o;
            o << std::setw(10) << std::setfill('0') << off << " 00000 n \n";
            out += o.str();
        }
        out += "trailer\n<< /Size " + std::to_string(objects.size() + 1) + " /Root " + std::to_string(catalog) + " 0 R /Info " + std::to_string(info) + " 0 R >>\nstartxref\n" +
               std::to_string(xref) + "\n%%EOF\n";
        return out;
    }

private:
    double w_, h_, margin_, y_{};
    std::vector<std::string> pages_;
};

Rgb status_colour(Status s) {
    return s == Status::Pass ? Rgb{0.12, 0.48, 0.27} : s == Status::Warn ? Rgb{0.60, 0.40, 0.0} : s == Status::Fail ? Rgb{0.70, 0.15, 0.12} : Rgb{0, 0, 0};
}

}  // namespace

std::string to_pdf(const Document& d) {
    Pdf pdf(595, 842, 50);
    const double cw = pdf.content_width();
    pdf.text(pdf.margin(), pdf.y() - 18, 20, true, d.title);
    pdf.set_y(pdf.y() - 30);
    if (!d.subtitle.empty()) {
        for (const auto& l : wrap(d.subtitle, 10.5, cw)) {
            pdf.text(pdf.margin(), pdf.y() - 12, 10.5, false, l, {0.35, 0.35, 0.35});
            pdf.set_y(pdf.y() - 14);
        }
    }
    pdf.set_y(pdf.y() - 8);

    for (const Block& b : d.blocks) {
        switch (b.kind) {
            case Block::Kind::Heading: {
                const double size = b.level <= 1 ? 16 : b.level == 2 ? 13 : 11;
                pdf.ensure(size + 30);
                pdf.set_y(pdf.y() - (b.level <= 2 ? 14 : 8));
                pdf.text(pdf.margin(), pdf.y() - size, size, true, b.text);
                pdf.set_y(pdf.y() - size - 4);
                if (b.level == 2) {
                    pdf.line(pdf.margin(), pdf.y(), pdf.margin() + cw, pdf.y(), 0.5);
                    pdf.set_y(pdf.y() - 4);
                }
                break;
            }
            case Block::Kind::Paragraph:
                for (const auto& l : wrap(b.text, 10, cw)) {
                    pdf.ensure(14);
                    pdf.text(pdf.margin(), pdf.y() - 11, 10, false, l);
                    pdf.set_y(pdf.y() - 13.5);
                }
                pdf.set_y(pdf.y() - 4);
                break;
            case Block::Kind::KeyValues:
                for (const auto& [k, v] : b.pairs) {
                    const auto lines = wrap(v, 10, cw - 170);
                    pdf.ensure(13.5 * static_cast<double>(lines.size()) + 2);
                    pdf.text(pdf.margin(), pdf.y() - 11, 10, true, k);
                    for (const auto& l : lines) {
                        pdf.text(pdf.margin() + 170, pdf.y() - 11, 10, false, l);
                        pdf.set_y(pdf.y() - 13.5);
                    }
                }
                pdf.set_y(pdf.y() - 4);
                break;
            case Block::Kind::Table: {
                const bool has_status = !b.row_status.empty();
                const std::size_t cols = b.header.size() + (has_status ? 1 : 0);
                std::vector<double> natural(cols, 0.0);
                for (std::size_t c = 0; c < b.header.size(); ++c) {
                    natural[c] = text_width(b.header[c], 9, true);
                    for (const auto& row : b.rows) {
                        if (c < row.size()) natural[c] = std::max(natural[c], std::min(text_width(row[c], 9), 0.45 * cw));
                    }
                }
                if (has_status) natural[cols - 1] = 34;
                double total = 0;
                for (double w : natural) total += w + 8;
                const double k = total > cw ? cw / total : 1.0;
                std::vector<double> width(cols);
                for (std::size_t c = 0; c < cols; ++c) width[c] = (natural[c] + 8) * k;
                pdf.ensure(40);
                double x = pdf.margin();
                for (std::size_t c = 0; c < b.header.size(); ++c) {
                    pdf.text(x + 2, pdf.y() - 10, 9, true, b.header[c]);
                    x += width[c];
                }
                if (has_status) pdf.text(x + 2, pdf.y() - 10, 9, true, "Result");
                pdf.set_y(pdf.y() - 14);
                pdf.line(pdf.margin(), pdf.y(), pdf.margin() + cw, pdf.y(), 0.7, {0.3, 0.3, 0.3});
                for (std::size_t r = 0; r < b.rows.size(); ++r) {
                    std::vector<std::vector<std::string>> cells;
                    std::size_t lines = 1;
                    for (std::size_t c = 0; c < b.header.size(); ++c) {
                        cells.push_back(wrap(c < b.rows[r].size() ? b.rows[r][c] : "", 9, width[c] - 6));
                        lines = std::max(lines, cells.back().size());
                    }
                    pdf.ensure(11.5 * static_cast<double>(lines) + 4);
                    x = pdf.margin();
                    for (std::size_t c = 0; c < b.header.size(); ++c) {
                        double yy = pdf.y() - 10;
                        for (const auto& l : cells[c]) {
                            pdf.text(x + 2, yy, 9, false, l);
                            yy -= 11.5;
                        }
                        x += width[c];
                    }
                    if (has_status && r < b.row_status.size() && b.row_status[r] != Status::None) {
                        pdf.text(x + 2, pdf.y() - 10, 9, true, status_word(b.row_status[r]), status_colour(b.row_status[r]));
                    }
                    pdf.set_y(pdf.y() - 11.5 * static_cast<double>(lines) - 3);
                    pdf.line(pdf.margin(), pdf.y(), pdf.margin() + cw, pdf.y(), 0.25, {0.82, 0.82, 0.82});
                }
                pdf.set_y(pdf.y() - 6);
                break;
            }
            case Block::Kind::Figure: {
                Pt lo, hi;
                b.figure.bounds(lo, hi);
                const double dw = std::max(hi.x - lo.x, 1e-6), dh = std::max(hi.y - lo.y, 1e-6);
                const double scale = std::min(cw / dw, 300.0 / dh);
                pdf.ensure(dh * scale + 30);
                pdf.emit_drawing(b.figure, pdf.margin() + (cw - dw * scale) / 2, pdf.y() - dh * scale, scale);
                pdf.set_y(pdf.y() - dh * scale - 12);
                for (const auto& l : wrap(b.text, 8.5, cw)) {
                    pdf.text(pdf.margin(), pdf.y() - 8, 8.5, false, l, {0.35, 0.35, 0.35});
                    pdf.set_y(pdf.y() - 11);
                }
                pdf.set_y(pdf.y() - 6);
                break;
            }
            case Block::Kind::Callout: {
                const auto lines = wrap(b.text, 10, cw - 60);
                const double h = 13.5 * static_cast<double>(lines.size()) + 8;
                pdf.ensure(h + 6);
                pdf.rect(pdf.margin(), pdf.y() - h, cw, h, status_colour(b.status), 1.0);
                pdf.text(pdf.margin() + 6, pdf.y() - 14, 10, true, status_word(b.status), status_colour(b.status));
                double yy = pdf.y() - 14;
                for (const auto& l : lines) {
                    pdf.text(pdf.margin() + 52, yy, 10, false, l);
                    yy -= 13.5;
                }
                pdf.set_y(pdf.y() - h - 8);
                break;
            }
        }
    }
    if (!d.notice.empty()) {
        pdf.set_y(pdf.y() - 10);
        for (const auto& l : wrap(d.notice, 8.5, cw)) {
            pdf.ensure(12);
            pdf.text(pdf.margin(), pdf.y() - 9, 8.5, false, l, {0.4, 0.4, 0.4});
            pdf.set_y(pdf.y() - 11);
        }
    }
    return pdf.build(d.title, d.title);
}

std::string drawing_to_pdf(const Drawing& drawing, const std::string& title, const std::vector<std::string>& info) {
    Pt lo, hi;
    drawing.bounds(lo, hi);
    const double dw = std::max(hi.x - lo.x, 1e-6), dh = std::max(hi.y - lo.y, 1e-6);
    const bool landscape = dw > dh * 0.9;
    const double pw = landscape ? 842 : 595, ph = landscape ? 595 : 842, margin = 36;
    const double block_h = 22 + 12.0 * static_cast<double>(info.size()) + 10;
    const double avail_w = pw - 2 * margin, avail_h = ph - 2 * margin - block_h - 10;
    const double pt_per_mm = 72.0 / 25.4;
    double ratio = 0.25;
    for (double r : {50.0, 40.0, 20.0, 10.0, 5.0, 4.0, 2.0, 1.0, 0.5, 0.25}) {
        if (dw * r * pt_per_mm <= avail_w && dh * r * pt_per_mm <= avail_h) {
            ratio = r;
            break;
        }
    }
    Pdf pdf(pw, ph, margin);
    const double scale = ratio * pt_per_mm;
    pdf.emit_drawing(drawing, margin + (avail_w - dw * scale) / 2, margin + block_h + 10 + (avail_h - dh * scale) / 2, scale);
    pdf.rect(margin, margin, pw - 2 * margin, block_h, {0, 0, 0}, 0.8);
    pdf.text(margin + 8, margin + block_h - 16, 12, true, title);
    double yy = margin + block_h - 30;
    for (const auto& l : info) {
        pdf.text(margin + 8, yy, 9, false, l);
        yy -= 12;
    }
    std::ostringstream sc;
    sc << "scale " << (ratio >= 1 ? std::to_string(static_cast<int>(ratio)) + ":1" : "1:" + std::to_string(static_cast<int>(1 / ratio))) << ", dimensions in mm";
    pdf.text(pw - margin - 8 - text_width(sc.str(), 9), margin + 8, 9, false, sc.str());
    return pdf.build(title, "Open Horology Studio");
}

}  // namespace ohs::report
