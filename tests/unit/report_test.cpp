// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "report/document.hpp"
#include "report/drawing.hpp"

using namespace ohs::report;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

Drawing sample() {
    Drawing d;
    d.title = "test <part> & more";
    d.polyline({{0, 0}, {10, 0}, {10, 5}, {0, 5}}, true);
    d.circle({5, 2.5}, 1.5);
    d.line({0, -2}, {10, -2}, Layer::Dimension);
    d.line({5, -1}, {5, 6}, Layer::Centre);
    d.text({5, -4}, "10 mm", 1.5, Layer::Dimension, 1);
    return d;
}

// Every xref entry must point at "<n> 0 obj", the trailer must point at the xref, and the page count must match.
void check_pdf_structure(const std::string& pdf, int expected_pages) {
    REQUIRE(pdf.rfind("%PDF-1.4", 0) == 0);
    REQUIRE(pdf.find("%%EOF") != std::string::npos);
    const std::size_t sx = pdf.rfind("startxref\n");
    REQUIRE(sx != std::string::npos);
    const std::size_t xref = static_cast<std::size_t>(std::stoul(pdf.substr(sx + 10)));
    REQUIRE(pdf.compare(xref, 4, "xref") == 0);
    std::istringstream in(pdf.substr(xref));
    std::string word;
    int first = 0, count = 0;
    in >> word >> first >> count;
    REQUIRE(first == 0);
    std::string line;
    std::getline(in, line);  // rest of "0 N"
    std::getline(in, line);  // free entry
    for (int i = 1; i < count; ++i) {
        std::getline(in, line);
        const std::size_t off = static_cast<std::size_t>(std::stoul(line.substr(0, 10)));
        REQUIRE(pdf.compare(off, std::to_string(i).size() + 6, std::to_string(i) + " 0 obj") == 0);
    }
    int pages = 0;
    for (std::size_t p = 0; (p = pdf.find("/Type /Page ", p)) != std::string::npos; ++p) ++pages;
    CHECK(pages == expected_pages);
}

}  // namespace

TEST_CASE("SVG of a drawing is well formed and escapes text") {
    const std::string svg = to_svg(sample());
    CHECK(svg.rfind("<svg", 0) == 0);
    CHECK_THAT(svg, ContainsSubstring("<polygon class=\"outline\""));
    CHECK_THAT(svg, ContainsSubstring("<circle class=\"outline\""));
    CHECK_THAT(svg, ContainsSubstring("test &lt;part&gt; &amp; more"));
    CHECK_THAT(svg, ContainsSubstring("prefers-color-scheme:dark"));
    CHECK_THAT(svg, ContainsSubstring("text-anchor=\"middle\""));
    // y is flipped: the top edge of the part (y = 5) has a smaller SVG y than the bottom (y = 0).
    std::smatch m;
    const std::regex poly("points=\"([^\"]*)\"");
    REQUIRE(std::regex_search(svg, m, poly));
    std::istringstream pts(m[1]);
    std::string first, second, third;
    pts >> first >> second >> third;
    const double y0 = std::stod(first.substr(first.find(',') + 1)), y2 = std::stod(third.substr(third.find(',') + 1));
    CHECK(y2 < y0);
    Pt lo, hi;
    sample().bounds(lo, hi);
    CHECK(lo.x == Approx(0.0));
    CHECK(hi.x == Approx(10.0));
    CHECK(lo.y < -3.5);  // the dimension text hangs below
}

TEST_CASE("DXF round trip keeps entities, closed flag and layers") {
    const std::string dxf = to_dxf(sample());
    const DxfSummary s = summarize_dxf(dxf);
    CHECK(s.polylines == 1);
    CHECK(s.closed_polylines == 1);
    CHECK(s.vertices == 4);
    CHECK(s.circles == 1);
    CHECK(s.lines == 2);
    CHECK(s.texts == 1);
    CHECK(std::find(s.layers.begin(), s.layers.end(), "DIMENSION") != s.layers.end());
    CHECK(std::find(s.layers.begin(), s.layers.end(), "OUTLINE") != s.layers.end());
    CHECK_THAT(dxf, ContainsSubstring("$INSUNITS\n70\n4"));
    CHECK_THROWS_AS(summarize_dxf("this is not a dxf"), ohs::Error);
    CHECK_THROWS_AS(summarize_dxf("0\nSECTION\n2\nENTITIES\n"), ohs::Error);
}

TEST_CASE("line chart has axes, a series polyline per series and a legend") {
    const Drawing d = line_chart("Amplitude", "hours", "deg", {{"dial up", {{0, 270}, {25, 250}, {50, 225}}}, {"crown up", {{0, 250}, {50, 200}}}});
    int polylines = 0, circles = 0;
    bool legend = false;
    for (const auto& p : d.items) {
        polylines += p.kind == Primitive::Kind::Polyline && p.layer >= Layer::Data0;
        circles += p.kind == Primitive::Kind::Circle;
        legend |= p.kind == Primitive::Kind::Text && p.text == "crown up";
    }
    CHECK(polylines == 2);
    CHECK(circles == 5);
    CHECK(legend);
    // Empty data still produces a drawing.
    CHECK_FALSE(line_chart("x", "a", "b", {}).items.empty());
}

TEST_CASE("HTML report carries tables, badges, figures and the notice") {
    Document doc;
    doc.title = "Report <1>";
    doc.subtitle = "OM10 & friends";
    doc.notice = "CC BY-SA 3.0";
    doc.heading("Section");
    doc.paragraph("Plain text & symbols");
    doc.key_values({{"module", "0.15"}});
    doc.table({"check", "value"}, {{"a", "1"}, {"b", "2"}}, {Status::Pass, Status::Fail});
    doc.figure(sample(), "A figure");
    doc.callout(Status::Warn, "Be careful");
    const std::string html = to_html(doc);
    CHECK_THAT(html, ContainsSubstring("<title>Report &lt;1&gt;</title>"));
    CHECK_THAT(html, ContainsSubstring("<span class=\"badge pass\">PASS</span>"));
    CHECK_THAT(html, ContainsSubstring("<span class=\"badge fail\">FAIL</span>"));
    CHECK_THAT(html, ContainsSubstring("<dt>module</dt><dd>0.15</dd>"));
    CHECK_THAT(html, ContainsSubstring("<svg"));
    CHECK_THAT(html, ContainsSubstring("prefers-color-scheme:dark"));
    CHECK_THAT(html, ContainsSubstring("CC BY-SA 3.0"));
    CHECK_THAT(html, ContainsSubstring("name=\"viewport\""));
    CHECK(doc.overall() == Status::Fail);
    Document ok;
    ok.callout(Status::Pass, "fine");
    CHECK(ok.overall() == Status::Pass);
    CHECK(Document{}.overall() == Status::None);
}

TEST_CASE("PDF report is structurally valid and paginates") {
    Document doc;
    doc.title = "A long report";
    doc.subtitle = "with enough content for several pages";
    for (int s = 0; s < 6; ++s) {
        doc.heading("Section " + std::to_string(s));
        for (int p = 0; p < 6; ++p) doc.paragraph(std::string(400, 'x') + " words words words (parentheses) \\ backslash ° µm ≥ π");
        std::vector<std::vector<std::string>> rows;
        for (int r = 0; r < 12; ++r) rows.push_back({"row " + std::to_string(r), "some longer text that has to wrap inside its column when the table is narrow", "1.234"});
        doc.table({"name", "description", "value"}, rows, std::vector<Status>(12, Status::Pass));
        doc.figure(sample(), "figure");
        doc.callout(Status::Fail, "failed");
    }
    const std::string pdf = to_pdf(doc);
    std::size_t n = 0;
    for (std::size_t p = 0; (p = pdf.find("/Type /Page ", p)) != std::string::npos; ++p) ++n;
    CHECK(n >= 5);
    check_pdf_structure(pdf, static_cast<int>(n));
    CHECK_THAT(pdf, ContainsSubstring("/BaseFont /Helvetica-Bold"));
    CHECK_THAT(pdf, ContainsSubstring("page 1 of " + std::to_string(n)));
    // Characters outside WinAnsi are spelled out; parentheses and backslashes are escaped.
    CHECK_THAT(pdf, ContainsSubstring("\\(parentheses\\)"));
    CHECK_THAT(pdf, ContainsSubstring("\\\\ backslash"));
    CHECK_THAT(pdf, ContainsSubstring(">= pi"));

    // If poppler is installed, an independent reader must accept the file and find the text.
    if (std::system("command -v pdftotext > /dev/null 2>&1") == 0) {
        const auto path = std::filesystem::temp_directory_path() / "ohs_report_test.pdf";
        std::ofstream(path, std::ios::binary) << pdf;
        const auto txt = std::filesystem::temp_directory_path() / "ohs_report_test.txt";
        REQUIRE(std::system(("pdftotext '" + path.string() + "' '" + txt.string() + "' 2>/dev/null").c_str()) == 0);
        std::ifstream in(txt);
        std::stringstream buf;
        buf << in.rdbuf();
        CHECK_THAT(buf.str(), ContainsSubstring("A long report"));
        CHECK_THAT(buf.str(), ContainsSubstring("Section 5"));
        CHECK_THAT(buf.str(), ContainsSubstring("PASS"));
        std::filesystem::remove(path);
        std::filesystem::remove(txt);
    }
}

TEST_CASE("technical drawing PDF picks a scale and writes the title block") {
    const std::string pdf = drawing_to_pdf(sample(), "Gear 24T", {"module 0.15", "material brass"});
    check_pdf_structure(pdf, 1);
    CHECK_THAT(pdf, ContainsSubstring("Gear 24T"));
    CHECK_THAT(pdf, ContainsSubstring("scale 10:1, dimensions in mm"));  // the largest standard scale at which the 14 mm tall drawing fits A4
    Drawing big;
    big.polyline({{0, 0}, {500, 0}, {500, 300}, {0, 300}}, true);
    CHECK_THAT(drawing_to_pdf(big, "Big", {}), ContainsSubstring("scale 1:"));
}

TEST_CASE("WinAnsi mapping keeps Latin-1 and spells out the rest") {
    CHECK(to_winansi("25 °C") == "25 \xB0" "C");
    CHECK(to_winansi("5 µm") == "5 \xB5m");
    CHECK(to_winansi("a ≥ b ≤ c") == "a >= b <= c");
    CHECK(to_winansi("π r²") == "pi r\xB2");
    CHECK(to_winansi("en–dash") == "en\x96" "dash");
    CHECK(to_winansi("日本") == "??");
}
