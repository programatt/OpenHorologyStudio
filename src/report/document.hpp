// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "report/drawing.hpp"

namespace ohs::report {

enum class Status { None, Pass, Warn, Fail };

const char* status_word(Status s);

struct Block {
    enum class Kind { Heading, Paragraph, KeyValues, Table, Figure, Callout } kind{Kind::Paragraph};
    int level{2};
    std::string text;                                     ///< heading, paragraph, callout or figure caption
    std::vector<std::pair<std::string, std::string>> pairs;
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    std::vector<Status> row_status;                       ///< optional, one per row
    Drawing figure;
    Status status{Status::None};
};

/// A report as data: the same document is written as HTML and as PDF.
struct Document {
    std::string title;
    std::string subtitle;
    std::string notice;       ///< licence or attribution line printed at the end
    std::vector<Block> blocks;

    void heading(std::string text, int level = 2);
    void paragraph(std::string text);
    void key_values(std::vector<std::pair<std::string, std::string>> pairs);
    void table(std::vector<std::string> header, std::vector<std::vector<std::string>> rows, std::vector<Status> status = {});
    void figure(Drawing drawing, std::string caption);
    void callout(Status status, std::string text);

    /// Worst status among callouts and table rows: Fail > Warn > Pass > None.
    Status overall() const;
};

/// Self-contained HTML (inline CSS following the reader's light or dark scheme, inline SVG).
std::string to_html(const Document& document);

/// PDF 1.4 on A4 pages with Helvetica text, tables and vector figures; no external tools.
std::string to_pdf(const Document& document);

/// One-page PDF of a technical drawing, scaled to fit and titled; `info` lines go in the title block.
std::string drawing_to_pdf(const Drawing& drawing, const std::string& title, const std::vector<std::string>& info);

/// Maps UTF-8 text to the PDF WinAnsi (Latin-1) byte range, replacing characters outside it by
/// close ASCII spellings (>= becomes ">=", pi becomes "pi").
std::string to_winansi(const std::string& utf8);

}  // namespace ohs::report
