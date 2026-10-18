// SPDX-License-Identifier: Apache-2.0
#include "manufacturing/part_drawing.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>

#include "core/error.hpp"

namespace ohs::manufacturing {

namespace {

using mesh::P2;
using report::Layer;
using report::Pt;

std::string num(double v, int prec = 3) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

}  // namespace

PartDrawing make_part_drawing(const PartDrawingInput& in) {
    if (in.region.rings.empty()) throw Error("the part has no outline to draw");
    PartDrawing out;
    out.drawing.title = in.title;

    double lx = 1e300, ly = 1e300, hx = -1e300, hy = -1e300;
    for (const P2& p : in.region.rings[0]) {
        lx = std::min(lx, p.x);
        hx = std::max(hx, p.x);
        ly = std::min(ly, p.y);
        hy = std::max(hy, p.y);
    }
    out.width = hx - lx;
    out.height = hy - ly;
    auto T = [&](const P2& p) { return Pt{p.x - lx, p.y - ly}; };
    const double ref = std::max(out.width, out.height);
    const double text_h = std::max(0.9, 0.035 * ref);

    std::vector<Pt> outline;
    for (const P2& p : in.region.rings[0]) outline.push_back(T(p));
    out.drawing.polyline(outline, true, Layer::Outline);

    for (std::size_t k = 1; k < in.region.rings.size(); ++k) {
        const auto& ring = in.region.rings[k];
        double cx = 0, cy = 0;
        for (const P2& p : ring) {
            cx += p.x;
            cy += p.y;
        }
        cx /= static_cast<double>(ring.size());
        cy /= static_cast<double>(ring.size());
        double rmean = 0, rmax = 0, rmin = 1e300;
        for (const P2& p : ring) {
            const double r = std::hypot(p.x - cx, p.y - cy);
            rmean += r;
            rmax = std::max(rmax, r);
            rmin = std::min(rmin, r);
        }
        rmean /= static_cast<double>(ring.size());
        HoleInfo h;
        h.x = cx - lx;
        h.y = cy - ly;
        h.diameter = 2 * rmean;
        h.round = ring.size() >= 12 && (rmax - rmin) < 0.02 * rmean;
        if (h.round) {
            out.drawing.circle({h.x, h.y}, rmean, Layer::Outline);
            const double arm = std::max(1.5 * rmean, 0.8);
            out.drawing.line({h.x - arm, h.y}, {h.x + arm, h.y}, Layer::Centre);
            out.drawing.line({h.x, h.y - arm}, {h.x, h.y + arm}, Layer::Centre);
            out.drawing.text({h.x + rmean * 0.75 + 0.3 * text_h, h.y + rmean * 0.75 + 0.3 * text_h}, "D " + num(h.diameter), text_h, Layer::Dimension, 0);
        } else {
            std::vector<Pt> pts;
            for (const P2& p : ring) pts.push_back(T(p));
            out.drawing.polyline(pts, true, Layer::Outline);
        }
        out.holes.push_back(h);
    }

    // Overall dimensions.
    const double off = 0.12 * ref + text_h;
    out.drawing.line({0, -0.6 * text_h}, {0, -off - 0.6 * text_h}, Layer::Dimension);
    out.drawing.line({out.width, -0.6 * text_h}, {out.width, -off - 0.6 * text_h}, Layer::Dimension);
    out.drawing.line({0, -off}, {out.width, -off}, Layer::Dimension);
    out.drawing.text({out.width / 2, -off - 1.4 * text_h}, num(out.width), text_h, Layer::Dimension, 1);
    out.drawing.line({out.width + 0.6 * text_h, 0}, {out.width + off + 0.6 * text_h, 0}, Layer::Dimension);
    out.drawing.line({out.width + 0.6 * text_h, out.height}, {out.width + off + 0.6 * text_h, out.height}, Layer::Dimension);
    out.drawing.line({out.width + off, 0}, {out.width + off, out.height}, Layer::Dimension);
    out.drawing.text({out.width + off + 0.5 * text_h, out.height / 2}, num(out.height), text_h, Layer::Dimension, 0);

    out.info.push_back(in.title);
    std::ostringstream size;
    size << "Overall " << num(out.width) << " x " << num(out.height) << " x " << num(in.thickness) << " mm" << (in.material.empty() ? "" : ", material " + in.material);
    out.info.push_back(size.str());
    if (in.teeth > 0) out.info.push_back(std::to_string(in.teeth) + " teeth" + (in.module > 0 ? ", module " + num(in.module, 4) : ""));
    for (std::size_t k = 0; k < out.holes.size(); ++k) {
        const HoleInfo& h = out.holes[k];
        out.info.push_back("Hole " + std::to_string(k + 1) + ": " + (h.round ? "D " + num(h.diameter) : "irregular") + " at x " + num(h.x) + ", y " + num(h.y) + " from the lower left corner");
    }
    for (const std::string& n : in.notes) out.info.push_back(n);
    return out;
}

}  // namespace ohs::manufacturing
