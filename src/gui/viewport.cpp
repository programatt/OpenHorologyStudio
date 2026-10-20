// SPDX-License-Identifier: Apache-2.0
#include "gui/viewport.hpp"

#include "core/parallel.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <iterator>

#include <QFontMetrics>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QWheelEvent>

namespace ohs::gui {

namespace {

QColor role_colour(model::Role role) {
    using model::Role;
    switch (role) {
        case Role::Plate:
        case Role::Bridge: return QColor(184, 178, 158);
        case Role::Arbor:
        case Role::Stem:
        case Role::Pinion:
        case Role::PalletFork:
        case Role::Screw: return QColor(156, 160, 168);
        case Role::Lever: return QColor(176, 180, 188);   // bright steel, set off from the screws and arbors
        case Role::Spring: return QColor(92, 110, 154);   // blued steel
        case Role::Bearing: return QColor(190, 138, 66);  // a redder brass than the train's
        case Role::Jewel: return QColor(200, 40, 60);
        case Role::Hairspring: return QColor(70, 110, 190);
        case Role::Rotor: return QColor(168, 112, 74);  // copper: apart from the brass train and the plates
        case Role::DateRing: return QColor(226, 224, 214);  // a white date disc
        default: return QColor(201, 163, 76);
    }
}

/// The colour of a mobile when the rig is shown: one hue each, spread round the wheel.
QColor mobile_colour(int mobile) { return QColor::fromHsv((mobile * 67 + 20) % 360, 150, 225); }

}  // namespace

Viewport::Viewport(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
    settle_ = new QTimer(this);
    settle_->setSingleShot(true);
    settle_->setInterval(150);
    connect(settle_, &QTimer::timeout, this, [this] {
        fast_ = false;
        update();
    });
    set_renderer(saved_renderer());
}

Viewport::~Viewport() = default;

void Viewport::interactive_update() {
    // The graphics card draws every frame at full quality, so there is no cheap frame to settle from.
    if (!gl_on_) {
        fast_ = true;
        settle_->start();
    }
    update();
}

void Viewport::set_scene(std::shared_ptr<Scene> scene) {
    scene_ = std::move(scene);
    selected_.clear();
    drawn_valid_ = false;
    drawn_ = {};
    coarse_valid_ = false;
    drawn_coarse_ = {};
    ++look_;
    fit();
}

const Scene::Drawn& Viewport::drawn() const {
    if (!drawn_valid_ || drawn_revision_ != scene_->revision()) {
        drawn_ = scene_->draw_list();
        drawn_revision_ = scene_->revision();
        drawn_valid_ = true;
        // One run of triangles per part (draw_list goes part by part): its box and centroid, for picking and the wheel.
        bounds_.assign(scene_->parts().size(), PartBounds{});
        std::vector<std::size_t> corners(bounds_.size(), 0);
        std::vector<std::array<double, 3>> sum(bounds_.size(), {0, 0, 0});
        for (std::size_t i = 0; i < drawn_.triangles.size(); ++i) {
            PartBounds& b = bounds_[static_cast<std::size_t>(drawn_.part_index[i])];
            const std::size_t k = static_cast<std::size_t>(drawn_.part_index[i]);
            if (corners[k] == 0) {
                b.first = i;
                b.lo = {1e300, 1e300, 1e300};
                b.hi = {-1e300, -1e300, -1e300};
            }
            b.last = i + 1;
            for (const Point3* p : {&drawn_.triangles[i].a, &drawn_.triangles[i].b, &drawn_.triangles[i].c}) {
                for (int a = 0; a < 3; ++a) {
                    b.lo[a] = std::min(b.lo[a], (*p)[a]);
                    b.hi[a] = std::max(b.hi[a], (*p)[a]);
                    sum[k][a] += (*p)[a];
                }
                ++corners[k];
            }
        }
        for (std::size_t k = 0; k < bounds_.size(); ++k) {
            if (corners[k] == 0) continue;
            for (int a = 0; a < 3; ++a) bounds_[k].centroid[a] = sum[k][a] / static_cast<double>(corners[k]);
        }
    }
    return drawn_;
}

const Scene::Drawn& Viewport::drawn_coarse() const {
    if (!coarse_valid_ || coarse_revision_ != scene_->revision()) {
        drawn_coarse_ = scene_->draw_list(true);
        coarse_revision_ = scene_->revision();
        coarse_valid_ = true;
    }
    return drawn_coarse_;
}

QColor Viewport::backdrop() const {
    return palette().window().color().lightness() < 128 ? QColor(26, 28, 32) : QColor(226, 224, 218);
}

int Viewport::selected_index() const {
    if (!scene_ || selected_.empty()) return -1;
    const auto& parts = scene_->parts();
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (parts[i].name == selected_) return static_cast<int>(i);
    }
    return -1;
}

QColor Viewport::part_colour(int part, int selected) const {
    const QColor back = backdrop();
    const auto faded = [&back](const QColor& c) {
        return QColor((c.red() * 2 + back.red() * 3) / 5, (c.green() * 2 + back.green() * 3) / 5, (c.blue() * 2 + back.blue() * 3) / 5);
    };
    QColor base = role_colour(scene_->parts()[static_cast<std::size_t>(part)].role);
    if (!show_rig_ && walk_) {
        // The walk: the parts reached so far as they are, the mesh being looked at bright, the rest faded back.
        const bool current = part == walk_->driver || part == walk_->driven;
        const bool preview = part == walk_->preview_a || part == walk_->preview_b;
        if (current) base = walk_->gap ? QColor(240, 170, 40) : QColor(45, 205, 125);
        else if (preview) base = QColor(240, 170, 40);
        else if (static_cast<std::size_t>(part) >= walk_->walked.size() || !walk_->walked[static_cast<std::size_t>(part)]) base = faded(base);
    }
    if (show_rig_) {
        // Each mobile in a hue of its own; what does not turn is faded back so the train stands out.
        const int mobile = scene_->parts()[static_cast<std::size_t>(part)].mobile;
        base = mobile >= 0 ? mobile_colour(mobile) : faded(base);
    }
    // Cyan: the complement of the brass most parts are drawn in, and unlike the hairspring's blue.
    if (part == selected) base = QColor(40, 200, 235);
    return base;
}

void Viewport::set_view(double yaw_deg, double pitch_deg) {
    yaw_ = yaw_deg;
    pitch_ = std::clamp(pitch_deg, -180.0, 0.0);
    update();
}

void Viewport::fit() {
    zoom_ = 1.0;
    pan_x_ = pan_y_ = 0;
    update();
}

void Viewport::set_prompt(const QString& text) {
    prompt_ = text;
    if (prompt_.isEmpty()) unsetCursor();
    else setCursor(Qt::CrossCursor);
    update();
}

void Viewport::set_show_rig(bool on) {
    show_rig_ = on;
    ++look_;
    update();
}

void Viewport::set_walk(std::optional<WalkView> walk) {
    walk_ = std::move(walk);
    ++look_;
    update();
}

void Viewport::draw_walk(QPainter& painter, const QSize& size) const {
    const model::Movement& m = scene_->movement();
    const bool dark = palette().window().color().lightness() < 128;
    const QColor go(45, 205, 125), check(240, 170, 40);
    const auto centre = [&m](int component) {
        const model::PartFeatures& f = m.components[static_cast<std::size_t>(component)].features;
        return Point3{f.center.x, f.center.y, 0.5 * (f.z_min + f.z_max)};
    };
    const auto valid = [&m](int component) { return component >= 0 && static_cast<std::size_t>(component) < m.components.size(); };
    QFont small = font();  // the widget's, not the painter's: a snapshot's image would give a different default
    small.setPointSizeF(small.pointSizeF() * 0.95);
    small.setWeight(QFont::DemiBold);
    small.setStyleStrategy(QFont::NoSubpixelAntialias);  // the same pixels on screen and in a snapshot
    painter.setFont(small);
    const QFontMetrics fm(painter.font());
    // A name on a small plate, above the part, so it reads on every colour.
    const auto label = [&](int component, const QString& text, const QColor& ink, double below = 0.0) {
        const QPointF at = to_screen(centre(component), size);
        const QRectF box(at.x() - fm.horizontalAdvance(text) / 2.0 - 6, at.y() - 30 + below, fm.horizontalAdvance(text) + 12, fm.height() + 4);
        painter.setPen(QPen(ink, 1.5));
        painter.setBrush(dark ? QColor(26, 28, 32, 225) : QColor(240, 238, 232, 230));
        painter.drawRoundedRect(box, 4, 4);
        painter.setPen(dark ? QColor(236, 236, 238) : QColor(28, 28, 32));  // not palette().text(): that follows whether the window is active
        painter.drawText(box, Qt::AlignCenter, text);
    };
    const auto name = [&m](int component) {
        const auto& f = m.components[static_cast<std::size_t>(component)].features;
        return QString::fromStdString(m.components[static_cast<std::size_t>(component)].display_name()) + (f.teeth > 0 ? QString(" · %1T").arg(f.teeth) : QString());
    };

    // The pair that could fill a gap: a dotted line, with both parts named.
    if (valid(walk_->preview_a) && valid(walk_->preview_b)) {
        const QPointF a = to_screen(centre(walk_->preview_a), size), b = to_screen(centre(walk_->preview_b), size);
        painter.setPen(QPen(QColor(0, 0, 0, 120), 6.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(QPen(check, 3.4, Qt::DotLine, Qt::RoundCap));
        painter.drawLine(a, b);
        label(walk_->preview_a, name(walk_->preview_a), check);
        label(walk_->preview_b, name(walk_->preview_b), check, 52.0);
    }
    if (!valid(walk_->driver)) return;
    if (walk_->gap || !valid(walk_->driven)) {
        label(walk_->driver, tr("%1 — nothing meshes here yet").arg(name(walk_->driver)), check);
        return;
    }
    const QPointF a = to_screen(centre(walk_->driver), size), b = to_screen(centre(walk_->driven), size);
    painter.setPen(QPen(QColor(0, 0, 0, 140), 7.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(a, b);
    painter.setPen(QPen(go, 4.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(a, b);
    // The arrowhead sits towards the driven part, so the direction of the drive reads at a glance.
    const double dx = b.x() - a.x(), dy = b.y() - a.y(), len = std::hypot(dx, dy);
    if (len > 1.0) {
        const QPointF tip = a + QPointF(dx, dy) * 0.62;
        const double back = std::atan2(dy, dx) + std::numbers::pi, head = 13.0;
        QPolygonF arrow;
        arrow << tip << tip + QPointF(head * std::cos(back + 0.5), head * std::sin(back + 0.5))
              << tip + QPointF(head * std::cos(back - 0.5), head * std::sin(back - 0.5));
        painter.setPen(QPen(QColor(0, 0, 0, 140), 2.0));
        painter.setBrush(go);
        painter.drawPolygon(arrow);
    }
    label(walk_->driver, name(walk_->driver), go);
    label(walk_->driven, name(walk_->driven), go, 52.0);  // below its part, so it does not sit on the driver's
}

void Viewport::select(const std::string& part) {
    selected_ = part;
    ++look_;
    update();
}

QPointF Viewport::to_screen(const Point3& p, const QSize& size) const {
    const Point3 c = scene_->centre();
    const double scale = 0.45 * std::min(size.width(), size.height()) / scene_->radius() * zoom_;
    const double yaw = yaw_ * std::numbers::pi / 180.0, pitch = pitch_ * std::numbers::pi / 180.0;
    const double x = p[0] - c[0], y = p[1] - c[1], z = p[2] - c[2];
    const double x1 = std::cos(yaw) * x - std::sin(yaw) * y, y1 = std::sin(yaw) * x + std::cos(yaw) * y;
    const double y2 = std::cos(pitch) * y1 - std::sin(pitch) * z;
    return {size.width() / 2.0 + pan_x_ + scale * x1, size.height() / 2.0 + pan_y_ - scale * y2};
}

void Viewport::draw_rig(QPainter& painter, const QSize& size) const {
    const model::Movement& m = scene_->movement();
    const auto& review = scene_->review();
    const bool dark = palette().window().color().lightness() < 128;
    const QColor fed(60, 200, 110), unfed(120, 150, 210), manual(190, 120, 235), check(240, 170, 40);
    const auto centre = [&m](int component) {
        const model::PartFeatures& f = m.components[static_cast<std::size_t>(component)].features;
        return Point3{f.center.x, f.center.y, 0.5 * (f.z_min + f.z_max)};
    };

    // Meshes, over a dark edge so they read on every part colour.
    for (const model::MeshJoint& j : m.meshes) {
        const int ma = m.mobile_of(j.a), mb = m.mobile_of(j.b);
        const bool powered = ma >= 0 && mb >= 0 && review.chain.fed.size() > static_cast<std::size_t>(std::max(ma, mb)) &&
                             review.chain.fed[static_cast<std::size_t>(ma)] && review.chain.fed[static_cast<std::size_t>(mb)];
        QColor ink = powered ? fed : unfed;
        if (j.manual) ink = manual;
        if (j.confidence < 0.6) ink = check;
        const QPointF a = to_screen(centre(j.a), size), b = to_screen(centre(j.b), size);
        painter.setPen(QPen(QColor(0, 0, 0, 130), 6.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(QPen(ink, 3.4, j.manual ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(Qt::NoPen);
        painter.setBrush(ink);
        painter.drawEllipse(a, 4.0, 4.0);
        painter.drawEllipse(b, 4.0, 4.0);
    }

    // Meshes worth declaring.
    QFont small = painter.font();
    small.setPointSizeF(small.pointSizeF() * 0.9);
    painter.setFont(small);
    for (const rigging::MeshSuggestion& s : review.suggestions) {
        const QPointF a = to_screen(centre(s.a), size), b = to_screen(centre(s.b), size);
        painter.setPen(QPen(QColor(0, 0, 0, 110), 5.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(QPen(check, 3.0, Qt::DotLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(dark ? QColor(250, 215, 140) : QColor(120, 70, 0));
        painter.drawText((a + b) / 2.0 + QPointF(6, -5), tr("? %1").arg(s.confidence, 0, 'f', 2));
    }

    // A ring round each part a warning or an error is about.
    for (const rigging::Issue& issue : review.issues) {
        if (issue.severity == rigging::Severity::Info) continue;
        for (const std::string& name : issue.parts) {
            const int c = m.find_component(name);
            if (c < 0) continue;
            const Point3 at = centre(c);
            const double outer = m.components[static_cast<std::size_t>(c)].features.outer_radius;
            const QPointF p = to_screen(at, size), edge = to_screen({at[0] + outer, at[1], at[2]}, size);
            const double radius = std::max(10.0, std::hypot(edge.x() - p.x(), edge.y() - p.y()) + 4.0);
            painter.setPen(QPen(issue.severity == rigging::Severity::Error ? QColor(235, 70, 60) : check, 2.6, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(p, radius, radius);
        }
    }

    // What the colours mean.
    struct Key {
        QColor ink;
        Qt::PenStyle style;
        QString text;
    };
    const Key keys[] = {{fed, Qt::SolidLine, tr("mesh fed from the barrel")},
                        {unfed, Qt::SolidLine, tr("mesh the barrel does not reach")},
                        {manual, Qt::DashLine, tr("mesh you declared")},
                        {check, Qt::SolidLine, tr("low confidence, or a part to check (ring)")},
                        {check, Qt::DotLine, tr("? possible mesh to confirm")}};
    const QFontMetrics fm(painter.font());
    const int row = fm.height() + 4;
    const QRect box(10, 10, 14 + 30 + fm.horizontalAdvance(tr("low confidence, or a part to check (ring)")) + 12,
                    row * static_cast<int>(std::size(keys)) + 10);
    painter.setPen(Qt::NoPen);
    painter.setBrush(dark ? QColor(26, 28, 32, 200) : QColor(236, 234, 228, 215));
    painter.drawRoundedRect(box, 5, 5);
    int y = box.top() + 5 + row / 2;
    for (const Key& k : keys) {
        painter.setPen(QPen(k.ink, 3.0, k.style, Qt::RoundCap));
        painter.drawLine(box.left() + 8, y, box.left() + 36, y);
        painter.setPen(palette().text().color());
        painter.drawText(box.left() + 44, y + fm.ascent() / 2 - 1, k.text);
        y += row;
    }
}

void Viewport::draw_overlays(QPainter& painter, const QSize& size) const {
    const bool lifted = scene_->explode() > 0;  // the overlays follow the assembled positions only
    if (show_rig_ && !lifted) draw_rig(painter, size);
    else if (walk_ && !lifted) draw_walk(painter, size);
    if (scene_->show_trails() && !lifted) {
        const auto trails = scene_->trails();
        for (std::size_t i = 0; i < trails.size(); ++i) {
            const QColor hue = QColor::fromHsv(static_cast<int>((i * 47) % 360), 200, 230);
            const auto& pts = trails[i].points;
            for (std::size_t k = 1; k < pts.size(); ++k) {
                QColor c = hue;
                c.setAlpha(30 + static_cast<int>(200.0 * static_cast<double>(k) / static_cast<double>(pts.size() - 1)));  // older is fainter
                painter.setPen(QPen(c, 2.0, Qt::SolidLine, Qt::RoundCap));
                painter.drawLine(to_screen(pts[k - 1], size), to_screen(pts[k], size));
            }
            painter.setPen(Qt::NoPen);
            painter.setBrush(hue);
            painter.drawEllipse(to_screen(pts.back(), size), 3.0, 3.0);
        }
    }
    if (scene_->show_forces() && !lifted && !scene_->forces().empty()) {
        double biggest = 0.0;
        for (const auto& f : scene_->forces()) biggest = std::max(biggest, f.newtons);
        const QColor ink(225, 50, 50);
        const QColor text = palette().window().color().lightness() < 128 ? QColor(240, 200, 200) : QColor(140, 20, 20);
        for (const auto& f : scene_->forces()) {
            if (biggest <= 0) break;
            const double length = 0.22 * scene_->radius() * f.newtons / biggest;
            const Point3 tip{f.at[0] + f.direction[0] * length, f.at[1] + f.direction[1] * length, f.at[2]};
            const QPointF a = to_screen(f.at, size), b = to_screen(tip, size);
            painter.setPen(QPen(ink, 2.6, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(a, b);
            const double dx = b.x() - a.x(), dy = b.y() - a.y(), len = std::hypot(dx, dy);
            if (len > 1.0) {
                const double head = std::max(8.0, 0.28 * len), back = std::atan2(dy, dx) + std::numbers::pi;
                for (double turn : {0.45, -0.45}) {
                    painter.drawLine(b, QPointF(b.x() + head * std::cos(back + turn), b.y() + head * std::sin(back + turn)));
                }
            }
            painter.setPen(text);
            painter.drawText(b + QPointF(6, -4), tr("%1 mN").arg(f.newtons * 1000.0, 0, 'g', 3));
        }
    }
}

Viewport::View Viewport::view_for(const QSize& size) const {
    const double yaw = yaw_ * std::numbers::pi / 180.0, pitch = pitch_ * std::numbers::pi / 180.0;
    return {scene_->centre(), 0.45 * std::min(size.width(), size.height()) / scene_->radius() * zoom_,
            std::cos(yaw), std::sin(yaw), std::cos(pitch), std::sin(pitch),
            size.width() / 2.0 + pan_x_, size.height() / 2.0 + pan_y_};
}

bool Viewport::project_triangle(const Scene::Drawn& drawn, std::size_t i, const View& v, Projected& p) const {
    // Rotate about Z by yaw, then tilt about X by pitch (0 looks down the dial axis, -90 is edge-on, -180 looks from below).
    auto to_view = [&](const Point3& q) {
        const double x = q[0] - v.c[0], y = q[1] - v.c[1], z = q[2] - v.c[2];
        const double x1 = v.cy * x - v.sy * y, y1 = v.sy * x + v.cy * y;
        return Point3{x1, v.cp * y1 - v.sp * z, v.sp * y1 + v.cp * z};  // x right, y up, z towards the viewer
    };
    // Light from the upper left, in view space.
    const double lx = -0.4, ly = 0.5, lz = 0.77;
    const Triangle& t = drawn.triangles[i];
    p = Projected{};
    const Point3 a = to_view(t.a), b = to_view(t.b), cc = to_view(t.c);
    const Point3* corners[3] = {&a, &b, &cc};
    for (int k = 0; k < 3; ++k) {
        p.x[k] = v.half_w + v.scale * (*corners[k])[0];
        p.y[k] = v.half_h - v.scale * (*corners[k])[1];
    }
    p.depth = (a[2] + b[2] + cc[2]) / 3.0;
    p.part = drawn.part_index[i];
    p.value = (drawn.values[i][0] + drawn.values[i][1] + drawn.values[i][2]) / 3.0;

    const Point3 u{b[0] - a[0], b[1] - a[1], b[2] - a[2]}, w{cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]};
    const Point3 n{u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len < 1e-12) return false;
    // The far side of a closed solid is always covered by its near side, so it is not drawn: about half the
    // triangles. Not with a cut-away (it shows the inside), nor on a field plot (its faces are not oriented).
    if (n[2] < 0 && !scene_->section().enabled && scene_->parts()[static_cast<std::size_t>(p.part)].closed && std::isnan(p.value)) return false;
    const double dot = (n[0] * lx + n[1] * ly + n[2] * lz) / len;
    p.shade = 0.35 + 0.65 * std::abs(dot);
    return true;
}

std::vector<Viewport::Projected> Viewport::project(const QSize& size, bool skip_tiny) const {
    std::vector<Projected> out;
    if (!scene_) return out;
    // A frame that is about to be replaced is drawn from the coarse mesh, without the triangles under a pixel.
    const Scene::Drawn& drawn = skip_tiny ? drawn_coarse() : this->drawn();
    const View view = view_for(size);
    // Projecting is independent per triangle: chunks of the list on threads, joined in order.
    constexpr std::size_t kChunk = 16384;
    const std::size_t n = drawn.triangles.size();
    const std::size_t chunks = (n + kChunk - 1) / kChunk;
    std::vector<std::vector<Projected>> parts(chunks);
    parallel_for(chunks, [&](std::size_t c) {
        std::vector<Projected>& mine = parts[c];
        mine.reserve(std::min(kChunk, n - c * kChunk));
        for (std::size_t i = c * kChunk; i < std::min(n, (c + 1) * kChunk); ++i) {
            Projected p;
            if (!project_triangle(drawn, i, view, p)) continue;
            if (skip_tiny) {
                const double area2 = std::abs((p.x[1] - p.x[0]) * (p.y[2] - p.y[0]) - (p.x[2] - p.x[0]) * (p.y[1] - p.y[0]));
                if (area2 < 0.5) continue;  // under a quarter of a pixel
            }
            mine.push_back(p);
        }
    }, chunks > 1 ? 0u : 1u);
    out.reserve(n);
    for (auto& mine : parts) out.insert(out.end(), mine.begin(), mine.end());
    // Painter's algorithm: far to near. View-space z grows towards the viewer.
    std::sort(out.begin(), out.end(), [](const Projected& a, const Projected& b) { return a.depth < b.depth; });
    return out;
}

void Viewport::draw(QPainter& painter, const QSize& size, bool fast) const {
    painter.fillRect(QRect(QPoint(0, 0), size), backdrop());
    // Antialiased triangles need a thin outline to hide the seams between them, and the two together are most
    // of a frame. Aliased fills tile without seams, so a fast frame draws neither.
    painter.setRenderHint(QPainter::Antialiasing, !fast);
    if (fast) painter.setPen(Qt::NoPen);
    if (!scene_) {
        // An empty state that says how to get started rather than only that there is nothing.
        const QRect all(QPoint(0, 0), size);
        QFont title = painter.font();
        title.setPointSizeF(title.pointSizeF() * 1.4);
        title.setWeight(QFont::DemiBold);
        painter.setFont(title);
        painter.setPen(palette().text().color());
        painter.drawText(all.adjusted(0, 0, 0, -40), Qt::AlignCenter, tr("No movement open"));
        painter.setFont(font());
        painter.setPen(palette().placeholderText().color());
        painter.drawText(all.adjusted(24, 30, -24, 0), Qt::AlignCenter | Qt::TextWordWrap,
                         tr("Open a project, import a STEP file, or try the demonstration movement from the toolbar."));
        return;
    }
    const FieldOverlay* field = scene_->field();
    const int selected = selected_index();
    std::vector<QPolygonF> selected_outline;  // the selected part's triangles, drawn again on top
    std::vector<QColor> colours(scene_->parts().size());
    for (std::size_t i = 0; i < colours.size(); ++i) colours[i] = part_colour(static_cast<int>(i), selected);
    for (const Projected& p : project(size, fast)) {
        QColor base = colours[static_cast<std::size_t>(p.part)];
        if (p.part == selected) selected_outline.push_back(QPolygonF({{p.x[0], p.y[0]}, {p.x[1], p.y[1]}, {p.x[2], p.y[2]}}));
        if (field && !std::isnan(p.value)) {
            const auto c = field_colour((p.value - field->lo) / (field->hi - field->lo));
            base = QColor(c[0], c[1], c[2]);
        }
        const QColor col(std::min(255, int(base.red() * p.shade)), std::min(255, int(base.green() * p.shade)),
                         std::min(255, int(base.blue() * p.shade)));
        if (!fast) painter.setPen(QPen(col, 0.6));
        painter.setBrush(col);
        const QPointF pts[3] = {{p.x[0], p.y[0]}, {p.x[1], p.y[1]}, {p.x[2], p.y[2]}};
        painter.drawPolygon(pts, 3);
    }
    // X-ray: a part selected from the tree or the list may be hidden behind others; a faint tint over
    // everything shows where it is.
    if (!selected_outline.empty()) {
        QColor ghost(40, 200, 235);
        ghost.setAlpha(60);
        painter.setPen(Qt::NoPen);
        painter.setBrush(ghost);
        for (const QPolygonF& poly : selected_outline) painter.drawPolygon(poly);
    }
    draw_decorations(painter, size);
}

void Viewport::draw_decorations(QPainter& painter, const QSize& size) const {
    painter.setRenderHint(QPainter::Antialiasing, true);  // the overlays are few: always smooth
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    draw_overlays(painter, size);
    if (const FieldOverlay* field = scene_->field()) draw_legend(painter, size, *field);
}

void Viewport::draw_legend(QPainter& painter, const QSize& size, const FieldOverlay& field) const {
    const QColor text = palette().window().color().lightness() < 128 ? QColor(230, 230, 230) : QColor(30, 30, 30);
    const int bar_w = 14, bar_h = std::min(180, size.height() / 2), x = size.width() - 78, y = 44;
    QLinearGradient gradient(0, y + bar_h, 0, y);
    for (int i = 0; i <= 8; ++i) {
        const auto c = field_colour(i / 8.0);
        gradient.setColorAt(i / 8.0, QColor(c[0], c[1], c[2]));
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(gradient);
    painter.drawRect(x, y, bar_w, bar_h);
    painter.setPen(text);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(x, y, bar_w, bar_h);
    painter.drawText(QRect(x - 60, y - 22, 150, 18), Qt::AlignLeft, QString::fromStdString(field.label));
    painter.drawText(QRect(x + bar_w + 4, y - 6, 60, 14), Qt::AlignLeft, QString::number(field.hi, 'g', 4));
    painter.drawText(QRect(x + bar_w + 4, y + bar_h - 8, 60, 14), Qt::AlignLeft, QString::number(field.lo, 'g', 4));
}

void Viewport::paintEvent(QPaintEvent*) {
    if (gl_on_) return;  // the OpenGL surface over this widget draws everything
    QPainter painter(this);
    draw(painter, size(), fast_);
    draw_notes(painter);
}

void Viewport::draw_notes(QPainter& painter) const {
    if (scene_ && !prompt_.isEmpty()) {
        // A pick is waiting for a click: a plate across the top, on screen only.
        QFont big = font();
        big.setPointSizeF(big.pointSizeF() * 1.1);
        big.setWeight(QFont::DemiBold);
        painter.setFont(big);
        const QFontMetrics fm(big);
        const QRectF box(10, 10, std::min<double>(width() - 20, fm.horizontalAdvance(prompt_) + 24), fm.height() + 14);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(240, 170, 40), 2.0));
        painter.setBrush(QColor(26, 28, 32, 235));
        painter.drawRoundedRect(box, 6, 6);
        painter.setPen(QColor(236, 236, 238));
        painter.drawText(box, Qt::AlignCenter, prompt_);
    }
    // How to move the view, on screen only: render() (snapshots, video) leaves it out.
    if (scene_) {
        QFont small = font();
        small.setPointSizeF(small.pointSizeF() * 0.85);
        painter.setFont(small);
        painter.setPen(palette().placeholderText().color());
        painter.drawText(rect().adjusted(10, 0, -10, -8), Qt::AlignLeft | Qt::AlignBottom,
                         tr("Drag a part to turn the movement · drag beside it to pan · click to select · "
                            "the wheel zooms toward the selected part"));
    }
}

QImage Viewport::render(const QSize& size, bool fast) const {
    QImage image(size, QImage::Format_ARGB32);
    QPainter painter(&image);
    draw(painter, size, fast);
    return image;
}

std::string Viewport::pick(const QPoint& pos) const {
    if (!scene_) return {};
    // The front-most triangle under the cursor, as the painter's order would have it, without projecting or sorting
    // the whole movement: only the parts whose screen box holds the point are looked into.
    const Scene::Drawn& drawn = this->drawn();
    const View view = view_for(size());
    const double px = pos.x(), py = pos.y();
    double best_depth = -1e300;
    int best_part = -1;
    for (std::size_t k = 0; k < bounds_.size(); ++k) {
        const PartBounds& b = bounds_[k];
        if (b.last <= b.first) continue;
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
        for (int corner = 0; corner < 8; ++corner) {
            const Point3 q{corner & 1 ? b.hi[0] : b.lo[0], corner & 2 ? b.hi[1] : b.lo[1], corner & 4 ? b.hi[2] : b.lo[2]};
            const QPointF s = to_screen(q, size());
            x0 = std::min(x0, s.x());
            x1 = std::max(x1, s.x());
            y0 = std::min(y0, s.y());
            y1 = std::max(y1, s.y());
        }
        if (px < x0 - 1 || px > x1 + 1 || py < y0 - 1 || py > y1 + 1) continue;
        for (std::size_t i = b.first; i < b.last; ++i) {
            Projected p;
            if (!project_triangle(drawn, i, view, p) || p.depth <= best_depth) continue;
            // On the same side of all three edges.
            const double d0 = (p.x[1] - p.x[0]) * (py - p.y[0]) - (p.y[1] - p.y[0]) * (px - p.x[0]);
            const double d1 = (p.x[2] - p.x[1]) * (py - p.y[1]) - (p.y[2] - p.y[1]) * (px - p.x[1]);
            const double d2 = (p.x[0] - p.x[2]) * (py - p.y[2]) - (p.y[0] - p.y[2]) * (px - p.x[2]);
            const bool negative = d0 < 0 || d1 < 0 || d2 < 0, positive = d0 > 0 || d1 > 0 || d2 > 0;
            if (negative && positive) continue;
            best_depth = p.depth;
            best_part = p.part;
        }
    }
    return best_part < 0 ? std::string{} : scene_->parts()[static_cast<std::size_t>(best_part)].name;
}

std::optional<QPointF> Viewport::screen_position(const std::string& part) const {
    if (!scene_ || part.empty()) return std::nullopt;
    (void)drawn();
    const auto& parts = scene_->parts();
    for (std::size_t k = 0; k < parts.size() && k < bounds_.size(); ++k) {
        if (parts[k].name != part) continue;
        if (bounds_[k].last <= bounds_[k].first) return std::nullopt;  // hidden
        return to_screen(bounds_[k].centroid, size());  // one point: where the part's triangles are centred
    }
    return std::nullopt;
}

void Viewport::mousePressEvent(QMouseEvent* e) {
    last_ = press_ = e->pos();
    dragged_ = false;
    // Left drag turns the movement when it starts on a part and pans when it starts beside it; the other
    // buttons always pan.
    if (e->button() == Qt::LeftButton) drag_ = scene_ && !pick(e->pos()).empty() ? Drag::Turn : Drag::Pan;
    else if (e->button() == Qt::RightButton || e->button() == Qt::MiddleButton) drag_ = Drag::Pan;
    setCursor(drag_ == Drag::Turn ? Qt::ClosedHandCursor : Qt::SizeAllCursor);
}

void Viewport::mouseMoveEvent(QMouseEvent* e) {
    const QPoint d = e->pos() - last_;
    last_ = e->pos();
    if ((e->pos() - press_).manhattanLength() > 3) dragged_ = true;
    if (drag_ == Drag::Turn) {
        // Across turns the movement about its central (dial) axis; up and down tilts it.
        yaw_ -= d.x() * 0.5;
        pitch_ = std::clamp(pitch_ - d.y() * 0.5, -180.0, 0.0);
        interactive_update();
    } else if (drag_ == Drag::Pan) {
        pan_x_ += d.x();
        pan_y_ += d.y();
        interactive_update();
    }
}

void Viewport::mouseReleaseEvent(QMouseEvent* e) {
    drag_ = Drag::None;
    if (prompt_.isEmpty()) unsetCursor();
    else setCursor(Qt::CrossCursor);
    if (dragged_ || e->button() != Qt::LeftButton || !scene_) return;
    selected_ = pick(e->pos());
    ++look_;
    update();
    emit part_picked(QString::fromStdString(selected_));
}

void Viewport::wheelEvent(QWheelEvent* e) {
    // With a part selected the zoom closes in on it: the part stays where it is on screen and the view
    // grows or shrinks around it. Screen = centre + pan + scale * view, so keeping the point A fixed while
    // the scale changes by f needs pan' = (A - centre) - f * (A - centre - pan). A is taken before zooming.
    const auto anchor = screen_position(selected_);
    const double before = zoom_;
    zoom_ = std::clamp(zoom_ * std::pow(1.0015, e->angleDelta().y()), 0.1, 50.0);
    if (anchor) {
        const double f = zoom_ / before;
        const double cx = width() / 2.0, cy = height() / 2.0;
        pan_x_ = (anchor->x() - cx) - f * (anchor->x() - cx - pan_x_);
        pan_y_ = (anchor->y() - cy) - f * (anchor->y() - cy - pan_y_);
    }
    interactive_update();
}

}  // namespace ohs::gui
