// SPDX-License-Identifier: Apache-2.0
#include "gui/viewport.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

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
        case Role::Pinion:
        case Role::PalletFork:
        case Role::Screw: return QColor(156, 160, 168);
        case Role::Jewel: return QColor(200, 40, 60);
        case Role::Hairspring: return QColor(70, 110, 190);
        default: return QColor(201, 163, 76);
    }
}

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
}

void Viewport::interactive_update() {
    fast_ = true;
    settle_->start();
    update();
}

void Viewport::set_scene(std::shared_ptr<Scene> scene) {
    scene_ = std::move(scene);
    selected_.clear();
    fit();
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

void Viewport::select(const std::string& part) {
    selected_ = part;
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

void Viewport::draw_overlays(QPainter& painter, const QSize& size) const {
    const bool lifted = scene_->explode() > 0;  // the overlays follow the assembled positions only
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

std::vector<Viewport::Projected> Viewport::project(const QSize& size) const {
    std::vector<Projected> out;
    if (!scene_) return out;
    const auto drawn = scene_->draw_list();
    const Point3 c = scene_->centre();
    const double r = scene_->radius();
    const double scale = 0.45 * std::min(size.width(), size.height()) / r * zoom_;
    const double yaw = yaw_ * std::numbers::pi / 180.0, pitch = pitch_ * std::numbers::pi / 180.0;
    const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);

    // Rotate about Z by yaw, then tilt about X by pitch (0 looks down the dial axis, -90 is edge-on, -180 looks from below).
    auto view = [&](const Point3& p, double& sx, double& sy_, double& depth) {
        const double x = p[0] - c[0], y = p[1] - c[1], z = p[2] - c[2];
        const double x1 = cy * x - sy * y, y1 = sy * x + cy * y;
        const double y2 = cp * y1 - sp * z, z2 = sp * y1 + cp * z;
        sx = size.width() / 2.0 + pan_x_ + scale * x1;
        sy_ = size.height() / 2.0 + pan_y_ - scale * y2;
        depth = z2;
    };
    // Light from the upper left, in view space.
    const double lx = -0.4, ly = 0.5, lz = 0.77;
    const bool cull = !scene_->section().enabled;
    out.reserve(drawn.triangles.size());
    for (std::size_t i = 0; i < drawn.triangles.size(); ++i) {
        const Triangle& t = drawn.triangles[i];
        Projected p{};
        double d[3];
        view(t.a, p.x[0], p.y[0], d[0]);
        view(t.b, p.x[1], p.y[1], d[1]);
        view(t.c, p.x[2], p.y[2], d[2]);
        p.depth = (d[0] + d[1] + d[2]) / 3.0;
        p.part = drawn.part_index[i];
        p.value = (drawn.values[i][0] + drawn.values[i][1] + drawn.values[i][2]) / 3.0;

        // Face normal in view space (x right, y up, z towards the viewer).
        auto vp = [&](const Point3& q) {
            const double x = q[0] - c[0], y = q[1] - c[1], z = q[2] - c[2];
            const double x1 = cy * x - sy * y, y1 = sy * x + cy * y;
            return Point3{x1, cp * y1 - sp * z, sp * y1 + cp * z};
        };
        const Point3 a = vp(t.a), b = vp(t.b), cc = vp(t.c);
        const Point3 u{b[0] - a[0], b[1] - a[1], b[2] - a[2]}, v{cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]};
        Point3 n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len < 1e-12) continue;
        // The far side of a closed solid is always covered by its near side, so it is not drawn: about half the
        // triangles. Not with a cut-away (it shows the inside), nor on a field plot (its faces are not oriented).
        if (n[2] < 0 && cull && scene_->parts()[static_cast<std::size_t>(p.part)].closed && std::isnan(p.value)) continue;
        const double dot = (n[0] * lx + n[1] * ly + n[2] * lz) / len;
        p.shade = 0.35 + 0.65 * std::abs(dot);
        out.push_back(p);
    }
    // Painter's algorithm: far to near. View-space z grows towards the viewer.
    std::sort(out.begin(), out.end(), [](const Projected& a, const Projected& b) { return a.depth < b.depth; });
    return out;
}

void Viewport::draw(QPainter& painter, const QSize& size, bool fast) const {
    painter.fillRect(QRect(QPoint(0, 0), size), palette().window().color().lightness() < 128 ? QColor(26, 28, 32) : QColor(226, 224, 218));
    // Antialiased triangles need a thin outline to hide the seams between them, and the two together are most
    // of a frame. Aliased fills tile without seams, so a fast frame draws neither.
    painter.setRenderHint(QPainter::Antialiasing, !fast);
    if (fast) painter.setPen(Qt::NoPen);
    if (!scene_) {
        painter.setPen(palette().text().color());
        painter.drawText(QRect(QPoint(0, 0), size), Qt::AlignCenter, tr("Open a project to see the movement"));
        return;
    }
    const FieldOverlay* field = scene_->field();
    for (const Projected& p : project(size)) {
        QColor base = role_colour(scene_->parts()[p.part].role);
        // Cyan: the complement of the brass most parts are drawn in, and unlike the hairspring's blue.
        if (!selected_.empty() && scene_->parts()[p.part].name == selected_) base = QColor(40, 200, 235);
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
    painter.setRenderHint(QPainter::Antialiasing, true);  // the overlays are few: always smooth
    draw_overlays(painter, size);
    if (field) draw_legend(painter, size, *field);
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
    QPainter painter(this);
    draw(painter, size(), fast_);
}

QImage Viewport::render(const QSize& size) const {
    QImage image(size, QImage::Format_ARGB32);
    QPainter painter(&image);
    draw(painter, size);
    return image;
}

std::string Viewport::pick(const QPoint& pos) const {
    const auto tris = project(size());
    // Nearest to the viewer first.
    for (auto it = tris.rbegin(); it != tris.rend(); ++it) {
        const QPolygonF poly({{it->x[0], it->y[0]}, {it->x[1], it->y[1]}, {it->x[2], it->y[2]}});
        if (poly.containsPoint(QPointF(pos), Qt::OddEvenFill)) return scene_->parts()[it->part].name;
    }
    return {};
}

void Viewport::mousePressEvent(QMouseEvent* e) {
    last_ = press_ = e->pos();
    dragged_ = false;
}

void Viewport::mouseMoveEvent(QMouseEvent* e) {
    const QPoint d = e->pos() - last_;
    last_ = e->pos();
    if ((e->pos() - press_).manhattanLength() > 3) dragged_ = true;
    if (e->buttons() & Qt::LeftButton) {
        yaw_ -= d.x() * 0.5;
        pitch_ = std::clamp(pitch_ - d.y() * 0.5, -180.0, 0.0);
        interactive_update();
    } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
        pan_x_ += d.x();
        pan_y_ += d.y();
        interactive_update();
    }
}

void Viewport::mouseReleaseEvent(QMouseEvent* e) {
    if (dragged_ || e->button() != Qt::LeftButton || !scene_) return;
    selected_ = pick(e->pos());
    update();
    emit part_picked(QString::fromStdString(selected_));
}

void Viewport::wheelEvent(QWheelEvent* e) {
    zoom_ = std::clamp(zoom_ * std::pow(1.0015, e->angleDelta().y()), 0.1, 50.0);
    interactive_update();
}

}  // namespace ohs::gui
