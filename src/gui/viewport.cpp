// SPDX-License-Identifier: Apache-2.0
#include "gui/viewport.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

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
}

void Viewport::set_scene(std::shared_ptr<Scene> scene) {
    scene_ = std::move(scene);
    selected_.clear();
    fit();
}

void Viewport::set_view(double yaw_deg, double pitch_deg) {
    yaw_ = yaw_deg;
    pitch_ = std::clamp(pitch_deg, -90.0, 90.0);
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

std::vector<Viewport::Projected> Viewport::project(const QSize& size) const {
    std::vector<Projected> out;
    if (!scene_) return out;
    const auto drawn = scene_->draw_list();
    const Point3 c = scene_->centre();
    const double r = scene_->radius();
    const double scale = 0.45 * std::min(size.width(), size.height()) / r * zoom_;
    const double yaw = yaw_ * std::numbers::pi / 180.0, pitch = pitch_ * std::numbers::pi / 180.0;
    const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);

    // Rotate about Z by yaw, then tilt about X by pitch (0 looks down the dial axis, -90 is edge-on).
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
        const double dot = (n[0] * lx + n[1] * ly + n[2] * lz) / len;
        p.shade = 0.35 + 0.65 * std::abs(dot);
        out.push_back(p);
    }
    // Painter's algorithm: far to near. View-space z grows towards the viewer.
    std::sort(out.begin(), out.end(), [](const Projected& a, const Projected& b) { return a.depth < b.depth; });
    return out;
}

void Viewport::draw(QPainter& painter, const QSize& size) const {
    painter.fillRect(QRect(QPoint(0, 0), size), palette().window().color().lightness() < 128 ? QColor(34, 36, 40) : QColor(236, 238, 242));
    painter.setRenderHint(QPainter::Antialiasing, true);
    if (!scene_) {
        painter.setPen(palette().text().color());
        painter.drawText(QRect(QPoint(0, 0), size), Qt::AlignCenter, tr("Open a project to see the movement"));
        return;
    }
    for (const Projected& p : project(size)) {
        QColor base = role_colour(scene_->parts()[p.part].role);
        if (!selected_.empty() && scene_->parts()[p.part].name == selected_) base = QColor(255, 120, 30);
        const QColor col(std::min(255, int(base.red() * p.shade)), std::min(255, int(base.green() * p.shade)),
                         std::min(255, int(base.blue() * p.shade)));
        painter.setPen(QPen(col, 0.6));
        painter.setBrush(col);
        const QPointF pts[3] = {{p.x[0], p.y[0]}, {p.x[1], p.y[1]}, {p.x[2], p.y[2]}};
        painter.drawPolygon(pts, 3);
    }
}

void Viewport::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    draw(painter, size());
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
        set_view(yaw_ - d.x() * 0.5, pitch_ - d.y() * 0.5);
    } else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
        pan_x_ += d.x();
        pan_y_ += d.y();
        update();
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
    update();
}

}  // namespace ohs::gui
