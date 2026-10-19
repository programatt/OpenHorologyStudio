// SPDX-License-Identifier: Apache-2.0
#include "gui/plot.hpp"

#include <algorithm>

#include <QImage>
#include <QPainter>
#include <QPainterPath>

namespace ohs::gui {

PlotWidget::PlotWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(180);
}

void PlotWidget::set_data(const QString& title, const QString& x_label, const QString& y_label,
                          std::vector<std::pair<double, double>> points) {
    title_ = title;
    x_label_ = x_label;
    y_label_ = y_label;
    points_ = std::move(points);
    std::sort(points_.begin(), points_.end());
    update();
}

void PlotWidget::clear() {
    points_.clear();
    title_.clear();
    update();
}

void PlotWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    draw(painter, size());
}

QImage PlotWidget::render(const QSize& size) const {
    QImage image(size, QImage::Format_ARGB32);
    QPainter painter(&image);
    draw(painter, size);
    return image;
}

void PlotWidget::draw(QPainter& painter, const QSize& size) const {
    // The theme's colours: the panel's base, its text, a quiet frame and the accent for the data.
    painter.fillRect(QRect(QPoint(0, 0), size), palette().base().color());
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QColor ink = palette().text().color();
    QColor frame = ink;
    frame.setAlpha(70);

    const QRectF box(52, 26, size.width() - 70, size.height() - 66);
    painter.setPen(ink);
    QFont bold = painter.font();
    bold.setWeight(QFont::DemiBold);
    painter.save();
    painter.setFont(bold);
    painter.drawText(QRectF(0, 4, size.width(), 20), Qt::AlignCenter, title_);
    painter.restore();
    painter.setPen(frame);
    painter.drawRect(box);
    painter.setPen(ink);
    if (points_.empty()) {
        painter.drawText(box, Qt::AlignCenter, tr("no data"));
        return;
    }

    double x0 = points_.front().first, x1 = points_.back().first;
    double y0 = points_.front().second, y1 = y0;
    for (const auto& p : points_) {
        y0 = std::min(y0, p.second);
        y1 = std::max(y1, p.second);
    }
    if (x1 - x0 < 1e-12) { x0 -= 1; x1 += 1; }
    if (y1 - y0 < 1e-12) { y0 -= 1; y1 += 1; }
    const double pad = 0.08 * (y1 - y0);
    y0 -= pad;
    y1 += pad;
    auto sx = [&](double x) { return box.left() + (x - x0) / (x1 - x0) * box.width(); };
    auto sy = [&](double y) { return box.bottom() - (y - y0) / (y1 - y0) * box.height(); };

    painter.drawText(QRectF(box.left(), box.bottom() + 4, box.width(), 16), Qt::AlignLeft, QString::number(x0, 'g', 4));
    painter.drawText(QRectF(box.left(), box.bottom() + 4, box.width(), 16), Qt::AlignRight, QString::number(x1, 'g', 4));
    painter.drawText(QRectF(box.left(), box.bottom() + 20, box.width(), 16), Qt::AlignCenter, x_label_);
    painter.drawText(QRectF(2, box.top(), 48, 16), Qt::AlignRight, QString::number(y1, 'g', 3));
    painter.drawText(QRectF(2, box.bottom() - 16, 48, 16), Qt::AlignRight, QString::number(y0, 'g', 3));
    painter.save();
    painter.translate(12, box.center().y());
    painter.rotate(-90);
    painter.drawText(QRectF(-box.height() / 2, -2, box.height(), 16), Qt::AlignCenter, y_label_);
    painter.restore();

    QPainterPath path;
    path.moveTo(sx(points_.front().first), sy(points_.front().second));
    for (const auto& p : points_) path.lineTo(sx(p.first), sy(p.second));
    const QColor accent = palette().highlight().color();
    painter.setPen(QPen(accent, 2));
    painter.drawPath(path);
    painter.setBrush(accent);
    for (const auto& p : points_) painter.drawEllipse(QPointF(sx(p.first), sy(p.second)), 3.5, 3.5);
}

}  // namespace ohs::gui
