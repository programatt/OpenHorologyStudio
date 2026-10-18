// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <utility>
#include <vector>

#include <QString>
#include <QWidget>

namespace ohs::gui {

/// A small line plot with labelled axes and marked points, drawn with Qt's painter.
class PlotWidget : public QWidget {
    Q_OBJECT
public:
    explicit PlotWidget(QWidget* parent = nullptr);

    void set_data(const QString& title, const QString& x_label, const QString& y_label,
                  std::vector<std::pair<double, double>> points);
    void clear();
    const std::vector<std::pair<double, double>>& points() const { return points_; }
    QString title() const { return title_; }

    QImage render(const QSize& size) const;

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void draw(QPainter& painter, const QSize& size) const;

    QString title_, x_label_, y_label_;
    std::vector<std::pair<double, double>> points_;
};

}  // namespace ohs::gui
