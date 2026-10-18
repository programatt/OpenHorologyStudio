// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>

#include <QImage>
#include <QWidget>

#include "gui/scene.hpp"

namespace ohs::gui {

/// The 3D viewport. It draws the scene with Qt's painter (shaded, depth-sorted triangles), so it
/// works on every platform and without a GPU, including the offscreen platform the tests use.
/// Left drag orbits, right or middle drag pans, the wheel zooms, a click picks a part.
class Viewport : public QWidget {
    Q_OBJECT
public:
    explicit Viewport(QWidget* parent = nullptr);

    void set_scene(std::shared_ptr<Scene> scene);
    Scene* scene() const { return scene_.get(); }

    void set_view(double yaw_deg, double pitch_deg);
    void fit();
    void select(const std::string& part);
    const std::string& selected() const { return selected_; }

    /// Name of the part under `pos` (widget coordinates), or an empty string.
    std::string pick(const QPoint& pos) const;
    /// Renders the current view; used for tests and image export.
    QImage render(const QSize& size) const;

signals:
    void part_picked(const QString& name);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    struct Projected {
        double x[3], y[3];
        double depth;
        int part;
        double shade;
    };
    std::vector<Projected> project(const QSize& size) const;
    void draw(QPainter& painter, const QSize& size) const;

    std::shared_ptr<Scene> scene_;
    std::string selected_;
    double yaw_{35}, pitch_{-55};  ///< degrees; pitch 0 looks straight down the dial axis, -90 is edge-on
    double zoom_{1.0};
    double pan_x_{0}, pan_y_{0};
    QPoint last_;
    QPoint press_;
    bool dragged_{false};
};

}  // namespace ohs::gui
