// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QImage>
#include <QTimer>
#include <QWidget>

#include "gui/scene.hpp"

namespace ohs::gui {

/// How the viewport draws. Software is Qt's painter (shaded, depth-sorted triangles): it works on every
/// platform and without a GPU, including the offscreen platform the tests use. OpenGL puts the triangles in
/// a vertex buffer and lets the graphics card project, shade and depth-test them, which keeps a large
/// movement smooth while orbiting. Auto picks OpenGL only when a hardware GPU is there to use.
enum class Renderer { Auto, Software, OpenGL };

/// The 3D viewport (see Renderer for how it draws). Left drag orbits, right or middle drag pans, the wheel
/// zooms, a click picks a part. Snapshots and video (render()) always use the software path, so they come
/// out the same on every machine.
class Viewport : public QWidget {
    Q_OBJECT
public:
    explicit Viewport(QWidget* parent = nullptr);

    ~Viewport() override;

    /// The renderer chosen in the settings (the OHS_RENDERER environment variable - auto, software or opengl -
    /// wins over it), and storing a choice.
    static Renderer saved_renderer();
    static void save_renderer(Renderer renderer);
    /// What this machine offers: whether an OpenGL context can be made at all, whether it is a hardware GPU
    /// (not llvmpipe or another software rasteriser, which is slower than the painter), and its name. Probed
    /// once.
    struct GpuInfo {
        bool usable{false};
        bool hardware{false};
        QString name;
    };
    static const GpuInfo& gpu();
    /// Switches the renderer now. OpenGL falls back to the painter when no context can be made.
    void set_renderer(Renderer renderer);
    Renderer renderer() const { return renderer_; }
    /// True while the picture is drawn by OpenGL.
    bool using_opengl() const { return gl_on_; }
    /// Repaints whichever surface draws the scene.
    void update();

    void set_scene(std::shared_ptr<Scene> scene);
    Scene* scene() const { return scene_.get(); }

    void set_view(double yaw_deg, double pitch_deg);
    void fit();
    void select(const std::string& part);
    /// Repaints for a change that comes in a stream (the timeline playing, a slider being dragged): the
    /// frame is drawn quickly without antialiasing, and once the stream stops the view is drawn again at
    /// full quality.
    void interactive_update();
    const std::string& selected() const { return selected_; }
    /// Colours each mobile of the train and draws the rig over the parts: a line for every gear mesh (green
    /// where the barrel feeds it, grey-blue where it does not reach, dashed where you declared it, amber where
    /// the confidence is low), dotted amber lines for meshes worth declaring, and a ring round every part that
    /// needs a look.
    void set_show_rig(bool on);
    /// Says on screen what the next click does (empty: nothing special); the cursor becomes a cross meanwhile.
    void set_prompt(const QString& text);
    bool show_rig() const { return show_rig_; }

    /// What the Rig tab's walk through the train shows. The parts walked so far are drawn as usual and everything
    /// else is faded back; the mesh being looked at is bright, with an arrow from the part that drives to the one
    /// driven. Parts are indexed as the scene's parts are (the movement's components).
    struct WalkView {
        std::vector<char> walked;      ///< per part: 1 for the parts of the mobiles walked so far
        int driver{-1};                ///< the current step's parts; `driven` is -1 at a gap
        int driven{-1};
        bool gap{false};               ///< the chain stops at `driver`: it is drawn amber, with no arrow
        int preview_a{-1}, preview_b{-1};  ///< a pair that could fill the gap, drawn with a dotted line
    };
    void set_walk(std::optional<WalkView> walk);
    const std::optional<WalkView>& walk() const { return walk_; }
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }
    double zoom() const { return zoom_; }
    QPointF pan() const { return {pan_x_, pan_y_}; }
    /// Where a part is drawn (the mean of its visible triangles), in widget coordinates; empty when the
    /// part is hidden or unknown.
    std::optional<QPointF> screen_position(const std::string& part) const;

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
    void resizeEvent(QResizeEvent*) override;
    void changeEvent(QEvent*) override;

private:
    class GlSurface;
    friend class GlSurface;
    /// The triangles of the visible parts in world space, worked out again only when the scene changes: orbiting,
    /// panning and zooming reuse them.
    const Scene::Drawn& drawn() const;
    QColor backdrop() const;
    /// The colour a part is drawn in before shading: its role, the rig and walk overlays, or the selection.
    QColor part_colour(int part, int selected) const;
    int selected_index() const;
    void draw_decorations(QPainter& painter, const QSize& size) const;  ///< overlays and the colour legend
    void draw_notes(QPainter& painter) const;                           ///< the pick prompt and the controls hint, on screen only
    void gl_failed();                                                   ///< the context could not be used: back to the painter

    struct Projected {
        double x[3], y[3];
        double depth;
        int part;
        double shade;
        double value;  ///< field value at the triangle (mean of its corners), NaN without a field
    };
    std::vector<Projected> project(const QSize& size) const;
    /// `fast` fills the triangles without antialiasing or outlines: about a ninth of the time, with jagged
    /// silhouettes, for frames that are about to be replaced.
    void draw(QPainter& painter, const QSize& size, bool fast = false) const;
    void draw_legend(QPainter& painter, const QSize& size, const FieldOverlay& field) const;
    void draw_overlays(QPainter& painter, const QSize& size) const;  ///< force arrows and motion trails
    void draw_rig(QPainter& painter, const QSize& size) const;       ///< meshes, suggestions and warnings
    void draw_walk(QPainter& painter, const QSize& size) const;      ///< the mesh being walked, and a preview
    QPointF to_screen(const Point3& p, const QSize& size) const;

    std::shared_ptr<Scene> scene_;
    std::string selected_;
    double yaw_{35}, pitch_{-55};  ///< degrees; pitch 0 looks straight down the dial axis, -90 is edge-on, -180 looks up at the back
    double zoom_{1.0};
    double pan_x_{0}, pan_y_{0};
    QPoint last_;
    QPoint press_;
    bool dragged_{false};
    /// What a drag does, decided where it starts: on a part it turns the movement, off it it pans.
    enum class Drag { None, Turn, Pan };
    Drag drag_{Drag::None};
    bool show_rig_{false};
    QString prompt_;
    std::optional<WalkView> walk_;
    bool fast_{false};          ///< the next paint is an interactive one
    QTimer* settle_{};          ///< draws at full quality once the interaction pauses
    Renderer renderer_{Renderer::Auto};
    GlSurface* gl_{};           ///< made the first time OpenGL is wanted
    bool gl_on_{false};
    std::uint64_t look_{0};     ///< moves when anything that changes the colours does
    mutable Scene::Drawn drawn_;
    mutable std::uint64_t drawn_revision_{0};
    mutable bool drawn_valid_{false};
};

}  // namespace ohs::gui
