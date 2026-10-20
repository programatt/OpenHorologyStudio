// SPDX-License-Identifier: Apache-2.0
// The OpenGL way of drawing the viewport. The triangles of the visible parts go into one vertex buffer,
// with their face normals and colours, and are rebuilt only when the scene or the colours change. Orbit,
// pan and zoom then cost one matrix: the graphics card projects, shades and depth-tests (so there is no
// sorting), where the painter projects and sorts every triangle on the CPU for every frame.
#include <cmath>
#include <cstring>
#include <numbers>

#include <QByteArray>
#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QMetaObject>
#include <QOffscreenSurface>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPainter>
#include <QResizeEvent>
#include <QSettings>
#include <QSurfaceFormat>

#include "gui/viewport.hpp"

namespace ohs::gui {

namespace {

// OpenGL 2.1 / GLSL 1.20: what the oldest graphics cards still in use understand.
const char* const kVertexShader = R"(#version 120
uniform mat4 mvp;
uniform mat3 rot;
attribute vec3 position;
attribute vec3 normal;
attribute vec4 colour;
varying vec4 shaded;
void main() {
    gl_Position = mvp * vec4(position, 1.0);
    // The same light as the painter's: from the upper left, in view space, lighting both sides of a face.
    float shade = 0.35 + 0.65 * abs(dot(rot * normal, vec3(-0.4, 0.5, 0.77)));
    shaded = vec4(colour.rgb * shade, 1.0);
}
)";

const char* const kFragmentShader = R"(#version 120
uniform vec4 flat_colour;
varying vec4 shaded;
void main() {
    gl_FragColor = flat_colour.a > 0.0 ? flat_colour : shaded;
}
)";

struct Vertex {
    float position[3];
    float normal[3];
    unsigned char colour[4];
};
static_assert(sizeof(Vertex) == 28);

QString setting_name(Renderer r) {
    switch (r) {
        case Renderer::Software: return "software";
        case Renderer::OpenGL: return "opengl";
        default: return "auto";
    }
}

Renderer parse_renderer(const QString& text, Renderer otherwise) {
    const QString t = text.trimmed().toLower();
    if (t == "software" || t == "cpu" || t == "painter") return Renderer::Software;
    if (t == "opengl" || t == "gl" || t == "gpu") return Renderer::OpenGL;
    if (t == "auto") return Renderer::Auto;
    return otherwise;
}

}  // namespace

Renderer Viewport::saved_renderer() {
    const QByteArray env = qgetenv("OHS_RENDERER");
    const Renderer saved = parse_renderer(QSettings().value("viewport/renderer", "auto").toString(), Renderer::Auto);
    return env.isEmpty() ? saved : parse_renderer(QString::fromLatin1(env), saved);
}

void Viewport::save_renderer(Renderer renderer) {
    QSettings().setValue("viewport/renderer", setting_name(renderer));
}

const Viewport::GpuInfo& Viewport::gpu() {
    static const GpuInfo info = [] {
        GpuInfo out;
        QOpenGLContext context;
        if (!context.create()) return out;
        QOffscreenSurface surface;
        surface.setFormat(context.format());
        surface.create();
        if (!surface.isValid() || !context.makeCurrent(&surface)) return out;
        const auto text = [&context](unsigned name) {
            const auto* s = reinterpret_cast<const char*>(context.functions()->glGetString(name));
            return s ? QString::fromLatin1(s) : QString();
        };
        const QString renderer = text(0x1F01 /* GL_RENDERER */), vendor = text(0x1F00 /* GL_VENDOR */);
        out.name = renderer.isEmpty() ? vendor : renderer;
        const QSurfaceFormat f = context.format();
        out.usable = f.majorVersion() > 2 || (f.majorVersion() == 2 && f.minorVersion() >= 1);
        // A software rasteriser draws on the CPU just as the painter does, and slower.
        const QString lower = (renderer + " " + vendor).toLower();
        out.hardware = out.usable && !lower.contains("llvmpipe") && !lower.contains("softpipe") && !lower.contains("swrast") &&
                       !lower.contains("software") && !lower.contains("swiftshader") && !lower.contains("basic render");
        context.doneCurrent();
        return out;
    }();
    return info;
}

/// The widget over the viewport that holds the OpenGL context. It takes no mouse events: the viewport under
/// it keeps handling them.
class Viewport::GlSurface : public QOpenGLWidget {
public:
    explicit GlSurface(Viewport* view) : QOpenGLWidget(view), view_(view) {
        QSurfaceFormat format;
        format.setDepthBufferSize(24);
        format.setSamples(4);
        setFormat(format);
        setAttribute(Qt::WA_TransparentForMouseEvents);
    }

    ~GlSurface() override {
        makeCurrent();
        buffer_.destroy();
        program_.reset();
        doneCurrent();
    }

    bool failed() const { return failed_; }

protected:
    void initializeGL() override {
        gl_ = QOpenGLContext::currentContext()->functions();
        gl_->initializeOpenGLFunctions();
        program_ = std::make_unique<QOpenGLShaderProgram>();
        const bool ok = program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) &&
                        program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) &&
                        program_->link() && buffer_.create();
        built_ = false;
        if (!ok) {
            qWarning("viewport: the OpenGL shaders did not build (%s); drawing with the painter", qPrintable(program_->log()));
            failed_ = true;
            QMetaObject::invokeMethod(view_, [v = view_] { v->gl_failed(); }, Qt::QueuedConnection);
        }
    }

    void paintGL() override {
        QPainter painter(this);
        if (failed_ || !view_->scene_) {
            view_->draw(painter, size(), false);  // the empty-state text, or the painter's picture
            if (!failed_) view_->draw_notes(painter);
            return;
        }
        painter.beginNativePainting();
        draw_scene();
        painter.endNativePainting();
        view_->draw_decorations(painter, size());
        view_->draw_notes(painter);
    }

private:
    void upload() {
        const Viewport& v = *view_;
        const Scene& scene = *v.scene_;
        const Scene::Drawn& drawn = v.drawn();
        const int selected = v.selected_index();
        std::vector<QColor> colours(scene.parts().size());
        for (std::size_t i = 0; i < colours.size(); ++i) colours[i] = v.part_colour(static_cast<int>(i), selected);
        const FieldOverlay* field = scene.field();
        const Point3 c = scene.centre();

        std::vector<Vertex> vertices;
        vertices.reserve(drawn.triangles.size() * 3);
        selected_runs_.clear();
        for (std::size_t i = 0; i < drawn.triangles.size(); ++i) {
            const Triangle& t = drawn.triangles[i];
            const double ux = t.b[0] - t.a[0], uy = t.b[1] - t.a[1], uz = t.b[2] - t.a[2];
            const double vx = t.c[0] - t.a[0], vy = t.c[1] - t.a[1], vz = t.c[2] - t.a[2];
            double n[3] = {uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
            const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (len < 1e-12) continue;
            const int part = drawn.part_index[i];
            QColor base = colours[static_cast<std::size_t>(part)];
            const double value = (drawn.values[i][0] + drawn.values[i][1] + drawn.values[i][2]) / 3.0;
            if (field && !std::isnan(value)) {
                const auto f = field_colour((value - field->lo) / (field->hi - field->lo));
                base = QColor(f[0], f[1], f[2]);
            }
            Vertex vert{};
            for (int k = 0; k < 3; ++k) vert.normal[k] = static_cast<float>(n[k] / len);
            vert.colour[0] = static_cast<unsigned char>(base.red());
            vert.colour[1] = static_cast<unsigned char>(base.green());
            vert.colour[2] = static_cast<unsigned char>(base.blue());
            vert.colour[3] = 255;
            if (part == selected) {
                const int first = static_cast<int>(vertices.size());
                if (!selected_runs_.empty() && selected_runs_.back().first + selected_runs_.back().second == first) selected_runs_.back().second += 3;
                else selected_runs_.emplace_back(first, 3);
            }
            for (const Point3* p : {&t.a, &t.b, &t.c}) {
                for (int k = 0; k < 3; ++k) vert.position[k] = static_cast<float>((*p)[k] - c[k]);  // about the centre: floats keep their precision
                vertices.push_back(vert);
            }
        }
        buffer_.bind();
        buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        buffer_.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(Vertex)));
        buffer_.release();
        count_ = static_cast<int>(vertices.size());
        built_ = true;
        scene_ = view_->scene_.get();
        revision_ = scene.revision();
        look_ = view_->look_;
    }

    void draw_scene() {
        constexpr unsigned GL_COLOR_BUFFER_BIT_ = 0x4000, GL_DEPTH_BUFFER_BIT_ = 0x100, GL_DEPTH_TEST_ = 0x0B71, GL_CULL_FACE_ = 0x0B44,
                           GL_BLEND_ = 0x0BE2, GL_TRIANGLES_ = 0x0004, GL_FLOAT_ = 0x1406, GL_UNSIGNED_BYTE_ = 0x1401,
                           GL_SRC_ALPHA_ = 0x0302, GL_ONE_MINUS_SRC_ALPHA_ = 0x0303;
        const Viewport& v = *view_;
        const Scene& scene = *v.scene_;
        if (!built_ || scene_ != v.scene_.get() || revision_ != scene.revision() || look_ != v.look_) upload();

        const double dpr = devicePixelRatioF();
        gl_->glViewport(0, 0, static_cast<int>(width() * dpr), static_cast<int>(height() * dpr));
        const QColor back = v.backdrop();
        gl_->glClearColor(static_cast<float>(back.redF()), static_cast<float>(back.greenF()), static_cast<float>(back.blueF()), 1.0f);
        gl_->glClear(GL_COLOR_BUFFER_BIT_ | GL_DEPTH_BUFFER_BIT_);
        if (count_ == 0) return;

        // The painter's projection, as a matrix: rotate about Z by yaw, tilt about X by pitch, scale, pan.
        const double w = width(), h = height();
        const double scale = 0.45 * std::min(w, h) / scene.radius() * v.zoom_;
        const double yaw = v.yaw_ * std::numbers::pi / 180.0, pitch = v.pitch_ * std::numbers::pi / 180.0;
        const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
        const double depth = scene.radius() * (4.0 + 4.0 * scene.explode());
        const float rot[9] = {static_cast<float>(cy),      static_cast<float>(-sy),     0.0f,
                              static_cast<float>(cp * sy), static_cast<float>(cp * cy), static_cast<float>(-sp),
                              static_cast<float>(sp * sy), static_cast<float>(sp * cy), static_cast<float>(cp)};
        const QMatrix3x3 rotation(rot);
        const float sx = static_cast<float>(2.0 * scale / w), syy = static_cast<float>(2.0 * scale / h), sz = static_cast<float>(-1.0 / depth);
        // clang-format off
        const QMatrix4x4 mvp(sx * rot[0],  sx * rot[1],  sx * rot[2],  static_cast<float>(2.0 * v.pan_x_ / w),
                             syy * rot[3], syy * rot[4], syy * rot[5], static_cast<float>(-2.0 * v.pan_y_ / h),
                             sz * rot[6],  sz * rot[7],  sz * rot[8],  0.0f,
                             0.0f,         0.0f,         0.0f,         1.0f);
        // clang-format on

        program_->bind();
        program_->setUniformValue("mvp", mvp);
        program_->setUniformValue("rot", rotation);
        program_->setUniformValue("flat_colour", QVector4D(0, 0, 0, 0));
        buffer_.bind();
        const int position = program_->attributeLocation("position"), normal = program_->attributeLocation("normal"),
                  colour = program_->attributeLocation("colour");
        program_->enableAttributeArray(position);
        program_->enableAttributeArray(normal);
        program_->enableAttributeArray(colour);
        gl_->glVertexAttribPointer(position, 3, GL_FLOAT_, 0, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, position)));
        gl_->glVertexAttribPointer(normal, 3, GL_FLOAT_, 0, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, normal)));
        gl_->glVertexAttribPointer(colour, 4, GL_UNSIGNED_BYTE_, 1, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, colour)));

        gl_->glEnable(GL_DEPTH_TEST_);
        gl_->glDisable(GL_CULL_FACE_);  // the depth test hides the far side of a solid; open surfaces show both sides
        gl_->glDrawArrays(GL_TRIANGLES_, 0, count_);

        // X-ray: a faint tint of the selected part over everything, wherever it is hidden.
        if (!selected_runs_.empty()) {
            program_->setUniformValue("flat_colour", QVector4D(40 / 255.0f, 200 / 255.0f, 235 / 255.0f, 60 / 255.0f));
            gl_->glDisable(GL_DEPTH_TEST_);
            gl_->glEnable(GL_BLEND_);
            gl_->glBlendFunc(GL_SRC_ALPHA_, GL_ONE_MINUS_SRC_ALPHA_);
            for (const auto& [first, n] : selected_runs_) gl_->glDrawArrays(GL_TRIANGLES_, first, n);
            gl_->glDisable(GL_BLEND_);
        }
        program_->disableAttributeArray(position);
        program_->disableAttributeArray(normal);
        program_->disableAttributeArray(colour);
        buffer_.release();
        program_->release();
        gl_->glDisable(GL_DEPTH_TEST_);
    }

    Viewport* view_;
    QOpenGLFunctions* gl_{};
    std::unique_ptr<QOpenGLShaderProgram> program_;
    QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};
    int count_{0};
    std::vector<std::pair<int, int>> selected_runs_;  ///< first vertex and vertex count of the selected part's triangles
    bool built_{false};
    bool failed_{false};
    const Scene* scene_{};
    std::uint64_t revision_{0}, look_{0};
};

void Viewport::update() {
    QWidget::update();
    if (gl_on_) gl_->update();
}

void Viewport::set_renderer(Renderer renderer) {
    renderer_ = renderer;
    bool want = false;
    if (renderer == Renderer::OpenGL) want = gpu().usable;
    else if (renderer == Renderer::Auto) want = gpu().hardware;
    if (want && !gl_) {
        gl_ = new GlSurface(this);
        gl_->setGeometry(rect());
    }
    gl_on_ = want;
    if (gl_) gl_->setVisible(want);
    update();
}

void Viewport::gl_failed() {
    gl_on_ = false;
    if (gl_) gl_->hide();
    update();
}

void Viewport::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    if (gl_) gl_->setGeometry(rect());
}

void Viewport::changeEvent(QEvent* e) {
    QWidget::changeEvent(e);
    if (e->type() == QEvent::PaletteChange) {
        ++look_;  // the backdrop the faded parts blend into follows the palette
        update();
    }
}

}  // namespace ohs::gui
