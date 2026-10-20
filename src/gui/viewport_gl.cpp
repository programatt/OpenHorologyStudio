// SPDX-License-Identifier: Apache-2.0
// The OpenGL way of drawing the viewport. The triangles of every part go into one vertex buffer, in the
// part's own frame with their face normals, once per load. A frame is then a model matrix and a colour per
// part (a few hundred uniforms): the turn of a mobile, the swing of a lever, the exploded view and the
// cut-away plane are all done by the graphics card, which also projects, shades and depth-tests (so there is
// no sorting). The painter projects and sorts every triangle on the CPU for every frame; an earlier version
// of this renderer rebuilt and uploaded the whole buffer whenever the pose changed.
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
uniform mat4 mvp;           // view * model of this part
uniform mat3 rot;           // view rotation * the part's rotation, for the normals
uniform vec4 clip;          // the cut-away plane in the part's frame: dot(clip.xyz, position) > clip.w is cut away
uniform vec4 part_colour;
uniform float vertex_colour;  // 1: the colours come with the vertices (a result field)
attribute vec3 position;
attribute vec3 normal;
attribute vec4 colour;
varying vec4 shaded;
varying float cut;
void main() {
    gl_Position = mvp * vec4(position, 1.0);
    // The same light as the painter's: from the upper left, in view space, lighting both sides of a face.
    float shade = 0.35 + 0.65 * abs(dot(rot * normal, vec3(-0.4, 0.5, 0.77)));
    vec4 base = vertex_colour > 0.5 ? colour : part_colour;
    shaded = vec4(base.rgb * shade, 1.0);
    cut = dot(clip.xyz, position) - clip.w;
}
)";

const char* const kFragmentShader = R"(#version 120
uniform vec4 flat_colour;
varying vec4 shaded;
varying float cut;
void main() {
    if (cut > 0.0) discard;
    gl_FragColor = flat_colour.a > 0.0 ? flat_colour : shaded;
}
)";

/// A part's triangle corner, still in the part's own frame; the colour comes with the part.
struct StaticVertex {
    float position[3];
    float normal[3];
};
static_assert(sizeof(StaticVertex) == 24);

/// The same with a colour of its own: the triangles of a result field.
struct FieldVertex {
    float position[3];
    float normal[3];
    unsigned char colour[4];
};
static_assert(sizeof(FieldVertex) == 28);

bool face_normal(const Triangle& t, float (&out)[3]) {
    const double ux = t.b[0] - t.a[0], uy = t.b[1] - t.a[1], uz = t.b[2] - t.a[2];
    const double vx = t.c[0] - t.a[0], vy = t.c[1] - t.a[1], vz = t.c[2] - t.a[2];
    const double n[3] = {uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len < 1e-12) return false;
    for (int k = 0; k < 3; ++k) out[k] = static_cast<float>(n[k] / len);
    return true;
}

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
        field_buffer_.destroy();
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
                        program_->link() && buffer_.create() && field_buffer_.create();
        built_ = false;
        field_built_ = false;
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
    // Every part's triangles, once: they only change with a load, or a rig edit that moves a pivot.
    void upload_parts() {
        const Scene& scene = *view_->scene_;
        std::vector<StaticVertex> vertices;
        std::size_t total = 0;
        for (const ScenePart& part : scene.parts()) total += part.triangles.size() * 3;
        vertices.reserve(total);
        ranges_.assign(scene.parts().size(), {0, 0});
        for (std::size_t i = 0; i < scene.parts().size(); ++i) {
            const int first = static_cast<int>(vertices.size());
            for (const Triangle& t : scene.parts()[i].triangles) {
                StaticVertex v{};
                if (!face_normal(t, v.normal)) continue;
                for (const Point3* p : {&t.a, &t.b, &t.c}) {
                    for (int k = 0; k < 3; ++k) v.position[k] = static_cast<float>((*p)[k]);
                    vertices.push_back(v);
                }
            }
            ranges_[i] = {first, static_cast<int>(vertices.size()) - first};
        }
        buffer_.bind();
        buffer_.setUsagePattern(QOpenGLBuffer::StaticDraw);
        buffer_.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(StaticVertex)));
        buffer_.release();
        built_ = true;
        scene_ = view_->scene_.get();
        geometry_revision_ = scene.geometry_revision();
    }

    // The result field of one part, with its colours, in the part's frame.
    void upload_field() {
        const Scene& scene = *view_->scene_;
        const FieldOverlay* field = scene.field();
        std::vector<FieldVertex> vertices;
        field_count_ = 0;
        field_part_ = -1;
        if (field) {
            for (std::size_t i = 0; i < scene.parts().size(); ++i) {
                if (scene.parts()[i].name == field->part) field_part_ = static_cast<int>(i);
            }
            vertices.reserve(field->triangles.size() * 3);
            for (std::size_t k = 0; k < field->triangles.size(); ++k) {
                const Triangle& t = field->triangles[k];
                FieldVertex v{};
                if (!face_normal(t, v.normal)) continue;
                const double value = (field->values[k][0] + field->values[k][1] + field->values[k][2]) / 3.0;
                const auto f = field_colour((value - field->lo) / (field->hi - field->lo));
                v.colour[0] = static_cast<unsigned char>(f[0]);
                v.colour[1] = static_cast<unsigned char>(f[1]);
                v.colour[2] = static_cast<unsigned char>(f[2]);
                v.colour[3] = 255;
                for (const Point3* p : {&t.a, &t.b, &t.c}) {
                    for (int c = 0; c < 3; ++c) v.position[c] = static_cast<float>((*p)[c]);
                    vertices.push_back(v);
                }
            }
        }
        field_buffer_.bind();
        field_buffer_.setUsagePattern(QOpenGLBuffer::StaticDraw);
        field_buffer_.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(FieldVertex)));
        field_buffer_.release();
        field_count_ = static_cast<int>(vertices.size());
        field_built_ = true;
        field_serial_ = scene.field_serial();
    }

    void draw_scene() {
        constexpr unsigned GL_COLOR_BUFFER_BIT_ = 0x4000, GL_DEPTH_BUFFER_BIT_ = 0x100, GL_DEPTH_TEST_ = 0x0B71, GL_CULL_FACE_ = 0x0B44,
                           GL_BLEND_ = 0x0BE2, GL_TRIANGLES_ = 0x0004, GL_FLOAT_ = 0x1406, GL_UNSIGNED_BYTE_ = 0x1401,
                           GL_SRC_ALPHA_ = 0x0302, GL_ONE_MINUS_SRC_ALPHA_ = 0x0303;
        const Viewport& v = *view_;
        const Scene& scene = *v.scene_;
        if (!built_ || scene_ != v.scene_.get() || geometry_revision_ != scene.geometry_revision()) {
            upload_parts();
            field_built_ = false;
        }
        if (!field_built_ || field_serial_ != scene.field_serial()) upload_field();

        const double dpr = devicePixelRatioF();
        gl_->glViewport(0, 0, static_cast<int>(width() * dpr), static_cast<int>(height() * dpr));
        const QColor back = v.backdrop();
        gl_->glClearColor(static_cast<float>(back.redF()), static_cast<float>(back.greenF()), static_cast<float>(back.blueF()), 1.0f);
        gl_->glClear(GL_COLOR_BUFFER_BIT_ | GL_DEPTH_BUFFER_BIT_);

        // The painter's projection, as a matrix: rotate about Z by yaw, tilt about X by pitch, scale, pan.
        const double w = width(), h = height();
        const double scale = 0.45 * std::min(w, h) / scene.radius() * v.zoom_;
        const double yaw = v.yaw_ * std::numbers::pi / 180.0, pitch = v.pitch_ * std::numbers::pi / 180.0;
        const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
        const double depth = scene.radius() * (4.0 + 4.0 * scene.explode());
        const double rot[9] = {cy, -sy, 0.0, cp * sy, cp * cy, -sp, sp * sy, sp * cy, cp};
        const float sx = static_cast<float>(2.0 * scale / w), syy = static_cast<float>(2.0 * scale / h), sz = static_cast<float>(-1.0 / depth);
        // clang-format off
        const QMatrix4x4 view(sx * static_cast<float>(rot[0]),  sx * static_cast<float>(rot[1]),  sx * static_cast<float>(rot[2]),  static_cast<float>(2.0 * v.pan_x_ / w),
                              syy * static_cast<float>(rot[3]), syy * static_cast<float>(rot[4]), syy * static_cast<float>(rot[5]), static_cast<float>(-2.0 * v.pan_y_ / h),
                              sz * static_cast<float>(rot[6]),  sz * static_cast<float>(rot[7]),  sz * static_cast<float>(rot[8]),  0.0f,
                              0.0f, 0.0f, 0.0f, 1.0f);
        // clang-format on

        const std::vector<Scene::PartPose> poses = scene.part_poses();
        const int selected = v.selected_index();
        const Point3 centre = scene.centre();
        const SectionPlane& cut = scene.section();

        program_->bind();
        const int position = program_->attributeLocation("position"), normal = program_->attributeLocation("normal"),
                  colour = program_->attributeLocation("colour");
        const int u_mvp = program_->uniformLocation("mvp"), u_rot = program_->uniformLocation("rot"),
                  u_clip = program_->uniformLocation("clip"), u_part = program_->uniformLocation("part_colour"),
                  u_vertex = program_->uniformLocation("vertex_colour"), u_flat = program_->uniformLocation("flat_colour");
        program_->setUniformValue(u_flat, QVector4D(0, 0, 0, 0));

        // The part's pose: [R | t] to world, the world about the centre, then the view.
        auto place = [&](const Scene::PartPose& pose) {
            const double* m = pose.m.data();
            const QMatrix4x4 model(static_cast<float>(m[0]), static_cast<float>(m[1]), static_cast<float>(m[2]), static_cast<float>(m[3] - centre[0]),
                                   static_cast<float>(m[4]), static_cast<float>(m[5]), static_cast<float>(m[6]), static_cast<float>(m[7] - centre[1]),
                                   static_cast<float>(m[8]), static_cast<float>(m[9]), static_cast<float>(m[10]), static_cast<float>(m[11] - centre[2]),
                                   0.0f, 0.0f, 0.0f, 1.0f);
            program_->setUniformValue(u_mvp, view * model);
            float nrot[9];  // the view's rotation times the part's
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    nrot[3 * r + c] = static_cast<float>(rot[3 * r] * m[c] + rot[3 * r + 1] * m[4 + c] + rot[3 * r + 2] * m[8 + c]);
                }
            }
            program_->setUniformValue(u_rot, QMatrix3x3(nrot));
            if (cut.enabled) {
                // Cut where n . (R p + t) > offset, that is (R^T n) . p > offset - n . t, in the part's own frame.
                const auto& n = cut.normal;
                const double rn[3] = {m[0] * n[0] + m[4] * n[1] + m[8] * n[2], m[1] * n[0] + m[5] * n[1] + m[9] * n[2],
                                      m[2] * n[0] + m[6] * n[1] + m[10] * n[2]};
                const double nt = n[0] * m[3] + n[1] * m[7] + n[2] * m[11];
                program_->setUniformValue(u_clip, QVector4D(static_cast<float>(rn[0]), static_cast<float>(rn[1]), static_cast<float>(rn[2]),
                                                            static_cast<float>(cut.offset - nt)));
            } else {
                program_->setUniformValue(u_clip, QVector4D(0, 0, 0, 1));
            }
        };

        gl_->glEnable(GL_DEPTH_TEST_);
        gl_->glDisable(GL_CULL_FACE_);  // the depth test hides the far side of a solid; open surfaces show both sides

        auto bind_static = [&] {
            buffer_.bind();
            program_->enableAttributeArray(position);
            program_->enableAttributeArray(normal);
            program_->disableAttributeArray(colour);
            gl_->glVertexAttribPointer(position, 3, GL_FLOAT_, 0, sizeof(StaticVertex), reinterpret_cast<const void*>(offsetof(StaticVertex, position)));
            gl_->glVertexAttribPointer(normal, 3, GL_FLOAT_, 0, sizeof(StaticVertex), reinterpret_cast<const void*>(offsetof(StaticVertex, normal)));
            program_->setUniformValue(u_vertex, 0.0f);
        };
        auto bind_field = [&] {
            field_buffer_.bind();
            program_->enableAttributeArray(position);
            program_->enableAttributeArray(normal);
            program_->enableAttributeArray(colour);
            gl_->glVertexAttribPointer(position, 3, GL_FLOAT_, 0, sizeof(FieldVertex), reinterpret_cast<const void*>(offsetof(FieldVertex, position)));
            gl_->glVertexAttribPointer(normal, 3, GL_FLOAT_, 0, sizeof(FieldVertex), reinterpret_cast<const void*>(offsetof(FieldVertex, normal)));
            gl_->glVertexAttribPointer(colour, 4, GL_UNSIGNED_BYTE_, 1, sizeof(FieldVertex), reinterpret_cast<const void*>(offsetof(FieldVertex, colour)));
            program_->setUniformValue(u_vertex, 1.0f);
        };

        bind_static();
        for (std::size_t i = 0; i < poses.size() && i < ranges_.size(); ++i) {
            if (!poses[i].visible || ranges_[i].second == 0 || static_cast<int>(i) == field_part_) continue;
            const QColor c = v.part_colour(static_cast<int>(i), selected);
            program_->setUniformValue(u_part, QVector4D(static_cast<float>(c.redF()), static_cast<float>(c.greenF()), static_cast<float>(c.blueF()), 1.0f));
            place(poses[i]);
            gl_->glDrawArrays(GL_TRIANGLES_, ranges_[i].first, ranges_[i].second);
        }
        if (field_part_ >= 0 && field_count_ > 0 && poses[static_cast<std::size_t>(field_part_)].visible) {
            bind_field();
            place(poses[static_cast<std::size_t>(field_part_)]);
            gl_->glDrawArrays(GL_TRIANGLES_, 0, field_count_);
            bind_static();
        }

        // X-ray: a faint tint of the selected part over everything, wherever it is hidden.
        if (selected >= 0 && static_cast<std::size_t>(selected) < poses.size() && poses[static_cast<std::size_t>(selected)].visible) {
            program_->setUniformValue(u_flat, QVector4D(40 / 255.0f, 200 / 255.0f, 235 / 255.0f, 60 / 255.0f));
            gl_->glDisable(GL_DEPTH_TEST_);
            gl_->glEnable(GL_BLEND_);
            gl_->glBlendFunc(GL_SRC_ALPHA_, GL_ONE_MINUS_SRC_ALPHA_);
            place(poses[static_cast<std::size_t>(selected)]);
            if (selected == field_part_ && field_count_ > 0) {
                bind_field();
                gl_->glDrawArrays(GL_TRIANGLES_, 0, field_count_);
            } else if (static_cast<std::size_t>(selected) < ranges_.size() && ranges_[static_cast<std::size_t>(selected)].second > 0) {
                gl_->glDrawArrays(GL_TRIANGLES_, ranges_[static_cast<std::size_t>(selected)].first, ranges_[static_cast<std::size_t>(selected)].second);
            }
            gl_->glDisable(GL_BLEND_);
        }
        program_->disableAttributeArray(position);
        program_->disableAttributeArray(normal);
        program_->disableAttributeArray(colour);
        field_buffer_.release();
        buffer_.release();
        program_->release();
        gl_->glDisable(GL_DEPTH_TEST_);
    }

    Viewport* view_;
    QOpenGLFunctions* gl_{};
    std::unique_ptr<QOpenGLShaderProgram> program_;
    QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};        ///< every part's triangles, in the parts' frames
    QOpenGLBuffer field_buffer_{QOpenGLBuffer::VertexBuffer};  ///< the result field of one part
    std::vector<std::pair<int, int>> ranges_;  ///< per part: first vertex and vertex count in buffer_
    int field_count_{0};
    int field_part_{-1};
    bool built_{false}, field_built_{false};
    bool failed_{false};
    const Scene* scene_{};
    std::uint64_t geometry_revision_{0}, field_serial_{0};
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
