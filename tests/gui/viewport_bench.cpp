// SPDX-License-Identifier: Apache-2.0
// Frames per second of the viewport with each renderer, on the demonstration movement or a project given on
// the command line: orbiting (only the view changes) and animating (the pose changes every frame).
// Needs a display with a graphics card: `ohs_viewport_bench [project.hsproj]`. Not a test.
#include <cstdio>

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QOpenGLWidget>

#include "gui/main_window.hpp"
#include "gui/theme.hpp"

namespace {

using ohs::gui::Renderer;

double frames_per_second(ohs::gui::Viewport& view, bool animate, int frames) {
    QElapsedTimer timer;
    timer.start();
    for (int i = 0; i < frames; ++i) {
        view.set_view(35 + i * 1.7, -55 + (i % 40) * 0.5);
        if (animate) view.scene()->set_time(i * 0.2);
        view.repaint();
        QApplication::processEvents();
    }
    // The graphics card works behind the CPU: read the last frame back so that its work is counted.
    if (auto* surface = view.findChild<QOpenGLWidget*>(); surface && view.using_opengl()) (void)surface->grabFramebuffer();
    return frames * 1000.0 / static_cast<double>(timer.elapsed());
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("Open Horology Studio bench");
    QApplication::setOrganizationName("Open Horology Studio bench");
    ohs::gui::apply_theme(app);
    ohs::gui::MainWindow window;
    window.resize(1500, 900);
    window.show();
    const bool opened = argc > 1 ? window.open_project(argv[1]) : window.open_demo("viewport-bench");
    if (!opened) return 1;
    QApplication::processEvents();
    auto& view = *window.viewport();
    std::size_t triangles = 0;
    for (const auto& p : view.scene()->parts()) triangles += p.triangles.size();
    std::printf("%zu parts, %zu triangles, viewport %dx%d\n", view.scene()->parts().size(), triangles, view.width(), view.height());
    const auto& gpu = ohs::gui::Viewport::gpu();
    std::printf("gpu: %s (usable %d, hardware %d)\n", qPrintable(gpu.name), gpu.usable, gpu.hardware);
    for (const auto& [name, renderer] : {std::pair{"software", Renderer::Software}, std::pair{"opengl", Renderer::OpenGL}}) {
        view.set_renderer(renderer);
        QApplication::processEvents();
        (void)frames_per_second(view, false, 5);  // warm up: context, shaders, buffers
        std::printf("%-8s orbit %7.1f fps   animate %7.1f fps%s\n", name, frames_per_second(view, false, 120), frames_per_second(view, true, 60),
                    renderer == Renderer::OpenGL && !view.using_opengl() ? "   (fell back to the painter)" : "");
    }
    // OHS_BENCH_IMAGES=dir: also keep a frame from each renderer, to compare by eye.
    if (const QString dir = qEnvironmentVariable("OHS_BENCH_IMAGES"); !dir.isEmpty()) {
        view.set_view(35, -55);
        view.set_renderer(Renderer::Software);
        view.grab().save(dir + "/software.png");
        view.set_renderer(Renderer::OpenGL);
        QApplication::processEvents();
        view.repaint();
        if (auto* surface = view.findChild<QOpenGLWidget*>()) surface->grabFramebuffer().save(dir + "/opengl.png");
    }
    return 0;
}
