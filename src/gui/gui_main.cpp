// SPDX-License-Identifier: Apache-2.0
// ohs-gui [project.hsproj] [--snapshot out.png] [--view top|bottom|iso|side] [--explode 0..1] [--time seconds]
//         [--video out.mp4 [--video-seconds 10] [--movement-seconds 60] [--fps 30] [--size 1280x720]]
// With --snapshot the window opens the project, renders the viewport to the file and exits (0 when it
// worked, 1 when the project could not be opened), which also serves scripted checks. --video does the
// same for an animation encoded with ffmpeg.
#include <QApplication>
#include <QCommandLineParser>
#include <QTimer>

#include "gui/main_window.hpp"
#include "gui/theme.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("Open Horology Studio");
    QApplication::setOrganizationName("Open Horology Studio");  // where the window layout is remembered
    ohs::gui::apply_theme(app);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("project", "Project file to open (optional)");
    QCommandLineOption snapshot("snapshot", "Render the viewport to this PNG and exit", "file");
    QCommandLineOption view("view", "top, bottom, iso or side (with --snapshot)", "name", "iso");
    QCommandLineOption explode("explode", "Exploded view 0..1 (with --snapshot)", "factor", "0");
    QCommandLineOption time("time", "Movement time in seconds (with --snapshot)", "seconds", "0");
    QCommandLineOption step("step", "STEP file (default: next to the project)", "file");
    QCommandLineOption video("video", "Render the animation to this video file (needs ffmpeg) and exit", "file");
    QCommandLineOption video_seconds("video-seconds", "Length of the video (with --video)", "seconds", "10");
    QCommandLineOption movement_seconds("movement-seconds", "Movement time the video covers (with --video)", "seconds", "60");
    QCommandLineOption fps("fps", "Frames per second (with --video)", "n", "30");
    QCommandLineOption size("size", "Video size (with --video)", "WxH", "1280x720");
    QCommandLineOption field("field", "Plot a result on a part: file.vtu,part[,von_mises|displacement] (with --snapshot or --video)", "spec");
    QCommandLineOption forces("forces", "Draw the tooth force arrows (with --snapshot or --video)");
    QCommandLineOption trails("trails", "Draw motion trails covering this many seconds of movement time (with --snapshot or --video)", "seconds");
    parser.addOptions({snapshot, view, explode, time, step, video, video_seconds, movement_seconds, fps, size, field, forces, trails});
    parser.process(app);

    ohs::gui::MainWindow window;
    const QStringList args = parser.positionalArguments();
    if (parser.isSet(snapshot)) {
        if (args.isEmpty() || !window.open_project(args.first(), parser.value(step))) return 1;
        const QString v = parser.value(view);
        window.viewport()->set_view(v == "top" || v == "side" || v == "bottom" ? 0 : 35, v == "top" ? 0 : v == "side" ? -90 : v == "bottom" ? -180 : -55);
        if (auto* scene = window.viewport()->scene()) {
            scene->set_explode(parser.value(explode).toDouble());
            scene->set_time(parser.value(time).toDouble());
        }
        if (parser.isSet(field)) {
            const QStringList f = parser.value(field).split(',');
            if (f.size() < 2 || !window.show_field(f[0], f[1], f.value(2, "von_mises"))) return 1;
        }
        if (auto* scene = window.viewport()->scene()) {
            if (parser.isSet(forces)) {
                try {
                    scene->compute_forces();
                    scene->set_show_forces(true);
                } catch (const std::exception& e) {
                    qWarning("no force arrows: %s", e.what());
                }
            }
            if (parser.isSet(trails)) scene->set_show_trails(true, parser.value(trails).toDouble());
        }
        return window.viewport()->render(QSize(1000, 800)).save(parser.value(snapshot)) ? 0 : 1;
    }
    if (parser.isSet(video)) {
        if (args.isEmpty() || !window.open_project(args.first(), parser.value(step))) return 1;
        const QString v = parser.value(view);
        window.viewport()->set_view(v == "top" || v == "side" || v == "bottom" ? 0 : 35, v == "top" ? 0 : v == "side" ? -90 : v == "bottom" ? -180 : -55);
        if (auto* scene = window.viewport()->scene()) scene->set_explode(parser.value(explode).toDouble());
        if (parser.isSet(field)) {
            const QStringList f = parser.value(field).split(',');
            if (f.size() < 2 || !window.show_field(f[0], f[1], f.value(2, "von_mises"))) return 1;
        }
        if (auto* scene = window.viewport()->scene()) {
            if (parser.isSet(forces)) {
                try {
                    scene->compute_forces();
                    scene->set_show_forces(true);
                } catch (const std::exception& e) {
                    qWarning("no force arrows: %s", e.what());
                }
            }
            if (parser.isSet(trails)) scene->set_show_trails(true, parser.value(trails).toDouble());
        }
        ohs::gui::VideoOptions options;
        options.video_seconds = parser.value(video_seconds).toDouble();
        options.movement_seconds = parser.value(movement_seconds).toDouble();
        options.fps = parser.value(fps).toInt();
        const QStringList wh = parser.value(size).split('x');
        if (wh.size() == 2) options.size = QSize(wh[0].toInt(), wh[1].toInt());
        const QString error = window.export_video(parser.value(video), options);
        if (!error.isEmpty()) qWarning("%s", qPrintable(error));
        return error.isEmpty() ? 0 : 1;
    }
    window.show();
    if (!args.isEmpty()) window.open_project(args.first(), parser.value(step));
    return app.exec();
}
