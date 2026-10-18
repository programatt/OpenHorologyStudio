// SPDX-License-Identifier: Apache-2.0
// ohs-gui [project.hsproj] [--snapshot out.png] [--view top|iso|side] [--explode 0..1] [--time seconds]
// With --snapshot the window opens the project, renders the viewport to the file and exits (0 when it
// worked, 1 when the project could not be opened), which also serves scripted checks.
#include <QApplication>
#include <QCommandLineParser>
#include <QTimer>

#include "gui/main_window.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("Open Horology Studio");

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("project", "Project file to open (optional)");
    QCommandLineOption snapshot("snapshot", "Render the viewport to this PNG and exit", "file");
    QCommandLineOption view("view", "top, iso or side (with --snapshot)", "name", "iso");
    QCommandLineOption explode("explode", "Exploded view 0..1 (with --snapshot)", "factor", "0");
    QCommandLineOption time("time", "Movement time in seconds (with --snapshot)", "seconds", "0");
    QCommandLineOption step("step", "STEP file (default: next to the project)", "file");
    parser.addOptions({snapshot, view, explode, time, step});
    parser.process(app);

    ohs::gui::MainWindow window;
    const QStringList args = parser.positionalArguments();
    if (parser.isSet(snapshot)) {
        if (args.isEmpty() || !window.open_project(args.first(), parser.value(step))) return 1;
        const QString v = parser.value(view);
        window.viewport()->set_view(v == "top" ? 0 : v == "side" ? 0 : 35, v == "top" ? 0 : v == "side" ? -90 : -55);
        if (auto* scene = window.viewport()->scene()) {
            scene->set_explode(parser.value(explode).toDouble());
            scene->set_time(parser.value(time).toDouble());
        }
        return window.viewport()->render(QSize(1000, 800)).save(parser.value(snapshot)) ? 0 : 1;
    }
    window.show();
    if (!args.isEmpty()) window.open_project(args.first(), parser.value(step));
    return app.exec();
}
