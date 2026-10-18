// SPDX-License-Identifier: Apache-2.0
#include <QApplication>

#include "gui/main_window.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("Open Horology Studio");
    ohs::gui::MainWindow window;
    window.show();
    if (argc > 1) window.open_project(QString::fromLocal8Bit(argv[1]));
    return app.exec();
}
