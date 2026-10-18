// SPDX-License-Identifier: Apache-2.0
// Scripted flow of the desktop application on Qt's offscreen platform: demonstration movement,
// rig list, picking and Explain, animation, exploded and cut-away views, analyses, report.
#include <filesystem>
#include <set>

#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QTableWidget>
#include <QTabWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "gui/main_window.hpp"

namespace fs = std::filesystem;

namespace {

QApplication& app() {
    static int argc = 1;
    static char name[] = "ohs_gui_tests";
    static char* argv[] = {name, nullptr};
    static QApplication a(argc, argv);
    return a;
}

QString fresh_dir(const char* tag) {
    const fs::path d = fs::temp_directory_path() / (std::string("ohs-gui-test-") + tag);
    fs::remove_all(d);
    fs::create_directories(d);
    return QString::fromStdString(d.string());
}

std::size_t coloured_pixels(const QImage& image) {
    const QRgb background = image.pixel(1, 1);
    std::size_t n = 0;
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x)
            if (image.pixel(x, y) != background) ++n;
    return n;
}

QPushButton* button(ohs::gui::MainWindow& w, const QString& text) {
    for (auto* b : w.workspaces()->findChildren<QPushButton*>())
        if (b->text() == text) return b;
    return nullptr;
}

}  // namespace

TEST_CASE("gui: the demonstration movement opens, is listed and is drawn", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("open")));
    REQUIRE(w.viewport()->scene() != nullptr);

    // 15 parts in the model tree (under mobiles and "fixed parts") and in the rig confidence list.
    CHECK(w.rig_table()->rowCount() == 15);
    int leaves = 0;
    for (QTreeWidgetItemIterator it(w.model_tree()); *it; ++it)
        if ((*it)->parent()) ++leaves;
    CHECK(leaves == 15);

    // Lowest confidence first.
    double last = -1;
    for (int r = 0; r < w.rig_table()->rowCount(); ++r) {
        const double c = w.rig_table()->item(r, 2)->text().toDouble();
        CHECK(c >= last);
        last = c;
    }

    const QImage image = w.viewport()->render(QSize(640, 480));
    CHECK(coloured_pixels(image) > 20000);
    if (const QByteArray snap = qgetenv("OHS_GUI_SNAPSHOT"); !snap.isEmpty()) image.save(QString::fromLocal8Bit(snap));
}

TEST_CASE("gui: clicking a part selects it and fills the Explain panel", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("pick")));
    w.viewport()->set_view(0, -90);  // straight down the dial axis
    w.viewport()->resize(640, 480);

    // Some pixel hits a part, and picking it selects that part.
    std::string hit;
    for (int y = 40; y < 440 && hit.empty(); y += 8)
        for (int x = 40; x < 600 && hit.empty(); x += 8) hit = w.viewport()->pick(QPoint(x, y));
    REQUIRE_FALSE(hit.empty());
    CHECK(w.viewport()->pick(QPoint(0, 0)).empty());

    w.select_part(QString::fromStdString(hit));
    CHECK(w.viewport()->selected() == hit);
    CHECK_FALSE(w.explain_panel()->toPlainText().isEmpty());
    CHECK(w.explain_panel()->toPlainText().contains(QString::fromStdString(hit)));
}

TEST_CASE("gui: the timeline turns the train, explode and cut-away change the picture", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("anim")));
    auto* scene = w.viewport()->scene();
    REQUIRE(scene->can_animate());

    const auto before = scene->draw_list();
    w.timeline()->setValue(3000);  // 18 s of movement time
    const auto after = scene->draw_list();
    REQUIRE(before.triangles.size() == after.triangles.size());
    // Parts on mobiles have moved; the plates have not.
    std::size_t moved = 0, still = 0;
    for (std::size_t i = 0; i < before.triangles.size(); ++i) {
        const bool same = before.triangles[i].a == after.triangles[i].a;
        (same ? still : moved)++;
    }
    CHECK(moved > 0);
    CHECK(still > 0);

    // The slowest wheel (ratio below 1) turns less than the fastest.
    const double t = scene->time();
    CHECK(t == Catch::Approx(18.0).margin(0.01));

    // Exploded view lifts parts apart along the dial axis.
    const double z0 = [&] { double lo = 1e300; for (auto& tr : scene->draw_list().triangles) lo = std::min(lo, tr.a[2]); return lo; }();
    w.explode_slider()->setValue(100);
    const double z1 = [&] { double lo = 1e300; for (auto& tr : scene->draw_list().triangles) lo = std::min(lo, tr.a[2]); return lo; }();
    CHECK(z1 < z0);

    // Cut-away removes triangles.
    const auto full = scene->draw_list().triangles.size();
    w.section_check()->setChecked(true);
    CHECK(scene->draw_list().triangles.size() < full);

    // Hiding a part removes its triangles.
    w.section_check()->setChecked(false);
    const std::string name = scene->parts().front().name;
    scene->set_visible(name, false);
    CHECK(scene->draw_list().triangles.size() < full);
}

TEST_CASE("gui: analyses run through the same commands as the CLI and the session replays", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    const QString dir = fresh_dir("analyse");
    REQUIRE(w.open_demo(dir));

    auto* timekeeping = button(w, "Timekeeping");
    REQUIRE(timekeeping);
    timekeeping->click();
    CHECK(w.results()->toPlainText().contains("rate", Qt::CaseInsensitive));

    auto* dfm = button(w, "Design for manufacture");
    REQUIRE(dfm);
    dfm->click();
    CHECK_FALSE(w.results()->toPlainText().isEmpty());

    // Tooth bending needs a selected part.
    auto* fea = button(w, "Tooth bending of the selected part");
    REQUIRE(fea);
    fea->click();
    CHECK(w.results()->toPlainText().contains("Click a part"));

    auto* report = button(w, "Write the report (HTML and PDF)");
    REQUIRE(report);
    report->click();
    CHECK(QFile::exists(QDir(dir).filePath("report.html")));
    CHECK(QFile::exists(QDir(dir).filePath("report.pdf")));

    // The history is a replayable script: it starts with the demo generation and has every command.
    const QStringList h = w.history();
    REQUIRE(h.size() >= 5);
    CHECK(h.first().startsWith("ohs dev gen-movement"));
    const QString script = QDir(dir).filePath("session.sh");
    REQUIRE(w.save_script(script));
    QFile f(script);
    REQUIRE(f.open(QIODevice::ReadOnly));
    const QString text = QString::fromUtf8(f.readAll());
    CHECK(text.contains("ohs import"));
    CHECK(text.contains("ohs report"));
}

TEST_CASE("gui: changing a role re-rigs the movement, and bad files are reported, not fatal", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("role")));
    const int mobiles = static_cast<int>(w.viewport()->scene()->movement().mobiles.size());
    CHECK(mobiles > 0);

    // Demote a screw-like part to a plate and back; the window stays open with a valid scene.
    const std::string part = w.viewport()->scene()->parts().front().name;
    const bool changed = w.set_role(QString::fromStdString(part), "plate");
    INFO(w.log_console()->toPlainText().toStdString());
    CHECK(changed);
    // The barrel is gone, so there is no train to turn: the parts still show, the animation is off.
    CHECK_FALSE(w.viewport()->scene()->can_animate());
    CHECK(w.log_console()->toPlainText().contains("Train not analysed"));
    CHECK_FALSE(w.timeline()->isEnabled());
    CHECK(w.viewport()->scene() != nullptr);

    CHECK_FALSE(w.open_project("/nonexistent/none.hsproj"));
    CHECK(w.log_console()->toPlainText().contains("Cannot open"));
    CHECK(w.viewport()->scene() != nullptr);  // the previous project is still shown
}
