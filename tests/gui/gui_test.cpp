// SPDX-License-Identifier: Apache-2.0
// Scripted flow of the desktop application on Qt's offscreen platform: demonstration movement,
// rig list, picking and Explain, animation, exploded and cut-away views, analyses, report.
#include <algorithm>
#include <filesystem>
#include <memory>
#include <set>

#include <QAction>
#include <QApplication>
#include <QGuiApplication>
#include <QOpenGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QEventLoop>
#include <QFile>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QTableWidget>
#include <QLineEdit>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QLineEdit>
#include <QStatusBar>
#include <QTreeWidget>
#include <QListWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMenu>
#include <QUndoStack>
#include <QTreeWidgetItemIterator>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Pnt.hxx>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "dynamics/calibre.hpp"
#include "dynamics/torque_chain.hpp"
#include "fea/results.hpp"
#include "gui/field.hpp"
#include "gui/job_queue.hpp"
#include "kinematics/kinematics.hpp"
#include "mesh/tetmesh.hpp"
#include "gui/main_window.hpp"
#include "gui/theme.hpp"
#include "geometry/step_io.hpp"
#include "keyless_parts.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"

namespace fs = std::filesystem;

namespace {

QApplication& app() {
    static int argc = 1;
    static char name[] = "ohs_gui_tests";
    static char* argv[] = {name, nullptr};
    // Never destroyed: tearing a QApplication down at process exit crashes with some Qt versions (6.4).
    static QApplication* a = new QApplication(argc, argv);
    return *a;
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

TEST_CASE("gui: the renderer is a setting, falls back to the painter without a GPU, and OpenGL draws what the painter does", "[gui][renderer]") {
    app();
    using ohs::gui::Renderer;
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("renderer")));
    ohs::gui::Viewport& view = *w.viewport();
    ohs::gui::Viewport::save_renderer(Renderer::Software);
    REQUIRE(ohs::gui::Viewport::saved_renderer() == Renderer::Software);
    ohs::gui::Viewport::save_renderer(Renderer::OpenGL);
    REQUIRE(ohs::gui::Viewport::saved_renderer() == Renderer::OpenGL);
    ohs::gui::Viewport::save_renderer(Renderer::Auto);  // what a fresh install has

    view.set_renderer(Renderer::Software);
    REQUIRE_FALSE(view.using_opengl());
    const QImage painter = view.render(QSize(400, 300));
    CHECK(coloured_pixels(painter) > 1000);

    // Software never uses OpenGL, whatever the machine has; forcing OpenGL falls back when no context can be made.
    const auto& gpu = ohs::gui::Viewport::gpu();
    view.set_renderer(Renderer::OpenGL);
    CHECK(view.using_opengl() == gpu.usable);
    view.set_renderer(Renderer::Auto);
    CHECK(view.using_opengl() == gpu.hardware);
    // Snapshots and video do not depend on the renderer.
    CHECK(view.render(QSize(400, 300)) == painter);
    view.set_renderer(Renderer::Software);
    CHECK_FALSE(view.using_opengl());
}

// The GPU keeps the parts still and moves them with a matrix: the turn of the train, the exploded view, the cut-away
// plane and the selection must come out as the painter draws them. Needs a hardware GPU and a display (skipped without).
TEST_CASE("gui: OpenGL draws the turned, exploded, cut and selected movement as the painter does", "[gui][renderer][gl]") {
    app();
    if (!ohs::gui::Viewport::gpu().hardware || QGuiApplication::platformName() == "offscreen") {
        SUCCEED("no hardware GPU on a real display here: nothing to compare");
        return;
    }
    const QString dir = fresh_dir("gl-parity");
    const QString step = QDir(dir).filePath("demo.step"), project = QDir(dir).filePath("demo.hsproj");
    REQUIRE(ohs::cli::run_captured({"dev", "gen-movement", "-o", step.toStdString()}).exit_code == 0);
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"}).exit_code == 0);
    auto scene = std::make_shared<ohs::gui::Scene>(ohs::gui::Scene::load(project.toStdString()));
    ohs::gui::Viewport view;
    view.resize(480, 360);
    view.set_scene(scene);
    view.set_renderer(ohs::gui::Renderer::OpenGL);
    view.show();
    for (int i = 0; i < 20; ++i) QCoreApplication::processEvents();
    REQUIRE(view.using_opengl());
    auto* surface = view.findChild<QOpenGLWidget*>();
    REQUIRE(surface);

    // Pixels that differ clearly: edges differ a little between the antialiasing of the two.
    const auto differing = [](const QImage& a, const QImage& b) {
        REQUIRE(a.size() == b.size());
        const QImage x = a.convertToFormat(QImage::Format_RGB32), y = b.convertToFormat(QImage::Format_RGB32);
        int bad = 0;
        for (int row = 0; row < x.height(); ++row) {
            for (int col = 0; col < x.width(); ++col) {
                const QRgb p = x.pixel(col, row), q = y.pixel(col, row);
                if (std::max({std::abs(qRed(p) - qRed(q)), std::abs(qGreen(p) - qGreen(q)), std::abs(qBlue(p) - qBlue(q))}) > 48) ++bad;
            }
        }
        return bad / static_cast<double>(x.width() * x.height());
    };
    const auto compare = [&](const char* what) {
        view.repaint();
        const QImage gpu = surface->grabFramebuffer();
        const QImage painter = view.render(view.size());
        CHECK(coloured_pixels(gpu) > 1000);
        const double off = differing(gpu.size() == painter.size() ? gpu : gpu.scaled(painter.size()), painter);
        INFO(what << ": " << off * 100 << " % of the pixels differ");
        CHECK(off < 0.04);
        gpu.save(QDir(dir).filePath(QString("gl-%1.png").arg(what)));
        painter.save(QDir(dir).filePath(QString("painter-%1.png").arg(what)));
    };

    compare("rest");
    scene->set_time(5000.0);  // the train has turned
    compare("turned");
    scene->set_explode(0.8);
    compare("exploded");
    scene->set_explode(0.0);
    ohs::gui::SectionPlane cut;
    cut.enabled = true;
    cut.normal = {0, 0, 1};
    cut.offset = 0.5 * (scene->centre()[2] + scene->radius() * 0.0);
    scene->set_section(cut);
    compare("cut");
    scene->set_section({});
    view.select("escape_wheel");
    compare("selected");
}

TEST_CASE("gui: OpenGL draws a result field on its part", "[gui][renderer][gl]") {
    app();
    if (!ohs::gui::Viewport::gpu().hardware || QGuiApplication::platformName() == "offscreen") {
        SUCCEED("no hardware GPU on a real display here: nothing to compare");
        return;
    }
    const QString dir = fresh_dir("gl-field");
    const QString step = QDir(dir).filePath("demo.step"), project = QDir(dir).filePath("demo.hsproj");
    REQUIRE(ohs::cli::run_captured({"dev", "gen-movement", "-o", step.toStdString()}).exit_code == 0);
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"}).exit_code == 0);
    auto scene = std::make_shared<ohs::gui::Scene>(ohs::gui::Scene::load(project.toStdString()));
    ohs::gui::Viewport view;
    view.resize(480, 360);
    view.set_scene(scene);
    view.set_renderer(ohs::gui::Renderer::OpenGL);
    view.show();
    for (int i = 0; i < 20; ++i) QCoreApplication::processEvents();
    REQUIRE(view.using_opengl());
    auto* surface = view.findChild<QOpenGLWidget*>();
    REQUIRE(surface);
    view.repaint();
    const QImage plain = surface->grabFramebuffer();

    // A field laid on the escape wheel: the triangles of its own part, with a value that rises along x.
    const auto it = std::find_if(scene->parts().begin(), scene->parts().end(), [](const auto& p) { return p.name == "escape_wheel"; });
    REQUIRE(it != scene->parts().end());
    ohs::gui::FieldOverlay field;
    field.part = it->name;
    field.label = "test";
    field.lo = -3;
    field.hi = 3;
    field.triangles = it->triangles;
    for (const auto& t : field.triangles) field.values.push_back({t.a[0], t.b[0], t.c[0]});
    scene->set_field(std::move(field));
    view.repaint();
    const QImage with_field = surface->grabFramebuffer();
    CHECK(with_field != plain);  // the part is now coloured by the field, which differs from its role colour
    const double hidden = scene->field() ? 0.0 : 1.0;
    CHECK(hidden == 0.0);
    scene->clear_field();
    view.repaint();
    CHECK(surface->grabFramebuffer() == plain);  // and clearing it brings the part back exactly
}

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

    // The parts are solids, so their back faces are skipped; the picture is the one with every face drawn (a
    // cut-away above the whole movement turns the skipping off), up to antialiased edges seen edge-on.
    auto* scene = w.viewport()->scene();
    for (const auto& p : scene->parts()) CHECK(p.closed);
    scene->set_section({true, {0, 0, 1}, 1e9});
    const QImage every_face = w.viewport()->render(QSize(640, 480));
    scene->set_section({});
    int differ = 0;
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x) {
            const QRgb a = image.pixel(x, y), b = every_face.pixel(x, y);
            if (std::abs(qRed(a) - qRed(b)) + std::abs(qGreen(a) - qGreen(b)) + std::abs(qBlue(a) - qBlue(b)) > 72) ++differ;
        }
    CHECK(differ < image.width() * image.height() / 500);
}

TEST_CASE("gui: clicking a part selects it and fills the Explain panel", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("pick")));
    w.viewport()->set_view(0, 0);  // straight down the dial axis
    w.viewport()->resize(640, 480);

    // Some pixel hits a part, and picking it selects that part.
    std::string hit;
    for (int y = 40; y < 440 && hit.empty(); y += 8)
        for (int x = 40; x < 600 && hit.empty(); x += 8) hit = w.viewport()->pick(QPoint(x, y));
    REQUIRE_FALSE(hit.empty());
    CHECK(w.viewport()->pick(QPoint(0, 0)).empty());

    // Selecting does not wait for the analyses in the queue.
    w.submit("timekeeping", {"timekeeping", w.project()});
    w.select_part(QString::fromStdString(hit));
    CHECK(w.unfinished_jobs() == 1);
    CHECK(w.viewport()->selected() == hit);
    CHECK_FALSE(w.explain_panel()->toPlainText().isEmpty());
    CHECK(w.explain_panel()->toPlainText().contains(QString::fromStdString(hit)));

    // A selected part hidden behind others (the escape wheel, from above) still shows where it is: it is
    // tinted over everything.
    w.viewport()->set_view(35, -55);
    w.select_part("");
    const QImage none = w.viewport()->render(QSize(640, 480));
    w.select_part("escape_wheel");
    const QImage tinted = w.viewport()->render(QSize(640, 480));
    int changed = 0;
    for (int y = 0; y < none.height(); ++y)
        for (int x = 0; x < none.width(); ++x) changed += none.pixel(x, y) != tinted.pixel(x, y);
    CHECK(changed > 100);  // none without the tint: the wheel is wholly hidden from here
    w.select_part(QString::fromStdString(hit));

    // A command does not wait for the queue either: the job reads its own copy of the project.
    CHECK(w.run_command({"materials"}) == 0);
    CHECK(w.unfinished_jobs() == 1);
    w.wait_for_jobs();
    CHECK(w.run_command({"materials"}) == 0);
}

TEST_CASE("gui: the timeline turns the train, explode and cut-away change the picture", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("anim")));
    w.viewport()->set_renderer(ohs::gui::Renderer::Software);  // the picture compared below is the painter's, not a GPU's
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

    // A frame drawn while the timeline moves is the quick, aliased one; when it stops the view settles at
    // full quality, the same picture render() gives.
    auto* v = w.viewport();
    v->resize(320, 240);
    w.timeline()->setValue(3100);
    const QImage fast = v->grab().toImage().convertToFormat(QImage::Format_RGB32);
    QElapsedTimer settle;
    settle.start();
    while (settle.elapsed() < 400) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    const QImage settled = v->grab().toImage().convertToFormat(QImage::Format_RGB32);
    CHECK(fast != settled);
    // Up to the controls hint along the bottom edge, which is drawn on screen only.
    const QRect above_hint(0, 0, v->width(), v->height() - 40);
    CHECK(settled.copy(above_hint) == v->render(v->size()).convertToFormat(QImage::Format_RGB32).copy(above_hint));
    CHECK(settled.copy(QRect(0, v->height() - 40, v->width(), 40)) !=
          v->render(v->size()).convertToFormat(QImage::Format_RGB32).copy(QRect(0, v->height() - 40, v->width(), 40)));

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
    scene->set_visible(name, true);
    CHECK(scene->draw_list().triangles.size() == full);

    // So does unticking it in the model tree, and only that part is hidden.
    QTreeWidgetItem* item = nullptr;
    for (QTreeWidgetItemIterator it(w.model_tree()); *it && !item; ++it)
        if ((*it)->parent() && (*it)->text(0).toStdString() == name) item = *it;
    REQUIRE(item);
    item->setCheckState(0, Qt::Unchecked);
    CHECK_FALSE(scene->visible(name));
    for (const auto& p : scene->parts()) CHECK(scene->visible(p.name) == (p.name != name));
    item->setCheckState(0, Qt::Checked);
    CHECK(scene->visible(name));
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
    w.wait_for_jobs();
    CHECK(w.results()->toPlainText().contains("rate", Qt::CaseInsensitive));

    auto* dfm = button(w, "Design for manufacture");
    REQUIRE(dfm);
    dfm->click();
    w.wait_for_jobs();
    CHECK_FALSE(w.results()->toPlainText().isEmpty());

    // Tooth bending needs a selected part.
    auto* fea = button(w, "Tooth bending of the selected part");
    REQUIRE(fea);
    fea->click();
    CHECK(w.results()->toPlainText().contains("Click a part"));

    auto* report = button(w, "Write the report (HTML and PDF)");
    REQUIRE(report);
    report->click();
    w.wait_for_jobs();
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

    // Demote the escape wheel to a plate: the train has nothing to beat against, so the animation is off,
    // but the parts still show and the window stays open with a valid scene.
    REQUIRE(w.set_role("escape_wheel", "plate"));
    CHECK_FALSE(w.viewport()->scene()->can_animate());
    CHECK_FALSE(w.timeline()->isEnabled());
    CHECK(w.viewport()->scene() != nullptr);

    // Demote the barrel as well: now nothing says where the train starts, and the log says so.
    REQUIRE(w.set_role("barrel", "plate"));
    CHECK_FALSE(w.viewport()->scene()->can_animate());
    CHECK(w.log_console()->toPlainText().contains("Train not analysed"));
    CHECK(w.viewport()->scene() != nullptr);

    CHECK_FALSE(w.open_project("/nonexistent/none.hsproj"));
    CHECK(w.log_console()->toPlainText().contains("Cannot open"));
    CHECK(w.viewport()->scene() != nullptr);  // the previous project is still shown
}

TEST_CASE("gui: analyses run as queued jobs and the timekeeping plot is filled", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("jobs")));

    auto* tk = button(w, "Timekeeping");
    auto* tor = button(w, "Torque chain");
    REQUIRE(tk);
    REQUIRE(tor);
    tk->click();
    tor->click();
    // Timekeeping is one job (the report and its plot data come from the same run), then the torque chain.
    CHECK(w.job_list()->count() == 2);
    CHECK(w.unfinished_jobs() > 0);
    // The window stays usable while they run: a queued job can be cancelled before it starts.
    CHECK(w.cancel_job(1));
    w.wait_for_jobs();
    CHECK(w.unfinished_jobs() == 0);
    CHECK(w.job_list()->item(0)->text().startsWith("done"));
    CHECK(w.job_list()->item(1)->text().startsWith("cancelled"));

    // Rate against amplitude from the full, half and low wind.
    REQUIRE(w.plot()->points().size() == 3);
    CHECK(w.plot()->points().front().first < w.plot()->points().back().first);
    CHECK(coloured_pixels(w.plot()->render(QSize(400, 220))) > 1000);

    // Cancelled jobs never ran: the torque chain is not in the history.
    bool torque_ran = false;
    for (const QString& h : w.history()) torque_ran |= h.startsWith("ohs torque");
    CHECK_FALSE(torque_ran);
}

TEST_CASE("gui: role changes can be undone and redone", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("undo")));
    const auto role_of = [&](const QString& part) {
        const auto& m = w.viewport()->scene()->movement();
        return m.components[m.find_component(part.toStdString())].role;
    };
    const QString part = "escape_wheel";
    const auto original = role_of(part);
    REQUIRE(original != ohs::model::Role::Plate);
    const int mobiles = static_cast<int>(w.viewport()->scene()->movement().mobiles.size());

    REQUIRE(w.change_role(part, "plate"));
    CHECK(role_of(part) == ohs::model::Role::Plate);
    CHECK(w.undo_stack()->count() == 1);
    CHECK(w.undo_stack()->undoText().contains("escape_wheel"));

    w.undo_stack()->undo();
    CHECK(role_of(part) == original);
    CHECK(static_cast<int>(w.viewport()->scene()->movement().mobiles.size()) == mobiles);
    CHECK(w.viewport()->scene()->can_animate());

    w.undo_stack()->redo();
    CHECK(role_of(part) == ohs::model::Role::Plate);
    CHECK_FALSE(w.viewport()->scene()->can_animate());
    w.undo_stack()->undo();
    CHECK(w.viewport()->scene()->can_animate());

    // The same role again is not a change.
    REQUIRE(w.change_role(part, QString::fromUtf8(ohs::model::to_string(original).data())));
    CHECK(w.undo_stack()->count() == 1);
}

TEST_CASE("gui: several parts get a role in one edit, and one undo puts them all back", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("multi-role")));
    const auto role_of = [&](const QString& part) {
        const auto& m = w.viewport()->scene()->movement();
        return m.components[m.find_component(part.toStdString())].role;
    };
    const auto first = role_of("fourth_wheel"), second = role_of("center_wheel");
    REQUIRE(first != ohs::model::Role::Screw);

    // Selecting two rows of the part list and applying a role is one command and one undo step.
    for (int row = 0; row < w.rig_table()->rowCount(); ++row) {
        const QString name = w.rig_table()->item(row, 0)->data(Qt::UserRole).toString();
        if (name == "fourth_wheel" || name == "center_wheel") w.rig_table()->selectionModel()->select(w.rig_table()->model()->index(row, 0), QItemSelectionModel::Select | QItemSelectionModel::Rows);
    }
    CHECK(w.rig_table()->selectionModel()->selectedRows().size() == 2);
    const int history_before = w.history().size();
    REQUIRE(w.change_roles({"fourth_wheel", "center_wheel"}, "screw"));
    CHECK(role_of("fourth_wheel") == ohs::model::Role::Screw);
    CHECK(role_of("center_wheel") == ohs::model::Role::Screw);
    CHECK(w.undo_stack()->count() == 1);
    CHECK(w.history().size() == history_before + 1);

    w.undo_stack()->undo();
    CHECK(role_of("fourth_wheel") == first);
    CHECK(role_of("center_wheel") == second);
    // Parts that already have the role are left out: nothing to do is not an edit.
    REQUIRE(w.change_roles({"fourth_wheel", "center_wheel"}, QString::fromUtf8(ohs::model::to_string(first).data())) == true);
}

TEST_CASE("gui: a label replaces the part name in the model tree", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("label")));
    const auto m = [&]() -> const ohs::model::Movement& { return w.viewport()->scene()->movement(); };
    const QString part = "escape_wheel";
    REQUIRE(m().find_component(part.toStdString()) >= 0);

    REQUIRE(w.change_label(part, "  Escape  "));
    const auto& c = m().components[static_cast<std::size_t>(m().find_component(part.toStdString()))];
    CHECK(c.label == "Escape");
    CHECK(c.display_name() == "Escape");
    CHECK(w.undo_stack()->count() == 1);

    bool found = false;
    for (QTreeWidgetItemIterator it(w.findChild<QTreeWidget*>()); *it; ++it) {
        if ((*it)->data(0, Qt::UserRole).toString() == part) {
            found = true;
            CHECK((*it)->text(0) == "Escape");
            CHECK((*it)->toolTip(0) == part);
        }
    }
    CHECK(found);

    // The walk of the train names the part by its label too.
    REQUIRE(w.change_label("barrel", "Mainspring barrel"));
    const auto& walk = w.viewport()->scene()->review().walk;
    REQUIRE_FALSE(walk.steps.empty());
    CHECK(walk.steps.front().text.find("Mainspring barrel") != std::string::npos);
    w.undo_stack()->undo();

    w.undo_stack()->undo();
    const auto& back = m().components[static_cast<std::size_t>(m().find_component(part.toStdString()))];
    CHECK(back.label.empty());
    CHECK(back.display_name() == part.toStdString());
}

TEST_CASE("gui: a mesh can be left out of the train and put back, undoably", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("winding")));
    const auto connected = [&] { return w.viewport()->scene()->review().chain.connected; };
    REQUIRE(connected());

    REQUIRE(w.set_winding("barrel", "center_pinion", true));
    CHECK_FALSE(connected());
    CHECK(w.viewport()->scene()->review().walk.steps.front().kind == ohs::rigging::WalkStep::Kind::Gap);

    w.undo_stack()->undo();
    CHECK(connected());
    w.undo_stack()->redo();
    CHECK_FALSE(connected());
    REQUIRE(w.set_winding("barrel", "center_pinion", false));
    CHECK(connected());
}

TEST_CASE("gui: the part list can be searched, and a pick says what the next click does", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("search")));
    auto* table = w.rig_table();
    const auto visible_rows = [&] {
        int n = 0;
        for (int r = 0; r < table->rowCount(); ++r) n += table->isRowHidden(r) ? 0 : 1;
        return n;
    };
    REQUIRE(visible_rows() == table->rowCount());

    auto* filter = w.findChild<QLineEdit*>();
    for (auto* edit : w.findChildren<QLineEdit*>()) {
        if (edit->placeholderText().startsWith("Search parts")) filter = edit;
    }
    REQUIRE(filter);
    filter->setText("ESCAPE");  // case does not matter; matches the part name and the role
    CHECK(visible_rows() >= 1);
    CHECK(visible_rows() < table->rowCount());
    for (int r = 0; r < table->rowCount(); ++r) {
        if (!table->isRowHidden(r)) CHECK(table->item(r, 0)->data(Qt::UserRole).toString().contains("escape", Qt::CaseInsensitive));
    }
    filter->setText("no such part");
    CHECK(visible_rows() == 0);
    filter->clear();
    CHECK(visible_rows() == table->rowCount());

    // A pick shows a prompt and can be cancelled; nothing is declared.
    w.pick_partner_for("barrel");
    CHECK(w.is_picking());
    CHECK(w.statusBar()->currentMessage().contains("Esc"));
    CHECK(w.viewport()->cursor().shape() == Qt::CrossCursor);
    emit w.viewport()->part_picked(QString());  // a click beside the movement cancels
    CHECK_FALSE(w.is_picking());
    CHECK(w.viewport()->cursor().shape() != Qt::CrossCursor);
}

TEST_CASE("gui: solving the train reports the wheels of the hands and changes nothing when it is connected", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("solve")));
    REQUIRE(w.viewport()->scene()->review().chain.connected);
    const int edits = w.undo_stack()->count();
    CHECK(w.solve_train(false));
    CHECK(w.undo_stack()->count() == edits);
    CHECK(w.statusBar()->currentMessage().contains("connected"));
}

TEST_CASE("gui: the beat rate of the project can be set in the Rig tab, undoably", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("beat")));
    QComboBox* box = nullptr;
    for (auto* c : w.findChildren<QComboBox*>()) {
        if (c->isEditable() && c->lineEdit() && c->lineEdit()->placeholderText().startsWith("not set")) box = c;
    }
    REQUIRE(box);
    const auto rate = [&] { return w.viewport()->scene()->movement().beat_rate_vph; };
    const double before = rate();
    CHECK(box->lineEdit()->text().toDouble() == before);

    REQUIRE(w.set_beat_rate(25200));
    CHECK(rate() == 25200);
    CHECK(box->lineEdit()->text() == "25200");
    CHECK(w.undo_stack()->undoText().contains("25200"));
    w.undo_stack()->undo();
    CHECK(rate() == before);
    CHECK(box->lineEdit()->text().toDouble() == before);
    CHECK_FALSE(w.set_beat_rate(0));
}

TEST_CASE("gui: running an analysis shows the Results panel even when it was closed", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("results")));
    w.results_dock()->hide();
    REQUIRE_FALSE(w.results_dock()->isVisible());
    auto* run = button(w, "Timegrapher (60 s)");
    REQUIRE(run);
    run->click();
    w.wait_for_jobs();
    QCoreApplication::processEvents();
    CHECK(w.results_dock()->isVisible());
    CHECK(w.results()->toPlainText().contains("virtual timegrapher"));
    CHECK(w.statusBar()->currentMessage().contains("Results"));
}

TEST_CASE("gui: the timegrapher screen draws the traces beat by beat", "[gui][timegrapher]") {
    app();
    ohs::gui::TimegrapherScreen screen;
    CHECK_FALSE(screen.has_run());
    std::vector<ohs::gui::ScreenBeat> beats;
    for (int i = 0; i < 200; ++i) {  // 25200 vph: a beat every 0.1429 s, 6 s/d fast, ticks and tocks 0.4 ms apart
        beats.push_back({i, i * 0.142857, i % 2 == 1, 142.857, -0.0094 * i + (i % 2 ? 0.4 : 0.0), 3.1});
    }
    screen.set_run(beats, {6.0, 285.0, 0.4}, 25200, 52, "dial_up");
    REQUIRE(screen.has_run());
    CHECK(screen.duration() == Catch::Approx(199 * 0.142857).margin(1e-6));

    const QSize size(1000, 617);
    const QImage idle = screen.render(size, 0.0);
    const QImage early = screen.render(size, 0.4);
    const QImage later = screen.render(size, 12.0);
    CHECK(idle != early);
    CHECK(early != later);
    CHECK(idle.size() == size);
    // The blue screen is there: most of the LCD is blue-dominant.
    int blue = 0;
    for (int y = 140; y < 340; y += 6)
        for (int x = 220; x < 760; x += 6) {
            const QRgb c = later.pixel(x, y);
            blue += qBlue(c) > qRed(c) + 40;
        }
    CHECK(blue > 900);

    screen.set_time(0.4);
    const int few = screen.beats_shown();
    screen.set_time(12.0);
    CHECK(screen.beats_shown() > few);
    CHECK(ohs::gui::TimegrapherScreen::position_code("crown_left") == "CL");
    CHECK(ohs::gui::TimegrapherScreen::position_code("dial_down") == "DD");
}

TEST_CASE("gui: a timegrapher run shows the screen and still prints the text", "[gui][timegrapher]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("tgscreen")));
    REQUIRE_FALSE(w.timegrapher()->has_run());
    auto* run = button(w, "Timegrapher (60 s)");
    REQUIRE(run);
    run->click();
    w.wait_for_jobs();
    QCoreApplication::processEvents();
    CHECK(w.results()->toPlainText().contains("virtual timegrapher"));
    REQUIRE(w.timegrapher()->has_run());
    CHECK(w.timegrapher()->isVisibleTo(&w));
    CHECK(w.timegrapher()->beats_shown() >= 0);
}

TEST_CASE("gui: a mesh declared by hand can be removed, undoably", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("unmesh")));
    const auto meshes = [&] { return w.viewport()->scene()->movement().meshes.size(); };
    const std::size_t before = meshes();
    REQUIRE(w.declare_mesh("barrel", "fourth_wheel"));
    CHECK(meshes() == before + 1);
    REQUIRE(w.remove_mesh("barrel", "fourth_wheel"));
    CHECK(meshes() == before);
    w.undo_stack()->undo();
    CHECK(meshes() == before + 1);
    CHECK_FALSE(w.remove_mesh("barrel", "center_pinion"));  // found from the geometry
}

TEST_CASE("job queue: jobs run in worker processes, failures and bad programs are reported", "[gui][jobs]") {
    app();
    ohs::gui::JobQueue q("/bin/sh");
    const int ok = q.submit("echo", {"-c", "echo hello; echo oops >&2; echo $$"});
    const int bad = q.submit("exit 3", {"-c", "exit 3"});
    q.wait();
    const auto& jobs = q.jobs();
    CHECK(jobs[ok].state == ohs::gui::JobQueue::State::Done);
    CHECK(jobs[ok].out.startsWith("hello\n"));
    CHECK(jobs[ok].err == "oops\n");
    CHECK(jobs[ok].out.trimmed().split('\n').last().toLongLong() != QCoreApplication::applicationPid());  // another process
    CHECK(jobs[bad].state == ohs::gui::JobQueue::State::Failed);
    CHECK(jobs[bad].exit_code == 3);

    ohs::gui::JobQueue missing("/nonexistent/ohs");
    const int m = missing.submit("nothing to run", {"--version"});
    missing.wait();
    CHECK(missing.jobs()[m].state == ohs::gui::JobQueue::State::Failed);
    CHECK(missing.jobs()[m].err.contains("cannot start"));
}

TEST_CASE("job queue: a running job can be cancelled, and the queue carries on", "[gui][jobs]") {
    app();
    ohs::gui::JobQueue q("/bin/sh");
    const int slow = q.submit("slow", {"-c", "sleep 60"});
    const int next = q.submit("next", {"-c", "echo after"});
    const int never = q.submit("never", {"-c", "echo skipped"});
    REQUIRE(q.jobs()[slow].state == ohs::gui::JobQueue::State::Running);
    CHECK(q.cancel(never));  // still queued
    QElapsedTimer clock;
    clock.start();
    CHECK(q.cancel(slow));  // running: the worker is killed
    q.wait();
    CHECK(clock.elapsed() < 10000);
    CHECK(q.jobs()[slow].state == ohs::gui::JobQueue::State::Cancelled);
    CHECK(q.jobs()[next].state == ohs::gui::JobQueue::State::Done);
    CHECK(q.jobs()[next].out == "after\n");
    CHECK(q.jobs()[never].state == ohs::gui::JobQueue::State::Cancelled);
    CHECK_FALSE(q.cancel(next));  // finished jobs cannot be cancelled
}

TEST_CASE("gui: the demonstration movement is built once, then opened from its cache, as imported", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    const QString dir = fresh_dir("demo-twice");
    auto wait_loaded = [&] {
        QElapsedTimer t;
        t.start();
        while (w.is_loading() && t.elapsed() < 300000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    };
    w.open_demo_async(dir);
    wait_loaded();
    REQUIRE(w.viewport()->scene());
    CHECK(QFileInfo::exists(QDir(dir).filePath("demo.hsproj.ohscache/parts.brep")));
    CHECK(QFileInfo::exists(QDir(dir).filePath("demo.hsproj.ohscache/mesh.bin")));
    const auto role_of = [&](const QString& part) {
        const auto& m = w.viewport()->scene()->movement();
        return m.components[m.find_component(part.toStdString())].role;
    };
    const auto original = role_of("escape_wheel");
    REQUIRE(w.change_role("escape_wheel", "plate"));
    CHECK(role_of("escape_wheel") == ohs::model::Role::Plate);

    // The second open runs no commands (nothing is generated or imported) and gives the movement as it was imported.
    const int commands_before = w.history().size();
    QElapsedTimer t;
    t.start();
    w.open_demo_async(dir);
    wait_loaded();
    WARN("the demonstration movement opened again in " << t.elapsed() << " ms");
    CHECK(w.history().size() == commands_before);
    CHECK(role_of("escape_wheel") == original);
}

TEST_CASE("gui: an import can be cancelled between parts, and reports its progress", "[gui]") {
    app();
    const QString dir = fresh_dir("cancel-import");
    const QString step = QDir(dir).filePath("demo.step"), project = QDir(dir).filePath("demo.hsproj");
    REQUIRE(ohs::cli::run_captured({"dev", "gen-movement", "-o", step.toStdString()}).exit_code == 0);

    std::atomic<std::size_t> last{0}, total{0};
    ohs::cli::ImportHooks hooks;
    hooks.progress = [&](std::size_t done, std::size_t n) {
        last = done;
        total = n;
    };
    ohs::cli::set_import_hooks(hooks);
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString()}).exit_code == 0);
    CHECK(total > 5);
    CHECK(last == total);

    std::atomic<bool> stop{true};
    hooks.cancel = &stop;
    ohs::cli::set_import_hooks(hooks);
    QFile::remove(project);
    const auto cancelled = ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString()});
    ohs::cli::set_import_hooks({});
    CHECK(cancelled.exit_code != 0);
    CHECK(cancelled.err.find("cancelled") != std::string::npos);
    CHECK_FALSE(QFileInfo::exists(project));  // no half-made project is left behind
}

TEST_CASE("job queue: a quick job does not wait behind a long one when two may run at once", "[gui][jobs]") {
    app();
    ohs::gui::JobQueue q("/bin/sh");
    q.set_max_parallel(2);
    const int slow = q.submit("slow", {"-c", "sleep 60"});
    const int quick = q.submit("quick", {"-c", "echo quick"});
    const int third = q.submit("third", {"-c", "echo third"});
    CHECK(q.jobs()[slow].state == ohs::gui::JobQueue::State::Running);
    CHECK(q.jobs()[quick].state == ohs::gui::JobQueue::State::Running);
    CHECK(q.jobs()[third].state == ohs::gui::JobQueue::State::Queued);  // only two at once
    QElapsedTimer clock;
    clock.start();
    while (q.jobs()[third].state != ohs::gui::JobQueue::State::Done && clock.elapsed() < 20000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    CHECK(q.jobs()[quick].state == ohs::gui::JobQueue::State::Done);
    CHECK(q.jobs()[third].out == "third\n");  // started when the quick one ended, with the slow one still running
    CHECK(q.jobs()[slow].state == ohs::gui::JobQueue::State::Running);
    CHECK(q.cancel(slow));
    q.wait();
    CHECK(q.jobs()[slow].state == ohs::gui::JobQueue::State::Cancelled);
}

TEST_CASE("job queue: jobs run in a resident daemon, stream their output, are cancelled by closing, and a crash leaves it up", "[gui][jobs]") {
    app();
    const QString dir = fresh_dir("job-daemon");
    const QString socket = QDir(dir).filePath("d.sock");
    QProcess daemon;
    daemon.start(ohs::gui::MainWindow::worker_program(), {"serve", "--socket", socket});
    REQUIRE(daemon.waitForStarted());
    QElapsedTimer wait;
    wait.start();
    while (!QFileInfo::exists(socket) && wait.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    REQUIRE(QFileInfo::exists(socket));

    ohs::gui::JobQueue q(ohs::gui::MainWindow::worker_program());
    q.set_daemon(socket);
    q.set_max_parallel(2);
    using S = ohs::gui::JobQueue::State;

    const int version = q.submit("version", {"--version"});
    q.wait();
    CHECK(q.jobs()[static_cast<std::size_t>(version)].state == S::Done);
    CHECK(q.jobs()[static_cast<std::size_t>(version)].out.startsWith("1."));

    // Output arrives while the job runs: the first line is there long before the job ends.
    const int sleeper = q.submit("sleep", {"dev", "sleep", "30"});
    wait.restart();
    while (q.jobs()[static_cast<std::size_t>(sleeper)].out.isEmpty() && wait.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    CHECK(q.jobs()[static_cast<std::size_t>(sleeper)].out.contains("sleeping"));
    CHECK(q.jobs()[static_cast<std::size_t>(sleeper)].state == S::Running);
    // Cancelling closes the connection; the job's process in the daemon dies, so a second job gets its slot at once.
    CHECK(q.cancel(sleeper));
    CHECK(q.jobs()[static_cast<std::size_t>(sleeper)].state == S::Cancelled);

    // A command that crashes its process: the job fails with a message, and the daemon carries on.
    const int crash = q.submit("crash", {"dev", "crash"});
    q.wait();
    CHECK(q.jobs()[static_cast<std::size_t>(crash)].state == S::Failed);
    CHECK(q.jobs()[static_cast<std::size_t>(crash)].err.contains("crashed"));
    const int after = q.submit("after", {"--version"});
    q.wait();
    CHECK(q.jobs()[static_cast<std::size_t>(after)].state == S::Done);
    CHECK(daemon.state() == QProcess::Running);

    // Nobody listening: the job runs in a process as before.
    ohs::gui::JobQueue plain(ohs::gui::MainWindow::worker_program());
    plain.set_daemon(QDir(dir).filePath("nobody.sock"));
    const int fallback = plain.submit("version", {"--version"});
    plain.wait();
    CHECK(plain.jobs()[static_cast<std::size_t>(fallback)].state == S::Done);

    daemon.terminate();
    daemon.waitForFinished(5000);
}

TEST_CASE("gui: with a daemon listening the window runs its analyses there and shows the same results", "[gui][jobs]") {
    app();
    const QString dir = fresh_dir("window-daemon");
    const QString socket = QDir(dir).filePath("d.sock");
    QProcess daemon;
    daemon.start(ohs::gui::MainWindow::worker_program(), {"serve", "--socket", socket});
    REQUIRE(daemon.waitForStarted());
    QElapsedTimer wait;
    wait.start();
    while (!QFileInfo::exists(socket) && wait.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    REQUIRE(QFileInfo::exists(socket));
    qputenv("OHS_SOCKET", socket.toUtf8());
    {
        ohs::gui::MainWindow w;
        w.show();
        CHECK(w.job_queue()->daemon() == socket);
        REQUIRE(w.open_demo(QDir(dir).filePath("demo")));
        button(w, "Timekeeping")->click();
        w.wait_for_jobs();
        CHECK(w.results()->toPlainText().contains("rate", Qt::CaseInsensitive));
        CHECK(w.plot()->points().size() == 3);  // the plot data came from the same run
        CHECK(w.job_queue()->jobs()[0].state == ohs::gui::JobQueue::State::Done);
    }
    qunsetenv("OHS_SOCKET");
    daemon.terminate();
    daemon.waitForFinished(5000);
}

TEST_CASE("job queue: unfinished jobs survive a restart and finished ones keep their results", "[gui][jobs]") {
    app();
    const QString dir = fresh_dir("job-journal");
    const QString journal = QDir(dir).filePath("jobs.json");
    const QString marker = QDir(dir).filePath("started-once");
    // The first run leaves a marker and then hangs; a second run finds the marker and finishes.
    const QString script = QString("if [ -e '%1' ]; then echo second run; else touch '%1'; sleep 60; fi").arg(marker);
    {
        ohs::gui::JobQueue first("/bin/sh");
        REQUIRE(first.load(journal) == 0);
        first.submit("quick", {"-c", "echo before"});
        const int hung = first.submit("hangs once", {"-c", script});
        first.submit("waits", {"-c", "echo queued behind"});
        QElapsedTimer t;
        t.start();
        while (!QFile::exists(marker) && t.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        REQUIRE(QFile::exists(marker));
        REQUIRE(first.jobs()[0].state == ohs::gui::JobQueue::State::Done);
        REQUIRE(first.jobs()[hung].state == ohs::gui::JobQueue::State::Running);
    }  // the application ends here: the worker is killed with a job running and one queued

    ohs::gui::JobQueue second("/bin/sh");
    CHECK(second.load(journal) == 2);
    REQUIRE(second.jobs().size() == 3);
    CHECK(second.jobs()[0].state == ohs::gui::JobQueue::State::Done);
    CHECK(second.jobs()[0].out == "before\n");  // a finished job keeps its result
    CHECK(second.jobs()[1].state == ohs::gui::JobQueue::State::Queued);  // was running
    CHECK(second.jobs()[1].attempts == 1);
    CHECK(second.jobs()[2].state == ohs::gui::JobQueue::State::Queued);
    second.resume();
    second.wait();
    CHECK(second.jobs()[1].state == ohs::gui::JobQueue::State::Done);
    CHECK(second.jobs()[1].attempts == 2);
    CHECK(second.jobs()[1].out == "second run\n");
    CHECK(second.jobs()[2].out == "queued behind\n");

    ohs::gui::JobQueue third("/bin/sh");
    CHECK(third.load(journal) == 0);  // nothing left to resume
    CHECK(third.jobs()[2].out == "queued behind\n");
}

TEST_CASE("field overlay: the boundary of a result mesh, coloured by the field and placed like the part", "[gui][field]") {
    const auto m = ohs::mesh::extrude(ohs::mesh::structured_rect(0, 0, 2, 1, 2, 1), 0, 1, 1, 2);
    ohs::fea::NodeField vm{"von_mises", std::vector<double>(m.nodes.size()), 1};
    ohs::fea::NodeField disp{"displacement", std::vector<double>(3 * m.nodes.size(), 0.0), 3};
    for (std::size_t i = 0; i < m.nodes.size(); ++i) {
        vm.values[i] = 10.0 + m.nodes[i].x;  // 10 at the clamped end, 12 at the far end
        disp.values[3 * i + 2] = -0.5 * m.nodes[i].x;
    }
    const QString path = QDir(fresh_dir("field")).filePath("r.vtu");
    ohs::fea::write_vtu(path.toStdString(), m, {vm, disp});

    CHECK(ohs::gui::plottable_fields(path.toStdString()) == std::vector<std::string>{"von_mises", "displacement"});
    const auto o = ohs::gui::field_overlay_from_vtu(path.toStdString(), "wheel", "von_mises", 1.0, 0.5);
    CHECK(o.part == "wheel");
    CHECK(o.lo == Catch::Approx(10.0));
    CHECK(o.hi == Catch::Approx(12.0));
    CHECK(o.triangles.size() == ohs::mesh::boundary_faces(m).size());
    CHECK(o.values.size() == o.triangles.size());
    double xmin = 1e9;
    for (const auto& t : o.triangles) xmin = std::min({xmin, t.a[0], t.b[0], t.c[0]});
    CHECK(xmin == Catch::Approx(-1.0));  // moved by the pivot
    const auto d = ohs::gui::field_overlay_from_vtu(path.toStdString(), "wheel", "displacement", 0, 0);
    CHECK(d.hi == Catch::Approx(1.0));  // the length of the displacement vector, 0.5 * x at x = 2
    CHECK_THROWS(ohs::gui::field_overlay_from_vtu(path.toStdString(), "wheel", "temperature", 0, 0));

    const auto lo = ohs::gui::field_colour(0.0), hi = ohs::gui::field_colour(1.0);
    CHECK(lo != hi);
    CHECK(ohs::gui::field_colour(-3.0) == lo);  // clamped
    CHECK(ohs::gui::field_colour(7.0) == hi);
}

TEST_CASE("gui: a shock analysis is drawn on the part as a colour field, and can be switched and cleared", "[gui][field]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("field-gui")));
    w.select_part("escape_wheel");
    const auto before = w.viewport()->render(QSize(600, 450));
    REQUIRE(button(w, "Shock and modes of the selected part"));
    button(w, "Shock and modes of the selected part")->click();
    w.wait_for_jobs();

    const auto* field = w.viewport()->scene()->field();
    REQUIRE(field);  // the job's VTU was found and plotted
    CHECK(field->part == "escape_wheel");
    CHECK(field->hi > field->lo);
    CHECK(field->label.find("von Mises") != std::string::npos);
    const auto after = w.viewport()->render(QSize(600, 450));
    CHECK(after != before);
    // The wheel is no longer one flat colour: the pixels carry a spread of scale colours.
    std::set<QRgb> colours;
    for (int y = 0; y < after.height(); y += 2)
        for (int x = 0; x < after.width(); x += 2) colours.insert(after.pixel(x, y));
    CHECK(colours.size() > 60);

    w.field_box()->setCurrentIndex(1);  // displacement
    CHECK(w.viewport()->scene()->field()->label.find("displacement") != std::string::npos);
    w.clear_field();
    CHECK(w.viewport()->scene()->field() == nullptr);
    CHECK(w.viewport()->render(QSize(600, 450)) == before);
}

TEST_CASE("gui: tooth force arrows and motion trails follow the torque chain and the kinematics", "[gui][overlays]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("overlays")));
    ohs::gui::Scene scene = ohs::gui::Scene::load(w.project().toStdString());
    const auto& m = scene.movement();

    // Forces: one arrow for each mesh on the path from the barrel to the escape wheel.
    scene.compute_forces(1.0);
    REQUIRE(scene.forces().size() >= 3);
    const auto cal = ohs::dynamics::default_calibre(m.beat_rate_vph);
    const auto chain = ohs::dynamics::build_chain(m, scene.train(), cal);
    const auto table = ohs::dynamics::torque_table(chain, cal, 1.0);
    for (const auto& f : scene.forces()) {
        INFO(f.driver << " -> " << f.driven);
        double torque = 0;
        for (const auto& row : table)
            if (row.mobile == f.driven) torque = row.torque_nmm;
        REQUIRE(torque > 0);
        CHECK(f.newtons > 0);
        // F times the pitch radius of the driven wheel is the torque the driven mobile receives.
        bool matched = false;
        for (const auto& mesh : m.meshes) {
            for (int driven_comp : {mesh.a, mesh.b}) {
                const int mob = m.mobile_of(driven_comp);
                if (mob < 0 || m.mobiles[static_cast<std::size_t>(mob)].name != f.driven) continue;
                const double r = 0.5 * mesh.module * m.components[static_cast<std::size_t>(driven_comp)].features.teeth;
                matched = matched || std::abs(f.newtons * r - torque) < 1e-9 * torque;
            }
        }
        CHECK(matched);
        // Tangential: perpendicular to the line of centres, a unit vector in the dial plane.
        const auto find = [&](const std::string& name) {
            for (const auto& mob : m.mobiles) if (mob.name == name) return mob.center;
            return ohs::model::Vec2{};
        };
        const auto c1 = find(f.driver), c2 = find(f.driven);
        const double len = std::hypot(c2.x - c1.x, c2.y - c1.y);
        CHECK(std::hypot(f.direction[0], f.direction[1]) == Catch::Approx(1.0));
        CHECK(std::abs(f.direction[0] * (c2.x - c1.x) + f.direction[1] * (c2.y - c1.y)) < 1e-9 * len);
    }
    // Wind scales every force: half wind, half the force.
    const double full = scene.forces().front().newtons;
    scene.compute_forces(0.5);
    CHECK(scene.forces().front().newtons < full);

    // Trails: a marker per turning mobile, ending where the mobile is now, covering at most a turn.
    scene.set_time(300.0);
    scene.set_show_trails(true, 600.0);
    const auto trails = scene.trails(48);
    REQUIRE(trails.size() >= 3);
    const auto angles = ohs::kinematics::mobile_angles(scene.train(), 300.0, {false, 0.2});
    double fastest_span = 0, slowest_span = 1e9;
    for (const auto& t : trails) {
        REQUIRE(t.points.size() == 48);
        std::size_t mobile = 0;
        for (std::size_t i = 0; i < m.mobiles.size(); ++i) if (m.mobiles[i].name == t.mobile) mobile = i;
        const auto& c = m.mobiles[mobile].center;
        const auto& last = t.points.back();
        CHECK(std::atan2(last[1] - c.y, last[0] - c.x) == Catch::Approx(std::remainder(angles[mobile], 2 * M_PI)).margin(1e-9));
        double span = 0;  // path length of the polyline in turns of the marker
        const double r = std::hypot(last[0] - c.x, last[1] - c.y);
        for (std::size_t k = 1; k < t.points.size(); ++k) {
            span += std::hypot(t.points[k][0] - t.points[k - 1][0], t.points[k][1] - t.points[k - 1][1]) / r;
        }
        CHECK(span <= 2 * M_PI + 0.2);  // never more than one revolution (chord length slightly under arc length)
        fastest_span = std::max(fastest_span, span);
        slowest_span = std::min(slowest_span, span);
    }
    CHECK(fastest_span > 5.5);   // fast wheels have turned a full circle in the window
    CHECK(slowest_span < 1.0);   // the barrel has hardly moved
    scene.set_show_trails(false);
    CHECK(scene.show_trails() == false);

    // Drawing: each overlay changes the picture.
    w.viewport()->set_view(0, 0);
    const auto plain = w.viewport()->render(QSize(600, 450));
    w.forces_check()->setChecked(true);
    const auto with_forces = w.viewport()->render(QSize(600, 450));
    CHECK(with_forces != plain);
    w.forces_check()->setChecked(false);
    w.trails_check()->setChecked(true);
    w.set_time(300.0);
    const auto with_trails = w.viewport()->render(QSize(600, 450));
    CHECK(with_trails != plain);
    w.trails_check()->setChecked(false);
    CHECK(w.viewport()->render(QSize(600, 450)) == w.viewport()->render(QSize(600, 450)));
}

TEST_CASE("gui: the animation is exported as a video whose frames show the train turning", "[gui][video]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("video")));
    const QString path = QDir(fresh_dir("video-out")).filePath("train.mp4");
    ohs::gui::VideoOptions options;
    options.video_seconds = 1.0;
    options.fps = 10;
    options.movement_seconds = 30.0;
    options.size = QSize(321, 241);  // odd sizes are rounded down to even ones
    const double time_before = w.viewport()->scene()->time();

    if (ohs::gui::find_ffmpeg().isEmpty()) {
        CHECK_FALSE(w.export_video(path, options).isEmpty());  // a clear message instead of a crash
        SKIP("ffmpeg is not available");
    }
    const QString error = w.export_video(path, options);
    REQUIRE(error.isEmpty());
    CHECK(QFileInfo(path).size() > 1000);
    CHECK(w.viewport()->scene()->time() == Catch::Approx(time_before));  // the view is left as it was

    // Read the file back: size, and a frame count of the 10 frames asked for.
    const QString ffprobe = QStandardPaths::findExecutable("ffprobe");
    if (!ffprobe.isEmpty()) {
        QProcess probe;
        probe.start(ffprobe, {"-v", "error", "-count_frames", "-select_streams", "v:0", "-show_entries",
                              "stream=width,height,nb_read_frames,codec_name", "-of", "csv=p=0", path});
        REQUIRE(probe.waitForFinished(30000));
        const QString out = QString::fromUtf8(probe.readAllStandardOutput()).trimmed();
        INFO(out.toStdString());
        CHECK(out.contains("h264"));
        CHECK(out.contains("320,240"));
        CHECK(out.endsWith(",10"));
    }

    // Unusable requests are reported, not attempted.
    ohs::gui::VideoOptions bad = options;
    bad.fps = 0;
    CHECK_FALSE(w.export_video(QDir(fresh_dir("video-out2")).filePath("x.mp4"), bad).isEmpty());
}

TEST_CASE("gui: jobs run in worker processes and resume when the project is opened again", "[gui][jobs]") {
    app();
    const QString dir = fresh_dir("window-jobs");
    const QString marker = QDir(dir).filePath("started-once");
    const QString script = QString("if [ -e '%1' ]; then echo second run; else touch '%1'; sleep 60; fi").arg(marker);
    qputenv("OHS_EXE", "/bin/sh");  // stand in for the ohs worker, so the job's behaviour is ours
    QString project;
    {
        ohs::gui::MainWindow w;
        w.show();
        REQUIRE(w.open_demo(dir));
        project = w.project();
        w.submit("hangs once", {"-c", script});
        QElapsedTimer t;
        t.start();
        while (!QFile::exists(marker) && t.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        REQUIRE(QFile::exists(marker));
        CHECK(w.job_list()->item(0)->text().startsWith("running"));
        CHECK(QFile::exists(project + ".jobs.json"));
    }  // the window closes with the job running

    ohs::gui::MainWindow w2;
    w2.show();
    REQUIRE(w2.open_project(project));
    REQUIRE(w2.job_list()->count() == 1);
    w2.wait_for_jobs();
    CHECK(w2.job_list()->item(0)->text().startsWith("done"));
    CHECK(w2.job_queue()->jobs()[0].attempts == 2);
    CHECK(w2.log_console()->toPlainText().contains("Resuming 1 job"));
    w2.job_list()->setCurrentRow(-1);
    w2.job_list()->setCurrentRow(0);  // selecting a finished job shows its output
    CHECK(w2.results()->toPlainText().contains("second run"));

    // A job of the running window can be cancelled while it runs.
    w2.submit("slow", {"-c", "sleep 60"});
    REQUIRE(w2.job_list()->item(1)->text().startsWith("running"));
    CHECK(w2.cancel_job(1));
    w2.wait_for_jobs();
    CHECK(w2.job_list()->item(1)->text().startsWith("cancelled"));
    qunsetenv("OHS_EXE");
}

TEST_CASE("gui: every project edit is undoable and undo restores the file exactly", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("undo-all")));
    const auto read = [&] {
        QFile f(w.project());
        REQUIRE(f.open(QIODevice::ReadOnly));
        return f.readAll();
    };
    const auto movement = [&]() -> const ohs::model::Movement& { return w.viewport()->scene()->movement(); };
    const QByteArray original = read();
    REQUIRE(w.undo_stack()->count() == 0);  // opening a project starts a fresh history

    // The escape wheel and the barrel are not meshed by the geometry.
    const QString a = "escape_wheel", b = "barrel";
    REQUIRE(movement().find_component(a.toStdString()) >= 0);
    REQUIRE(movement().find_component(b.toStdString()) >= 0);
    const auto manual_meshes = [&] {
        int n = 0;
        for (const auto& j : movement().meshes) n += j.manual ? 1 : 0;
        return n;
    };
    REQUIRE(manual_meshes() == 0);

    REQUIRE(w.declare_mesh(a, b));
    CHECK(manual_meshes() == 1);
    CHECK(w.undo_stack()->count() == 1);
    CHECK(w.undo_stack()->undoText().contains("Mesh escape_wheel with barrel"));
    const QByteArray with_mesh = read();
    CHECK(with_mesh != original);

    // A role change on top, then both undone one at a time.
    REQUIRE(w.change_role("center_wheel", "plate"));
    CHECK(w.undo_stack()->count() == 2);
    w.undo_stack()->undo();
    CHECK(read() == with_mesh);  // byte for byte, including role_source and confidence
    CHECK(manual_meshes() == 1);
    w.undo_stack()->undo();
    CHECK(read() == original);
    CHECK(manual_meshes() == 0);
    CHECK_FALSE(w.undo_stack()->canUndo());

    // Redo replays the edits; jumping about the way the history view does lands on the same states.
    w.undo_stack()->redo();
    CHECK(manual_meshes() == 1);
    w.undo_stack()->setIndex(2);
    CHECK(movement().components[static_cast<std::size_t>(movement().find_component("center_wheel"))].role == ohs::model::Role::Plate);
    w.undo_stack()->setIndex(0);
    CHECK(read() == original);

    // A failing command changes nothing and adds nothing to the history.
    CHECK_FALSE(w.declare_mesh(a, "no such part"));
    CHECK(read() == original);
    CHECK(w.undo_stack()->index() == 0);

    // The partner list holds the toothed parts, and the history belongs to one project.
    CHECK(w.mesh_partner_box()->findText("barrel") >= 0);
    w.undo_stack()->redo();
    REQUIRE(w.open_project(w.project()));
    CHECK(w.undo_stack()->count() == 0);
}

TEST_CASE("gui: drag a part to turn the movement, drag beside it to pan, the wheel zooms toward the selection", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("viewport-input")));
    auto* v = w.viewport();
    v->resize(640, 480);
    v->set_view(35, -55);

    const auto send = [&](QEvent::Type type, QPoint at, Qt::MouseButton button, Qt::MouseButtons held) {
        QMouseEvent e(type, QPointF(at), QPointF(v->mapToGlobal(at)), button, held, Qt::NoModifier);
        QCoreApplication::sendEvent(v, &e);
    };
    const auto drag = [&](QPoint from, QPoint to) {
        send(QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
        for (int i = 1; i <= 4; ++i) send(QEvent::MouseMove, from + (to - from) * i / 4, Qt::NoButton, Qt::LeftButton);
        send(QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
    };
    const auto wheel = [&](int delta) {
        QWheelEvent e(QPointF(320, 240), QPointF(v->mapToGlobal(QPoint(320, 240))), QPoint(), QPoint(0, delta), Qt::NoButton,
                      Qt::NoModifier, Qt::NoScrollPhase, false);
        QCoreApplication::sendEvent(v, &e);
    };
    const auto on_part = [&] {
        for (int y = 120; y < 360; y += 6)
            for (int x = 160; x < 480; x += 6)
                if (!v->pick(QPoint(x, y)).empty()) return QPoint(x, y);
        return QPoint(-1, -1);
    };
    const QPoint beside(8, 8);
    REQUIRE(v->pick(beside).empty());

    // Dragging from a part turns the movement about its axis and leaves the pan and the selection alone.
    QPoint hit = on_part();
    REQUIRE(hit.x() >= 0);
    const QPointF pan0 = v->pan();
    const double yaw0 = v->yaw();
    drag(hit, hit + QPoint(60, 0));
    CHECK(v->yaw() == Catch::Approx(yaw0 - 30));
    CHECK(v->pan() == pan0);
    CHECK(v->selected().empty());  // a drag is not a click

    // Dragging from beside the movement pans and does not turn it.
    drag(beside, beside + QPoint(40, 24));
    CHECK(v->pan() == pan0 + QPointF(40, 24));
    CHECK(v->yaw() == Catch::Approx(yaw0 - 30));

    // A click on a part selects it; the wheel then zooms toward it: the part stays where it is on screen.
    hit = on_part();
    REQUIRE(hit.x() >= 0);
    send(QEvent::MouseButtonPress, hit, Qt::LeftButton, Qt::LeftButton);
    send(QEvent::MouseButtonRelease, hit, Qt::LeftButton, Qt::NoButton);
    REQUIRE_FALSE(v->selected().empty());
    CHECK(w.explain_panel()->toPlainText().contains(QString::fromStdString(v->selected())));
    const auto at = v->screen_position(v->selected());
    REQUIRE(at);
    const double zoom0 = v->zoom();
    wheel(240);
    CHECK(v->zoom() > zoom0);
    const auto after = v->screen_position(v->selected());
    REQUIRE(after);
    CHECK(std::hypot(after->x() - at->x(), after->y() - at->y()) < 1.0);

    // A click beside the movement clears the selection, and the wheel zooms about the middle again.
    send(QEvent::MouseButtonPress, beside, Qt::LeftButton, Qt::LeftButton);
    send(QEvent::MouseButtonRelease, beside, Qt::LeftButton, Qt::NoButton);
    CHECK(v->selected().empty());
    const QPointF pan1 = v->pan();
    wheel(-120);
    CHECK(v->pan() == pan1);
}

TEST_CASE("gui: the editor follows the selection, playback controls agree, the analyses get the project", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("ui-state")));
    CHECK(w.windowTitle().startsWith("demo.hsproj"));

    // The rig editor is off until a part is selected, then starts at that part's role.
    auto* set_role = button(w, "Set role");
    REQUIRE(set_role);
    CHECK_FALSE(set_role->isEnabled());
    w.select_part("third_wheel");
    CHECK(set_role->isEnabled());
    QComboBox* role_box = nullptr;
    for (auto* c : w.workspaces()->findChildren<QComboBox*>())
        if (c->findText("escape_wheel") >= 0 && c->findText("hairspring") >= 0) role_box = c;
    REQUIRE(role_box);
    CHECK(role_box->currentText() == "wheel");
    w.select_part("");
    CHECK_FALSE(set_role->isEnabled());

    // The Animate page's Play button and the toolbar's are one control.
    QAction* play = nullptr;
    for (auto* a : w.findChildren<QAction*>())
        if (a->isCheckable() && a->text() == "Play") play = a;
    REQUIRE(play);
    play->setChecked(true);
    auto* play_button = button(w, "Pause");
    REQUIRE(play_button);
    CHECK(play_button->isChecked());
    w.set_playing(false);
    CHECK_FALSE(play->isChecked());
    CHECK(play->text() == "Play");

    // The readouts follow the controls.
    w.timeline()->setValue(5000);
    bool time_shown = false;
    for (auto* l : w.workspaces()->findChildren<QLabel*>()) time_shown |= l->text() == "30.0 s of 60 s";
    CHECK(time_shown);

    // Every analysis that needs the project is given it (the model cross-check used to be run without one).
    for (auto* b : w.workspaces()->findChildren<QPushButton*>()) {
        const QStringList args = b->property("ohs_args").toStringList();
        if (args.isEmpty() || args.first() == "materials") continue;
        INFO(b->text().toStdString());
        CHECK(args.contains("@project"));
    }
}

TEST_CASE("gui: the right-click menu of the model tree and rig list shows and hides parts by role", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("role-visibility")));
    auto* scene = w.viewport()->scene();
    for (QWidget* view : {static_cast<QWidget*>(w.model_tree()), static_cast<QWidget*>(w.rig_table())})
        CHECK(view->contextMenuPolicy() == Qt::CustomContextMenu);

    // One entry per role in the movement, ticked while every part of the role is shown.
    std::unique_ptr<QMenu> menu(w.visibility_menu());
    QAction* entry = nullptr;
    std::set<QString> roles;
    for (QAction* a : menu->actions()) {
        if (!a->isCheckable()) continue;  // the section heading
        roles.insert(a->data().toString());
        CHECK(a->isChecked());
        if (a->data().toString() == "plate") entry = a;
    }
    std::set<QString> expected;
    for (const auto& p : scene->parts()) expected.insert(QString::fromUtf8(ohs::model::to_string(p.role).data()));
    CHECK(roles == expected);
    REQUIRE(entry);

    auto role_of = [](const ohs::gui::ScenePart& p) { return QString::fromUtf8(ohs::model::to_string(p.role).data()); };
    auto tree_ticked = [&](const std::string& name) {
        for (QTreeWidgetItemIterator it(w.model_tree()); *it; ++it)
            if ((*it)->parent() && (*it)->text(0).toStdString() == name) return (*it)->checkState(0) == Qt::Checked;
        return false;
    };

    // Unticking hides every plate, and only the plates; the tree follows.
    entry->setChecked(false);
    for (const auto& p : scene->parts()) {
        CHECK(scene->visible(p.name) == (role_of(p) != "plate"));
        CHECK(tree_ticked(p.name) == scene->visible(p.name));
    }
    menu.reset(w.visibility_menu());
    for (QAction* a : menu->actions())
        if (a->isCheckable()) CHECK(a->isChecked() == (a->data().toString() != "plate"));

    // Ticking it again shows the plates.
    entry = nullptr;
    for (QAction* a : menu->actions())
        if (a->data().toString() == "plate") entry = a;
    REQUIRE(entry);
    entry->setChecked(true);
    for (const auto& p : scene->parts()) CHECK((scene->visible(p.name) && tree_ticked(p.name)));

    // A role with one part unticked in the tree is shown unticked in the menu and says how many are shown;
    // ticking it shows all its parts again.
    QString many;
    std::size_t most = 0;
    for (const QString& r : expected) {
        const auto n = static_cast<std::size_t>(std::count_if(scene->parts().begin(), scene->parts().end(),
                                                              [&](const auto& p) { return role_of(p) == r; }));
        if (n > most) most = n, many = r;
    }
    REQUIRE(most > 1);
    std::string one;
    for (const auto& p : scene->parts())
        if (role_of(p) == many) one = p.name;
    for (QTreeWidgetItemIterator it(w.model_tree()); *it; ++it)
        if ((*it)->parent() && (*it)->text(0).toStdString() == one) (*it)->setCheckState(0, Qt::Unchecked);
    REQUIRE_FALSE(scene->visible(one));
    menu.reset(w.visibility_menu());
    entry = nullptr;
    for (QAction* a : menu->actions())
        if (a->data().toString() == many) entry = a;
    REQUIRE(entry);
    CHECK_FALSE(entry->isChecked());
    CHECK(entry->text().contains(QString("%1 of %2 shown").arg(most - 1).arg(most)));
    entry->setChecked(true);
    for (const auto& p : scene->parts()) {
        INFO(p.name << " (" << role_of(p).toStdString() << ")");
        CHECK(scene->visible(p.name));
        CHECK(tree_ticked(p.name));
    }
    CHECK_FALSE(w.set_role_visible("no such role", false));

    // Show only hides every role but the one picked; Show all unhides everything.
    menu.reset(w.visibility_menu());
    QAction* show_all = nullptr;
    QMenu* show_only = nullptr;
    for (QAction* a : menu->actions()) {
        if (a->text() == "Show all") show_all = a;
        if (a->menu() && a->text() == "Show only") show_only = a->menu();
    }
    REQUIRE(show_all);
    REQUIRE(show_only);
    CHECK(show_only->actions().size() == static_cast<int>(expected.size()));
    for (QAction* a : show_only->actions()) {
        if (a->text() != "plate") continue;
        a->trigger();
        break;
    }
    for (const auto& p : scene->parts()) CHECK((scene->visible(p.name) == (role_of(p) == "plate") && tree_ticked(p.name) == scene->visible(p.name)));
    show_all->trigger();
    for (const auto& p : scene->parts()) CHECK((scene->visible(p.name) && tree_ticked(p.name)));
}

TEST_CASE("gui: the menus open projects on a worker thread and the window keeps running", "[gui]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    const QString dir = fresh_dir("async-open");
    auto wait_loaded = [&] {
        QElapsedTimer t;
        t.start();
        while (w.is_loading() && t.elapsed() < 300000) QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    };

    w.open_demo_async(dir);
    CHECK(w.is_loading());  // returned at once: the import runs in the background
    CHECK(w.project().isEmpty());
    CHECK(w.run_command({"materials"}) == 2);  // refused rather than queued behind the import
    wait_loaded();
    REQUIRE_FALSE(w.is_loading());
    CHECK(w.project() == QDir(dir).filePath("demo.hsproj"));
    REQUIRE(w.viewport()->scene());
    CHECK(w.rig_table()->rowCount() > 0);
    const QStringList h = w.history();
    REQUIRE(h.size() == 2);
    CHECK(h[0].startsWith("ohs dev gen-movement"));
    CHECK(h[1].startsWith("ohs import"));

    // Opening again replaces the scene with a fresh one and a fresh edit history.
    REQUIRE(w.change_role("center_wheel", "plate"));
    CHECK(w.undo_stack()->count() == 1);
    const auto* before = w.viewport()->scene();
    w.open_project_async(w.project());
    wait_loaded();
    CHECK(w.viewport()->scene() != before);
    CHECK(w.undo_stack()->count() == 0);

    // A file that cannot be read is reported, and the window carries on with what it had.
    w.open_project_async(QDir(dir).filePath("none.hsproj"));
    wait_loaded();
    CHECK(w.log_console()->toPlainText().contains("Cannot open"));
    CHECK(w.project() == QDir(dir).filePath("demo.hsproj"));
}

TEST_CASE("gui: the stem turns, a click rides up a tooth of the barrel and a spring bends with it", "[gui][keyless]") {
    app();
    const QString dir = fresh_dir("keyless");
    const QString step = QDir(dir).filePath("keyless.step"), project = QDir(dir).filePath("keyless.hsproj");
    ohs::geometry::export_step(ohs::test::train_with_keyless_work(), step.toStdString());
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"}).exit_code == 0);
    ohs::gui::Scene scene = ohs::gui::Scene::load(project.toStdString());
    REQUIRE(scene.can_animate());
    REQUIRE(scene.train().articulations.size() == 3);

    // Where the vertices of one part are drawn at a time, one entry per triangle corner.
    const auto drawn = [&scene](const std::string& name, double seconds) {
        scene.set_time(seconds);
        const auto list = scene.draw_list();
        std::vector<std::array<double, 3>> out;
        for (std::size_t i = 0; i < list.triangles.size(); ++i) {
            if (scene.parts()[static_cast<std::size_t>(list.part_index[i])].name != name) continue;
            for (const auto* p : {&list.triangles[i].a, &list.triangles[i].b, &list.triangles[i].c}) out.push_back({(*p)[0], (*p)[1], (*p)[2]});
        }
        return out;
    };
    const auto largest_move = [&](const std::string& name, double seconds) {
        const auto before = drawn(name, 0.0), after = drawn(name, seconds);
        REQUIRE(before.size() == after.size());
        double most = 0;
        for (std::size_t i = 0; i < before.size(); ++i) {
            most = std::max(most, std::hypot(after[i][0] - before[i][0], std::hypot(after[i][1] - before[i][1], after[i][2] - before[i][2])));
        }
        return most;
    };
    const double tooth = scene.train().root_period_hours * 3600.0 / 80;  // seconds for one tooth of the barrel to pass

    // The stem: an hour on, it has turned about its own axis (the rod's centre does not move, its surface does).
    CHECK(largest_move("part_a", 3600.0) > 0.5);
    const auto stem0 = drawn("part_a", 0.0), stem1 = drawn("part_a", 3600.0);
    double c0 = 0, c1 = 0;
    for (std::size_t i = 0; i < stem0.size(); ++i) {
        c0 += stem0[i][1] / stem0.size();
        c1 += stem1[i][1] / stem1.size();
    }
    CHECK(c1 == Catch::Approx(c0).margin(0.1));  // the mean of the corners drifts a little as they go round
    // The click: at 70 % of a tooth it is up, its tip moved by about the tooth depth, and at the end of the tooth
    // it is back; its pivot hardly moves.
    const double up = largest_move("part_b", 0.7 * tooth);
    CHECK(up > 0.15);
    CHECK(up < 0.5);
    CHECK(largest_move("part_b", tooth) < 1e-6);
    // The spring bends the same way, about its fixed end, which stays put.
    const double bend = largest_move("part_c", 0.7 * tooth);
    CHECK(bend > 0.05);
    const auto spring0 = drawn("part_c", 0.0), spring1 = drawn("part_c", 0.7 * tooth);
    double fixed_end_move = 1e9;
    for (std::size_t i = 0; i < spring0.size(); ++i) {
        if (spring0[i][0] > 6.99) fixed_end_move = std::min(fixed_end_move, std::hypot(spring1[i][0] - spring0[i][0], spring1[i][1] - spring0[i][1]));
    }
    CHECK(fixed_end_move < 0.02);  // its corners, 0.4 mm off the pivot, move by that much times the small angle
}

// Hidden (run with `ohs_gui_tests "[bench]"`): where the time goes on the demonstration movement.
TEST_CASE("bench: scene load, frame and pick times of the demonstration movement", "[.][bench]") {
    app();
    // OHS_BENCH_PROJECT names a larger project to measure (its CAD file beside it); the default is the demonstration.
    QString project = qEnvironmentVariable("OHS_BENCH_PROJECT"), step;
    if (project.isEmpty()) {
        const QString dir = fresh_dir("bench");
        step = QDir(dir).filePath("demo.step");
        project = QDir(dir).filePath("demo.hsproj");
        REQUIRE(ohs::cli::run_captured({"dev", "gen-movement", "-o", step.toStdString()}).exit_code == 0);
        REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"})
                    .exit_code == 0);
    }
    QElapsedTimer t;
    t.start();
    auto scene = std::make_shared<ohs::gui::Scene>(ohs::gui::Scene::load(project.toStdString(), step.toStdString()));
    const qint64 load_ms = t.elapsed();  // from the cache when the project has one: a second run shows the difference
    std::size_t triangles = 0;
    std::size_t coarse = 0;
    for (const auto& p : scene->parts()) {
        triangles += p.triangles.size();
        coarse += p.coarse.size();
    }
    WARN(coarse << " triangles in the coarse mesh drawn while interacting");
    ohs::gui::Viewport v;
    v.resize(1024, 768);
    v.set_scene(scene);
    t.restart();
    for (int i = 0; i < 20; ++i) (void)scene->centre();
    const double centre_ms = t.elapsed() / 20.0;
    auto frame = [&](const char* what) {
        t.restart();
        for (int i = 0; i < 5; ++i) {
            scene->set_time(i * 0.1);
            (void)v.render(QSize(1024, 768));
        }
        WARN(what << ": " << t.elapsed() / 5.0 << " ms per frame");
    };
    frame("plain");
    {
        QElapsedTimer f;
        f.start();
        for (int i = 0; i < 5; ++i) {
            scene->set_time(i * 0.1);
            (void)v.render(QSize(1024, 768), true);
        }
        WARN("fast (interactive): " << f.elapsed() / 5.0 << " ms per frame");
    }
    scene->set_show_trails(true, 10.0);
    frame("with trails");
    scene->set_show_trails(false);
    (void)v.pick(QPoint(512, 384));  // builds the drawn list
    QElapsedTimer q;
    q.start();
    std::string hit;
    for (int i = 0; i < 20; ++i) hit = v.pick(QPoint(400 + 10 * i, 384));
    const double pick_ms = q.nsecsElapsed() / 20e6;
    v.select(hit.empty() ? scene->parts().front().name : hit);
    q.restart();
    for (int i = 0; i < 20; ++i) (void)v.screen_position(v.selected());
    const double wheel_ms = q.nsecsElapsed() / 20e6;
    q.restart();
    for (int i = 0; i < 5; ++i) {
        scene->set_time(i * 0.1);
        (void)scene->draw_list();
    }
    const double draw_list_ms = q.nsecsElapsed() / 5e6;
    WARN("load " << load_ms << " ms, " << triangles << " triangles, centre() " << centre_ms << " ms, pick " << pick_ms
                 << " ms, screen_position " << wheel_ms << " ms, draw_list " << draw_list_ms << " ms");
}

TEST_CASE("gui: the rig page shows the train, what is left to rig and suggested meshes, and re-runs the auto-rig", "[gui][rig]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("rig-review")));
    const auto movement = [&]() -> const ohs::model::Movement& { return w.viewport()->scene()->movement(); };
    const auto role_of = [&](const char* part) {
        return movement().components[static_cast<std::size_t>(movement().find_component(part))].role;
    };
    const auto green_pixels = [](const QImage& image) {
        std::size_t n = 0;
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                if (image.pixel(x, y) == qRgb(60, 200, 110)) ++n;
        return n;
    };

    // A rigged train is connected and has nothing to suggest.
    CHECK(w.chain_label()->text().contains("Connected"));
    CHECK_FALSE(w.chain_label()->text().contains("Not connected"));
    CHECK(w.suggestion_list()->count() == 0);

    // The rig is drawn over the movement on request: the meshes the barrel feeds are green.
    const QImage plain = w.viewport()->render(QSize(640, 480));
    CHECK(green_pixels(plain) == 0);
    w.rig_overlay_check()->setChecked(true);
    CHECK(w.viewport()->show_rig());
    const QImage rigged = w.viewport()->render(QSize(640, 480));
    CHECK(green_pixels(rigged) > 40);
    w.rig_overlay_check()->setChecked(false);
    CHECK_FALSE(w.viewport()->show_rig());
    CHECK(green_pixels(w.viewport()->render(QSize(640, 480))) == 0);

    // Taking a wheel out of the train breaks it, says where, and lists it as a thing to do.
    REQUIRE(w.change_role("center_wheel", "plate"));
    CHECK(w.chain_label()->text().contains("Not connected"));
    CHECK(w.rig_tabs()->tabText(1).startsWith("To do ("));
    int broken = -1;
    for (int row = 0; row < w.issue_list()->count(); ++row)
        if (w.issue_list()->item(row)->text().contains("the train is broken")) broken = row;
    REQUIRE(broken >= 0);
    w.issue_list()->setCurrentRow(broken);
    CHECK_FALSE(w.viewport()->selected().empty());  // a part at one end of the break is selected
    CHECK_FALSE(w.viewport()->scene()->can_animate());
    w.rig_overlay_check()->setChecked(true);
    CHECK(green_pixels(w.viewport()->render(QSize(640, 480))) > 0);  // still fed as far as the break
    w.rig_overlay_check()->setChecked(false);

    // Re-running the auto-rig keeps the role that was set; starting from scratch forgets it. Both can be undone.
    const int edits = w.undo_stack()->count();
    REQUIRE(w.rerig());
    CHECK(role_of("center_wheel") == ohs::model::Role::Plate);
    CHECK(w.undo_stack()->count() == edits);  // nothing changed, so nothing to undo
    REQUIRE(w.rerig(true));
    CHECK(role_of("center_wheel") == ohs::model::Role::Wheel);
    CHECK(w.chain_label()->text().contains("Connected"));
    CHECK(w.undo_stack()->count() == edits + 1);
    CHECK(w.undo_stack()->undoText().contains("scratch"));
    w.undo_stack()->undo();
    CHECK(role_of("center_wheel") == ohs::model::Role::Plate);
    CHECK(w.chain_label()->text().contains("Not connected"));
    w.undo_stack()->undo();
    CHECK(role_of("center_wheel") == ohs::model::Role::Wheel);

    // Nudge the centre mobile off its mesh with the barrel: the strict test fails, the pair is suggested.
    CHECK_FALSE(w.accept_suggested_meshes());  // nothing to declare yet
    auto m = ohs::model::load_rig(w.project().toStdString());
    for (auto& c : m.components) {
        const std::string& n = c.features.name;
        if (n == "center_pinion" || n == "center_wheel" || n == "center_arbor") c.features.center.x += 0.05;
    }
    ohs::rigging::infer_structure(m, {});
    ohs::model::save_rig(m, w.project().toStdString());
    REQUIRE(w.open_project(w.project()));
    CHECK(w.chain_label()->text().contains("Not connected"));
    REQUIRE(w.suggestion_list()->count() >= 1);
    CHECK(w.rig_tabs()->tabText(2).startsWith("Suggested meshes ("));
    bool barrel_pair = false;
    for (int row = 0; row < w.suggestion_list()->count(); ++row) {
        const QString text = w.suggestion_list()->item(row)->text();
        barrel_pair |= text.contains("barrel") && text.contains("center_pinion");
    }
    CHECK(barrel_pair);
    w.suggestion_list()->setCurrentRow(0);  // selects the pair: the first part in the viewport, the second as the partner
    CHECK_FALSE(w.viewport()->selected().empty());
    CHECK_FALSE(w.mesh_partner_box()->currentText().isEmpty());

    REQUIRE(w.accept_suggested_meshes());
    CHECK(w.chain_label()->text().contains("Connected"));
    CHECK(w.suggestion_list()->count() == 0);
    CHECK(w.undo_stack()->undoText().contains("suggested mesh"));
    w.undo_stack()->undo();
    CHECK(w.chain_label()->text().contains("Not connected"));
    CHECK(w.suggestion_list()->count() >= 1);
}

TEST_CASE("gui: the Rig tab walks the train one mesh at a time and helps fill the gap", "[gui][rig]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_demo(fresh_dir("rig-walk")));
    const auto m = [&]() -> const ohs::model::Movement& { return w.viewport()->scene()->movement(); };
    const auto name_of = [&](int component) { return m().components[static_cast<std::size_t>(component)].features.name; };
    const auto role_of = [&](const char* part) {
        return m().components[static_cast<std::size_t>(m().find_component(part))].role;
    };

    // The demonstration train is four meshes, from the barrel to the escape pinion, and the walk starts at the first.
    REQUIRE(w.workspaces()->currentIndex() == 0);
    CHECK(w.current_step() == 0);
    REQUIRE(w.viewport()->walk().has_value());
    CHECK(name_of(w.viewport()->walk()->driver) == "barrel");
    CHECK(name_of(w.viewport()->walk()->driven) == "center_pinion");
    CHECK_FALSE(w.viewport()->walk()->gap);
    CHECK(w.step_label()->text().contains("barrel"));
    CHECK_FALSE(w.previous_step_button()->isEnabled());
    CHECK(w.next_step_button()->isEnabled());
    CHECK(w.chip_strip()->findChildren<QToolButton*>().size() == 4);

    // Next goes along the train; each step starts from the mobile the one before ended on.
    w.next_step_button()->click();
    CHECK(w.current_step() == 1);
    CHECK(name_of(w.viewport()->walk()->driver) == "center_wheel");
    CHECK(w.previous_step_button()->isEnabled());
    w.go_to_step(99);  // clamped to the last step
    CHECK(w.current_step() == 3);
    CHECK(w.step_label()->text().contains("train is complete"));
    CHECK_FALSE(w.next_step_button()->isEnabled());
    // Everything of the mobiles walked so far is drawn as it is; the rest is faded, so the picture differs.
    const QImage last = w.viewport()->render(QSize(640, 480));
    w.go_to_step(0);
    CHECK(w.viewport()->render(QSize(640, 480)) != last);

    // The walk is for the Rig tab: elsewhere the viewport shows the movement as it is.
    w.workspaces()->setCurrentIndex(1);
    CHECK_FALSE(w.viewport()->walk().has_value());
    w.workspaces()->setCurrentIndex(0);
    CHECK(w.viewport()->walk().has_value());
    w.rig_overlay_check()->setChecked(true);  // the whole rig instead
    CHECK_FALSE(w.viewport()->walk().has_value());
    w.rig_overlay_check()->setChecked(false);
    CHECK(w.viewport()->walk().has_value());

    // No wheel after the centre pinion: the walk ends in a gap there, and says so.
    REQUIRE(w.change_role("center_wheel", "plate"));
    CHECK(w.chain_label()->text().contains("Not connected"));
    w.go_to_step(99);
    REQUIRE(w.viewport()->walk().has_value());
    CHECK(w.viewport()->walk()->gap);
    CHECK(name_of(w.viewport()->walk()->driver) == "center_pinion");
    CHECK(w.step_label()->text().contains("Nothing meshes with center_pinion"));
    CHECK_FALSE(w.next_step_button()->isEnabled());
    // Pick the part it drives: the next click on a part in the viewport sets its role (here, back to a wheel).
    w.pick_part_for_role("wheel");
    CHECK(w.is_picking());
    emit w.viewport()->part_picked(QString("center_wheel"));
    CHECK_FALSE(w.is_picking());
    CHECK(role_of("center_wheel") == ohs::model::Role::Wheel);
    CHECK(w.chain_label()->text().contains("Connected"));
    CHECK(w.undo_stack()->undoText().contains("center_wheel"));

    // Nudge the centre mobile off its mesh with the barrel. The gap is at the barrel, the likely pair is listed and
    // shown, and picking the partner in the viewport declares the mesh and the walk carries on from there.
    auto rig = ohs::model::load_rig(w.project().toStdString());
    for (auto& c : rig.components) {
        const std::string& n = c.features.name;
        if (n == "center_pinion" || n == "center_wheel" || n == "center_arbor") c.features.center.x += 0.05;
    }
    ohs::rigging::infer_structure(rig, {});
    ohs::model::save_rig(rig, w.project().toStdString());
    REQUIRE(w.open_project(w.project()));
    CHECK(w.current_step() == 0);
    REQUIRE(w.viewport()->walk().has_value());
    CHECK(w.viewport()->walk()->gap);
    CHECK(name_of(w.viewport()->walk()->driver) == "barrel");
    REQUIRE(w.candidate_list()->count() >= 1);
    CHECK(w.candidate_list()->item(0)->text().contains("center_pinion"));
    CHECK(name_of(w.viewport()->walk()->preview_b) == "center_pinion");  // previewed in the viewport
    w.pick_partner_for("barrel");
    emit w.viewport()->part_picked(QString("center_pinion"));
    CHECK(w.chain_label()->text().contains("Connected"));
    CHECK(w.current_step() == 0);
    CHECK_FALSE(w.viewport()->walk()->gap);
    CHECK(name_of(w.viewport()->walk()->driven) == "center_pinion");
    w.undo_stack()->undo();
    CHECK(w.viewport()->walk()->gap);

    // A click beside the movement cancels a pick.
    w.pick_partner_for("barrel");
    emit w.viewport()->part_picked(QString());
    CHECK_FALSE(w.is_picking());
}

// Hidden (run with `ohs_gui_tests "[screenshots]"`): the whole window on each workspace, written to
// $OHS_SCREENSHOT_DIR (default: the temporary directory), for reviewing the layout.
TEST_CASE("screenshots: the window on each workspace", "[.][screenshots]") {
    ohs::gui::apply_theme(app());
    ohs::gui::MainWindow w;
    w.resize(1440, 900);
    w.show();
    QString out = qEnvironmentVariable("OHS_SCREENSHOT_DIR");
    if (out.isEmpty()) out = fresh_dir("screenshots");
    QDir().mkpath(out);
    const auto pump = [] {
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < 400) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    };
    pump();
    w.grab().save(QDir(out).filePath("00-empty.png"));
    REQUIRE(w.open_demo(fresh_dir("screenshots-demo")));
    w.select_part("escape_wheel");
    for (int i = 0; i < w.workspaces()->count(); ++i) {
        w.workspaces()->setCurrentIndex(i);
        pump();
        w.grab().save(QDir(out).filePath(QString("%1-%2.png").arg(i + 1, 2, 10, QChar('0')).arg(w.workspaces()->tabText(i).remove('&').replace(' ', '-').toLower())));
    }
    // The rig page walking the train: a step in the middle, then a train that is broken at the centre wheel, then
    // the whole rig drawn at once.
    w.workspaces()->setCurrentIndex(0);
    w.select_part("");
    w.go_to_step(1);
    pump();
    w.grab().save(QDir(out).filePath("90-walk-step.png"));
    REQUIRE(w.change_role("center_wheel", "plate"));
    w.go_to_step(99);
    pump();
    w.grab().save(QDir(out).filePath("91-walk-gap.png"));
    w.rig_overlay_check()->setChecked(true);
    w.rig_tabs()->setCurrentIndex(1);
    pump();
    w.grab().save(QDir(out).filePath("92-rig-overlay.png"));
}

TEST_CASE("gui: a rig edit does not wait for queued jobs, and a job reads the project as it was when queued", "[gui][jobs]") {
    app();
    ohs::gui::MainWindow w;
    w.show();
    const QString dir = fresh_dir("edit-while-queued");
    REQUIRE(w.open_demo(dir));
    const QString project = QDir(dir).filePath("demo.hsproj");

    w.submit("DFM", {"dfm", project});
    REQUIRE(w.unfinished_jobs() == 1);
    const auto job_args = w.job_queue()->jobs()[0].args;
    CHECK(job_args[1] != project);  // its own copy
    CHECK(QFileInfo(job_args[1]).dir() == QFileInfo(project).dir());
    CHECK(QFileInfo::exists(job_args[1]));

    // The edit runs at once with the job still going, and the project on disk is the edited one.
    REQUIRE(w.change_role("escape_wheel", "plate"));
    CHECK(w.unfinished_jobs() == 1);
    CHECK(ohs::model::load_rig(project.toStdString()).components[0].role != ohs::model::Role::Unknown);

    w.wait_for_jobs();
    const auto& job = w.job_queue()->jobs()[0];
    INFO("out: " << job.out.toStdString() << " err: " << job.err.toStdString() << " exit " << job.exit_code);
    CHECK_FALSE(job.out.isEmpty());  // it ran on its copy and found the CAD file and the parts (DFM may flag findings: exit 1)
    CHECK((job.exit_code == 0 || job.exit_code == 1));
    CHECK_FALSE(QFileInfo::exists(job_args[1]));  // the copy is removed once the job has ended
    // The cache the job used is the project's own.
    CHECK(QFileInfo::exists(project + ".ohscache"));
}

// A movement of many plain parts: what a rig edit costs when the part table is long. Refilling the table used to
// measure every row on every insert (quadratic: 0.85 s at 166 parts), so each edit froze the window.
TEST_CASE("gui: a rig edit on a movement of 400 parts does not freeze the window", "[gui][perf]") {
    app();
    const QString dir = fresh_dir("many-parts");
    const QString step = QDir(dir).filePath("many.step"), project = QDir(dir).filePath("many.hsproj");
    std::vector<ohs::geometry::ExportPart> parts;
    for (int i = 0; i < 400; ++i) {
        const double w = 1.0 + 0.01 * (i % 17), h = 1.5 + 0.02 * (i % 11);
        parts.push_back({"part_" + std::to_string(i), BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), w, h, 0.4).Shape(), 3.0 * (i % 20), 3.0 * (i / 20), 0.0, 0.0});
    }
    ohs::geometry::export_step(parts, step.toStdString());
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"}).exit_code == 0);

    ohs::gui::MainWindow w;
    w.show();
    REQUIRE(w.open_project(project));
    CHECK(w.viewport()->scene()->movement().components.size() == 400);
    // The table is only laid out while it is on screen: show its tab.
    for (int i = 0; i < w.rig_tabs()->count(); ++i) {
        if (w.rig_tabs()->widget(i)->isAncestorOf(w.rig_table())) w.rig_tabs()->setCurrentIndex(i);
    }
    QApplication::processEvents();
    CHECK(w.rig_table()->isVisible());

    QElapsedTimer t;
    t.start();
    for (int i = 0; i < 4; ++i) REQUIRE(w.change_role("part_7", i % 2 ? "plate" : "bridge"));
    const double per_edit_ms = t.elapsed() / 4.0;
    WARN("rig edit on 400 parts: " << per_edit_ms << " ms");
    // Generous, as the machine may be busy: the quadratic table fill took several seconds here.
    CHECK(per_edit_ms < 600.0);
}
