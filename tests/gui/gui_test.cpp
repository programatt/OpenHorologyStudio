// SPDX-License-Identifier: Apache-2.0
// Scripted flow of the desktop application on Qt's offscreen platform: demonstration movement,
// rig list, picking and Explain, animation, exploded and cut-away views, analyses, report.
#include <algorithm>
#include <filesystem>
#include <memory>
#include <set>

#include <QAction>
#include <QApplication>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
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
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QListWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMenu>
#include <QUndoStack>
#include <QTreeWidgetItemIterator>
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

    // A command does wait for the queue; one started from a click handled during that wait is refused rather
    // than run inside the first.
    int nested = -1;
    QTimer::singleShot(0, &w, [&] { nested = w.run_command({"materials"}); });
    CHECK(w.run_command({"materials"}) == 0);
    CHECK(w.unfinished_jobs() == 0);
    CHECK(nested == 2);
    CHECK(w.run_command({"materials"}) == 0);  // and the next one runs
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
    // Timekeeping queues two jobs (the report and its plot data), then the torque chain.
    CHECK(w.job_list()->count() == 3);
    CHECK(w.unfinished_jobs() > 0);
    // The window stays usable while they run: a queued job can be cancelled before it starts.
    CHECK(w.cancel_job(2));
    w.wait_for_jobs();
    CHECK(w.unfinished_jobs() == 0);
    CHECK(w.job_list()->item(0)->text().startsWith("done"));
    CHECK(w.job_list()->item(1)->text().startsWith("done"));
    CHECK(w.job_list()->item(2)->text().startsWith("cancelled"));

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

// Hidden (run with `ohs_gui_tests "[bench]"`): where the time goes on the demonstration movement.
TEST_CASE("bench: scene load, frame and pick times of the demonstration movement", "[.][bench]") {
    app();
    const QString dir = fresh_dir("bench");
    const QString step = QDir(dir).filePath("demo.step"), project = QDir(dir).filePath("demo.hsproj");
    REQUIRE(ohs::cli::run_captured({"dev", "gen-movement", "-o", step.toStdString()}).exit_code == 0);
    REQUIRE(ohs::cli::run_captured({"import", step.toStdString(), "-o", project.toStdString(), "--beat-rate", "28800"})
                .exit_code == 0);
    QElapsedTimer t;
    t.start();
    auto scene = std::make_shared<ohs::gui::Scene>(ohs::gui::Scene::load(project.toStdString(), step.toStdString()));
    const qint64 load_ms = t.elapsed();
    std::size_t triangles = 0;
    for (const auto& p : scene->parts()) triangles += p.triangles.size();
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
    scene->set_show_trails(true, 10.0);
    frame("with trails");
    scene->set_show_trails(false);
    t.restart();
    (void)v.pick(QPoint(512, 384));
    const qint64 pick_ms = t.elapsed();
    WARN("load " << load_ms << " ms, " << triangles << " triangles, centre() " << centre_ms << " ms, pick " << pick_ms << " ms");
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
}
