// SPDX-License-Identifier: Apache-2.0
#include "gui/main_window.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QListWidget>
#include <QThread>
#include <QUndoCommand>
#include <QUndoStack>
#include <QtConcurrent/QtConcurrentRun>
#include <QRegularExpression>
#include <QTreeWidgetItemIterator>
#include <QCheckBox>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <nlohmann/json.hpp>

#include "cli/cli_run.hpp"
#include "core/error.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"

namespace ohs::gui {

namespace {

QString shell_quote(const QString& s) {
    if (!s.isEmpty() && !s.contains(QRegularExpression("[^A-Za-z0-9_./=:,+@%-]"))) return s;
    QString q = s;
    q.replace("'", "'\\''");
    return "'" + q + "'";
}

/// Changing a part's role, undoable: redo applies the new role, undo puts the old one back.
class RoleCommand : public QUndoCommand {
public:
    RoleCommand(MainWindow* w, QString part, QString from, QString to)
        : QUndoCommand(QObject::tr("Role of %1: %2 to %3").arg(part, from, to)),
          w_(w), part_(std::move(part)), from_(std::move(from)), to_(std::move(to)) {}
    void redo() override { ok = w_->set_role(part_, to_); }
    void undo() override { w_->set_role(part_, from_); }
    bool ok{false};

private:
    MainWindow* w_;
    QString part_, from_, to_;
};

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(tr("Open Horology Studio"));
    resize(1280, 800);
    build_ui();
}

void MainWindow::build_ui() {
    viewport_ = new Viewport(this);
    workspaces_ = new QTabWidget(this);
    workspaces_->setMinimumWidth(360);
    workspaces_->addTab(build_rig_page(), tr("Import && Rig"));
    workspaces_->addTab(build_animate_page(), tr("Animate"));
    workspaces_->addTab(build_analysis_page(tr("Simulate"), {{tr("Torque chain"), {"torque", "@project"}},
                                                             {tr("Timekeeping"), {"timekeeping", "@project"}},
                                                             {tr("Timegrapher (60 s)"), {"timegrapher", "@project"}},
                                                             {tr("Check the models agree"), {"verify", "levels"}}}),
                        tr("Simulate"));
    workspaces_->addTab(build_analysis_page(tr("Analyse"), {{tr("Tooth bending of the selected part"),
                                                             {"fea", "@project", "--part", "@part"}},
                                                            {tr("Shock and modes of the selected part"),
                                                             {"fea", "@project", "--part", "@part", "--case", "shock"}},
                                                            {tr("Materials"), {"materials"}}}),
                        tr("Analyse"));
    workspaces_->addTab(build_analysis_page(tr("Manufacture"), {{tr("Design for manufacture"), {"dfm", "@project"}},
                                                                {tr("Collisions in motion"), {"collisions", "@project"}},
                                                                {tr("Tolerance stack-up"), {"tolerance", "@project"}}}),
                        tr("Manufacture"));
    workspaces_->addTab(build_analysis_page(tr("Report"), {{tr("Write the report (HTML and PDF)"),
                                                            {"report", "@project", "-o", "@report"}}}),
                        tr("Report"));

    auto* split = new QSplitter(Qt::Horizontal, this);
    split->addWidget(viewport_);
    split->addWidget(workspaces_);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);
    setCentralWidget(split);

    // Model tree: mobiles with their parts; ticking a part shows or hides it.
    tree_ = new QTreeWidget(this);
    tree_->setHeaderLabels({tr("Model"), tr("Role")});
    auto* tree_dock = new QDockWidget(tr("Model"), this);
    tree_dock->setObjectName("model_dock");
    tree_dock->setWidget(tree_);
    addDockWidget(Qt::LeftDockWidgetArea, tree_dock);

    explain_ = new QPlainTextEdit(this);
    explain_->setReadOnly(true);
    explain_->setPlaceholderText(tr("Click a part to see what it does, with this movement's numbers."));
    auto* explain_dock = new QDockWidget(tr("Explain"), this);
    explain_dock->setObjectName("explain_dock");
    explain_dock->setWidget(explain_);
    addDockWidget(Qt::LeftDockWidgetArea, explain_dock);

    log_ = new QPlainTextEdit(this);
    log_->setReadOnly(true);
    auto* log_dock = new QDockWidget(tr("Log"), this);
    log_dock->setObjectName("log_dock");
    log_dock->setWidget(log_);
    addDockWidget(Qt::BottomDockWidgetArea, log_dock);

    jobs_ = new QListWidget(this);
    auto* jobs_dock = new QDockWidget(tr("Jobs"), this);
    jobs_dock->setObjectName("jobs_dock");
    jobs_dock->setWidget(jobs_);
    addDockWidget(Qt::BottomDockWidgetArea, jobs_dock);
    connect(&watcher_, &QFutureWatcher<cli::Captured>::finished, this, &MainWindow::on_job_finished);
    connect(jobs_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { cancel_job(jobs_->currentRow()); });

    undo_ = new QUndoStack(this);
    auto* edit = menuBar()->addMenu(tr("&Edit"));
    edit->addAction(undo_->createUndoAction(this, tr("&Undo")));
    edit->addAction(undo_->createRedoAction(this, tr("&Redo")));
    edit->actions().at(0)->setShortcut(QKeySequence::Undo);
    edit->actions().at(1)->setShortcut(QKeySequence::Redo);

    status_ = new QLabel(tr("No project open"), this);
    statusBar()->addWidget(status_);

    auto* file = menuBar()->addMenu(tr("&File"));
    file->addAction(tr("&Open project..."), QKeySequence::Open, this, &MainWindow::on_open);
    file->addAction(tr("&Import STEP..."), this, &MainWindow::on_import);
    file->addAction(tr("Open the &demonstration movement"), this, &MainWindow::on_demo);
    file->addAction(tr("&Save session as script..."), this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, tr("Save session"), "session.sh");
        if (!path.isEmpty()) save_script(path);
    });
    file->addSeparator();
    file->addAction(tr("&Quit"), QKeySequence::Quit, this, &QWidget::close);
    auto* view = menuBar()->addMenu(tr("&View"));
    view->addAction(tr("Top (dial side)"), this, [this] { viewport_->set_view(0, -90); });
    view->addAction(tr("Isometric"), this, [this] { viewport_->set_view(35, -55); });
    view->addAction(tr("Side"), this, [this] { viewport_->set_view(0, -5); });
    view->addAction(tr("Fit"), this, [this] { viewport_->fit(); });

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &MainWindow::on_timer);
    connect(viewport_, &Viewport::part_picked, this, &MainWindow::select_part);
    connect(tree_, &QTreeWidget::itemChanged, this, [this] { on_tree_changed(); });
    connect(tree_, &QTreeWidget::itemSelectionChanged, this, &MainWindow::on_tree_selected);
}

QWidget* MainWindow::build_rig_page() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    auto* row = new QHBoxLayout;
    auto* open = new QPushButton(tr("Open project..."));
    auto* import = new QPushButton(tr("Import STEP..."));
    auto* demo = new QPushButton(tr("Demonstration movement"));
    for (auto* b : {open, import, demo}) row->addWidget(b);
    layout->addLayout(row);
    connect(open, &QPushButton::clicked, this, &MainWindow::on_open);
    connect(import, &QPushButton::clicked, this, &MainWindow::on_import);
    connect(demo, &QPushButton::clicked, this, &MainWindow::on_demo);

    rig_table_ = new QTableWidget(0, 4, page);
    rig_table_->setHorizontalHeaderLabels({tr("Part"), tr("Role"), tr("Confidence"), tr("Decided by")});
    rig_table_->horizontalHeader()->setStretchLastSection(true);
    rig_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    rig_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    layout->addWidget(new QLabel(tr("Rig confidence list (low confidence first-check these)")));
    layout->addWidget(rig_table_);
    auto* role_row = new QHBoxLayout;
    role_box_ = new QComboBox;
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::Role::Screw); ++r)
        role_box_->addItem(QString::fromUtf8(model::to_string(static_cast<model::Role>(r)).data()));
    auto* apply = new QPushButton(tr("Set role of the selected part (undoable)"));
    role_row->addWidget(role_box_);
    role_row->addWidget(apply, 1);
    layout->addLayout(role_row);
    connect(apply, &QPushButton::clicked, this, &MainWindow::on_apply_role);
    connect(rig_table_, &QTableWidget::itemSelectionChanged, this, [this] {
        const auto rows = rig_table_->selectionModel()->selectedRows();
        if (!rows.isEmpty() && !rebuilding_) select_part(rig_table_->item(rows.first().row(), 0)->text());
    });
    return page;
}

QWidget* MainWindow::build_animate_page() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);

    auto* row = new QHBoxLayout;
    play_ = new QPushButton(tr("Play"));
    play_->setCheckable(true);
    stepped_ = new QCheckBox(tr("One escapement step per beat"));
    speed_ = new QDoubleSpinBox;
    speed_->setRange(0.01, 100000);
    speed_->setValue(1.0);
    speed_->setPrefix(tr("Time x "));
    row->addWidget(play_);
    row->addWidget(stepped_);
    row->addWidget(speed_);
    layout->addLayout(row);

    timeline_ = new QSlider(Qt::Horizontal);
    timeline_->setRange(0, 10000);  // 0..60 s of movement time
    layout->addWidget(new QLabel(tr("Timeline (60 s of movement time)")));
    layout->addWidget(timeline_);

    explode_ = new QSlider(Qt::Horizontal);
    explode_->setRange(0, 100);
    layout->addWidget(new QLabel(tr("Exploded view")));
    layout->addWidget(explode_);

    section_ = new QCheckBox(tr("Cut-away plane (Z)"));
    section_offset_ = new QSlider(Qt::Horizontal);
    section_offset_->setRange(0, 1000);
    section_offset_->setValue(500);
    layout->addWidget(section_);
    layout->addWidget(section_offset_);
    layout->addStretch();

    connect(play_, &QPushButton::toggled, this, [this](bool on) { set_playing(on); });
    connect(timeline_, &QSlider::valueChanged, this, &MainWindow::on_timeline);
    for (auto* s : {explode_, section_offset_}) connect(s, &QSlider::valueChanged, this, &MainWindow::on_view_changed);
    connect(section_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(stepped_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    return page;
}

QWidget* MainWindow::build_analysis_page(const QString&, const std::vector<std::pair<QString, QStringList>>& commands) {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    for (const auto& [label, args] : commands) {
        auto* b = new QPushButton(label);
        b->setProperty("ohs_args", args);
        connect(b, &QPushButton::clicked, this, &MainWindow::on_run_analysis);
        layout->addWidget(b);
    }
    // The results panel is shared by every analysis workspace through the first page that makes it.
    if (!results_) {
        results_ = new QPlainTextEdit;
        results_->setReadOnly(true);
        results_->setPlaceholderText(tr("Results of the analyses appear here."));
        plot_ = new PlotWidget;
        auto* holder = new QWidget;
        auto* hl = new QVBoxLayout(holder);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->addWidget(plot_);
        hl->addWidget(results_);
        auto* dock = new QDockWidget(tr("Results"), this);
        dock->setObjectName("results_dock");
        dock->setWidget(holder);
        addDockWidget(Qt::BottomDockWidgetArea, dock);
    }
    layout->addStretch();
    return page;
}

void MainWindow::append_log(const QString& text) {
    log_->appendPlainText(text);
}

int MainWindow::run_command(const QStringList& args, QString* out) {
    wait_for_jobs();  // the command line runs one command at a time
    std::vector<std::string> a;
    for (const QString& s : args) a.push_back(s.toStdString());
    QStringList quoted;
    for (const QString& s : args) quoted << shell_quote(s);
    const QString line = "ohs " + quoted.join(' ');
    history_ << line;
    append_log("$ " + line);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const auto captured = cli::run_captured(a);
    QApplication::restoreOverrideCursor();
    const QString text = QString::fromStdString(captured.out);
    if (out) *out = text;
    if (!captured.err.empty()) append_log(QString::fromStdString(captured.err).trimmed());
    append_log(tr("exit code %1").arg(captured.exit_code));
    return captured.exit_code;
}

bool MainWindow::save_script(const QString& path) const {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream t(&f);
    t << "#!/bin/sh\n# Open Horology Studio session\nset -e\n";
    for (const QString& line : history_) t << line << '\n';
    return true;
}

bool MainWindow::open_project(const QString& project, const QString& step) {
    try {
        scene_ = std::make_shared<Scene>(Scene::load(project.toStdString(), step.toStdString()));
    } catch (const std::exception& e) {
        append_log(tr("Cannot open %1: %2").arg(project, e.what()));
        statusBar()->showMessage(tr("Cannot open the project; see the log"), 5000);
        return false;
    }
    project_ = project;
    step_ = step;
    timeline_->setValue(0);
    time_ = 0;
    viewport_->set_scene(scene_);
    refresh_panels();
    append_log(tr("Opened %1: %2 parts, %3 mobiles, %4 meshes")
                   .arg(project)
                   .arg(scene_->movement().components.size())
                   .arg(scene_->movement().mobiles.size())
                   .arg(scene_->movement().meshes.size()));
    return true;
}

bool MainWindow::import_step(const QString& step, const QString& project, double beat_rate_vph) {
    QStringList args{"import", step, "-o", project};
    if (beat_rate_vph > 0) args << "--beat-rate" << QString::number(beat_rate_vph);
    if (run_command(args) != 0) return false;
    return open_project(project, step);
}

bool MainWindow::open_demo(const QString& dir) {
    QDir().mkpath(dir);
    const QString step = QDir(dir).filePath("demo.step");
    if (run_command({"dev", "gen-movement", "-o", step}) != 0) return false;
    return import_step(step, QDir(dir).filePath("demo.hsproj"), 28800);
}

bool MainWindow::set_role(const QString& part, const QString& role) {
    if (project_.isEmpty()) return false;
    if (run_command({"rig", "set", project_, part, "--role", role}) != 0) return false;
    return open_project(project_, step_);
}

void MainWindow::refresh_panels() {
    rebuilding_ = true;
    const auto& m = scene_->movement();

    tree_->clear();
    std::vector<QTreeWidgetItem*> mobile_items;
    for (const auto& mob : m.mobiles) {
        auto* item = new QTreeWidgetItem(tree_, {QString::fromStdString(mob.name), tr("mobile")});
        item->setExpanded(true);
        mobile_items.push_back(item);
    }
    auto* fixed = new QTreeWidgetItem(tree_, {tr("fixed parts"), QString()});
    fixed->setExpanded(true);
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const auto& c = m.components[i];
        const int mob = m.mobile_of(static_cast<int>(i));
        auto* parent = mob >= 0 ? mobile_items[mob] : fixed;
        auto* item = new QTreeWidgetItem(parent, {QString::fromStdString(c.features.name),
                                                  QString::fromUtf8(model::to_string(c.role).data())});
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
    }

    rig_table_->setRowCount(static_cast<int>(m.components.size()));
    std::vector<int> order(m.components.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return m.components[a].role_confidence < m.components[b].role_confidence; });
    for (std::size_t row = 0; row < order.size(); ++row) {
        const auto& c = m.components[order[row]];
        rig_table_->setItem(static_cast<int>(row), 0, new QTableWidgetItem(QString::fromStdString(c.features.name)));
        rig_table_->setItem(static_cast<int>(row), 1,
                            new QTableWidgetItem(QString::fromUtf8(model::to_string(c.role).data())));
        rig_table_->setItem(static_cast<int>(row), 2, new QTableWidgetItem(QString::number(c.role_confidence, 'f', 2)));
        rig_table_->setItem(static_cast<int>(row), 3, new QTableWidgetItem(QString::fromStdString(c.role_source)));
    }
    rebuilding_ = false;

    QString s = tr("%1 | %2 parts").arg(QFileInfo(project_).fileName()).arg(m.components.size());
    if (!scene_->train_error().empty()) append_log(tr("Train not analysed: %1").arg(QString::fromStdString(scene_->train_error())));
    if (!scene_->can_animate()) s += tr(" | the animation is off (no train or no beat rate)");
    status_->setText(s);
    timeline_->setEnabled(scene_->can_animate());
    play_->setEnabled(scene_->can_animate());
}

void MainWindow::select_part(const QString& name) {
    if (!scene_) return;
    viewport_->select(name.toStdString());
    if (name.isEmpty()) {
        explain_->clear();
        return;
    }
    QString text;
    const int code = run_command({"explain", project_, name}, &text);
    explain_->setPlainText(code == 0 ? text : tr("No explanation for '%1'.").arg(name));
    rebuilding_ = true;
    for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
        if ((*it)->text(0) == name) tree_->setCurrentItem(*it);
    }
    rebuilding_ = false;
}

void MainWindow::on_tree_selected() {
    if (rebuilding_ || !scene_) return;
    const auto items = tree_->selectedItems();
    if (items.isEmpty() || !items.first()->parent()) return;
    select_part(items.first()->text(0));
}

void MainWindow::on_tree_changed() {
    if (rebuilding_ || !scene_) return;
    for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
        if (!(*it)->parent()) continue;
        scene_->set_visible((*it)->text(0).toStdString(), (*it)->checkState(0) == Qt::Checked);
    }
    viewport_->update();
}

void MainWindow::set_time(double seconds) {
    time_ = seconds;
    if (scene_) scene_->set_time(seconds);
    viewport_->update();
}

void MainWindow::set_playing(bool playing) {
    if (playing && scene_ && scene_->can_animate()) timer_->start();
    else timer_->stop();
    if (play_->isChecked() != playing) play_->setChecked(playing);
    play_->setText(playing ? tr("Pause") : tr("Play"));
}

void MainWindow::on_timer() {
    // Movement seconds advance by speed x real seconds; the timeline wraps at 60 s.
    double t = time_ + speed_->value() * timer_->interval() / 1000.0;
    if (t > 60.0) t = std::fmod(t, 60.0);
    timeline_->blockSignals(true);
    timeline_->setValue(static_cast<int>(t / 60.0 * timeline_->maximum()));
    timeline_->blockSignals(false);
    set_time(t);
}

void MainWindow::on_timeline(int value) {
    set_time(60.0 * value / timeline_->maximum());
}

void MainWindow::on_view_changed() {
    if (!scene_) return;
    scene_->set_explode(explode_->value() / 100.0);
    scene_->set_stepped(stepped_->isChecked());
    SectionPlane plane;
    plane.enabled = section_->isChecked();
    const Point3 c = scene_->centre();
    const double r = scene_->radius();
    plane.offset = c[2] + (section_offset_->value() / 500.0 - 1.0) * r;
    scene_->set_section(plane);
    viewport_->update();
}

void MainWindow::on_run_analysis() {
    if (project_.isEmpty()) {
        results_->setPlainText(tr("Open a project first."));
        return;
    }
    const QStringList template_args = sender()->property("ohs_args").toStringList();
    QStringList args;
    for (const QString& a : template_args) {
        if (a == "@project") args << project_;
        else if (a == "@part") {
            if (viewport_->selected().empty()) {
                results_->setPlainText(tr("Click a part first."));
                return;
            }
            args << QString::fromStdString(viewport_->selected());
        } else if (a == "@report") args << QFileInfo(project_).dir().filePath("report");
        else args << a;
    }
    submit(static_cast<QPushButton*>(sender())->text(), args);
    if (args.value(0) == "timekeeping") submit(tr("Timekeeping plot data"), {"--json", "timekeeping", project_});
}

bool MainWindow::change_role(const QString& part, const QString& role) {
    if (!scene_) return false;
    const int index = scene_->movement().find_component(part.toStdString());
    if (index < 0) return false;
    const QString from = QString::fromUtf8(model::to_string(scene_->movement().components[index].role).data());
    if (from == role) return true;
    auto* cmd = new RoleCommand(this, part, from, role);
    undo_->push(cmd);  // runs redo()
    return cmd->ok;
}

void MainWindow::on_apply_role() {
    if (viewport_->selected().empty()) {
        statusBar()->showMessage(tr("Select a part first"), 4000);
        return;
    }
    change_role(QString::fromStdString(viewport_->selected()), role_box_->currentText());
}

int MainWindow::unfinished_jobs() const {
    int n = 0;
    for (const Job& j : job_log_)
        if (j.state == Job::State::Queued || j.state == Job::State::Running) ++n;
    return n;
}

void MainWindow::submit(const QString& label, const QStringList& args) {
    job_log_.push_back({label, args, Job::State::Queued});
    jobs_->addItem(tr("queued: %1").arg(label));
    start_next_job();
}

bool MainWindow::cancel_job(int index) {
    if (index < 0 || index >= static_cast<int>(job_log_.size()) || job_log_[index].state != Job::State::Queued) return false;
    job_log_[index].state = Job::State::Cancelled;
    jobs_->item(index)->setText(tr("cancelled: %1").arg(job_log_[index].label));
    return true;
}

void MainWindow::start_next_job() {
    if (running_job_ >= 0) return;
    for (std::size_t i = 0; i < job_log_.size(); ++i) {
        if (job_log_[i].state != Job::State::Queued) continue;
        job_log_[i].state = Job::State::Running;
        running_job_ = static_cast<int>(i);
        jobs_->item(running_job_)->setText(tr("running: %1").arg(job_log_[i].label));
        std::vector<std::string> a;
        for (const QString& s : job_log_[i].args) a.push_back(s.toStdString());
        watcher_.setFuture(QtConcurrent::run([a] { return cli::run_captured(a); }));
        return;
    }
}

void MainWindow::on_job_finished() {
    if (running_job_ < 0) return;
    Job& job = job_log_[running_job_];
    const cli::Captured captured = watcher_.result();
    QStringList quoted;
    for (const QString& s : job.args) quoted << shell_quote(s);
    history_ << "ohs " + quoted.join(' ');
    append_log("$ ohs " + quoted.join(' '));
    if (!captured.err.empty()) append_log(QString::fromStdString(captured.err).trimmed());
    append_log(tr("exit code %1").arg(captured.exit_code));
    job.state = captured.exit_code == 0 ? Job::State::Done : Job::State::Failed;
    jobs_->item(running_job_)->setText(captured.exit_code == 0 ? tr("done: %1").arg(job.label)
                                                               : tr("failed (%2): %1").arg(job.label).arg(captured.exit_code));
    const QStringList args = job.args;
    running_job_ = -1;
    show_result(args, QString::fromStdString(captured.out), captured.exit_code);
    start_next_job();
}

void MainWindow::show_result(const QStringList& args, const QString& out, int code) {
    // The JSON timekeeping job only feeds the plot: rate against amplitude from the full, half and low wind.
    if (args.size() >= 2 && args[0] == "--json" && args[1] == "timekeeping") {
        if (code != 0) return;
        try {
            const auto j = nlohmann::json::parse(out.toStdString());
            std::vector<std::pair<double, double>> pts;
            for (const char* key : {"full", "half", "low"})
                if (j.contains(key)) pts.emplace_back(j[key].at("amplitude_deg").get<double>(), j[key].at("rate_s_per_day").get<double>());
            plot_->set_data(tr("Isochronism: rate against amplitude"), tr("amplitude (degrees)"), tr("rate (s/day)"), std::move(pts));
        } catch (const std::exception& e) {
            append_log(tr("Plot data unreadable: %1").arg(e.what()));
        }
        return;
    }
    results_->setPlainText(out + (code == 0 ? QString() : tr("\n(exit code %1: see the log)").arg(code)));
}

void MainWindow::wait_for_jobs() {
    while (unfinished_jobs() > 0) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        QThread::msleep(5);
    }
}

void MainWindow::on_open() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Open project"), {}, tr("OHS projects (*.hsproj *.toml)"));
    if (!path.isEmpty()) open_project(path);
}

void MainWindow::on_import() {
    const QString step = QFileDialog::getOpenFileName(this, tr("Import STEP"), {}, tr("STEP (*.step *.stp)"));
    if (step.isEmpty()) return;
    import_step(step, QFileInfo(step).dir().filePath(QFileInfo(step).completeBaseName() + ".hsproj"), 0);
}

void MainWindow::on_demo() {
    open_demo(QDir::temp().filePath("ohs-demo"));
}

}  // namespace ohs::gui
