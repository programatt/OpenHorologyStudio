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
#include <QMessageBox>
#include <QSignalBlocker>
#include <QUndoCommand>
#include <QUndoStack>
#include <QUndoView>
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
#include <QMenu>
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
#include "explain/explain.hpp"
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

QByteArray read_file(const QString& path) {
    QFile f(path);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

/// An edit of the project file made by an `ohs` command. The command has already run when the
/// command is pushed, so the first redo does nothing; later ones put the written text back.
class ProjectEdit : public QUndoCommand {
public:
    ProjectEdit(MainWindow* w, const QString& label, QByteArray before, QByteArray after)
        : QUndoCommand(label), w_(w), before_(std::move(before)), after_(std::move(after)) {}
    void redo() override {
        if (first_) {
            first_ = false;
            return;
        }
        w_->restore_project(after_);
    }
    void undo() override { w_->restore_project(before_); }

private:
    MainWindow* w_;
    QByteArray before_, after_;
    bool first_{true};
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
    log_->setMaximumBlockCount(20000);  // a long session's log would otherwise grow without end
    auto* log_dock = new QDockWidget(tr("Log"), this);
    log_dock->setObjectName("log_dock");
    log_dock->setWidget(log_);
    addDockWidget(Qt::BottomDockWidgetArea, log_dock);

    jobs_ = new QListWidget(this);
    auto* jobs_dock = new QDockWidget(tr("Jobs"), this);
    jobs_dock->setObjectName("jobs_dock");
    jobs_dock->setWidget(jobs_);
    addDockWidget(Qt::BottomDockWidgetArea, jobs_dock);
    queue_ = new JobQueue(worker_program(), this);
    connect(queue_, &JobQueue::changed, this, &MainWindow::on_job_changed);
    connect(queue_, &JobQueue::finished, this, &MainWindow::on_job_finished);
    connect(jobs_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { cancel_job(jobs_->currentRow()); });
    connect(jobs_, &QListWidget::currentRowChanged, this, [this](int row) {
        // Selecting a job shows its output again, which also works for jobs finished before a restart.
        if (row < 0 || row >= static_cast<int>(queue_->jobs().size())) return;
        const auto& job = queue_->jobs()[static_cast<std::size_t>(row)];
        if (job.state == JobQueue::State::Done || job.state == JobQueue::State::Failed) show_result(job.args, job.out, job.exit_code);
    });

    undo_ = new QUndoStack(this);
    auto* history_dock = new QDockWidget(tr("Edit history"), this);
    history_dock->setObjectName("history_dock");
    history_dock->setWidget(new QUndoView(undo_, this));  // click an entry to go back to that state
    addDockWidget(Qt::BottomDockWidgetArea, history_dock);
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
    file->addAction(tr("Export &video (MP4)..."), this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, tr("Export video"), "movement.mp4", tr("Video (*.mp4 *.webm *.gif)"));
        if (path.isEmpty()) return;
        const QString error = export_video(path);
        if (!error.isEmpty()) QMessageBox::warning(this, tr("Export video"), error);
    });
    file->addSeparator();
    file->addAction(tr("&Quit"), QKeySequence::Quit, this, &QWidget::close);
    auto* view = menuBar()->addMenu(tr("&View"));
    view->addAction(tr("Top (dial side)"), this, [this] { viewport_->set_view(0, 0); });
    view->addAction(tr("Isometric"), this, [this] { viewport_->set_view(35, -55); });
    view->addAction(tr("Side"), this, [this] { viewport_->set_view(0, -90); });
    view->addAction(tr("Bottom (movement back)"), this, [this] { viewport_->set_view(0, -180); });
    view->addAction(tr("Fit"), this, [this] { viewport_->fit(); });

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &MainWindow::on_timer);
    connect(viewport_, &Viewport::part_picked, this, &MainWindow::select_part);
    connect(tree_, &QTreeWidget::itemChanged, this, &MainWindow::on_tree_changed);
    connect(tree_, &QTreeWidget::itemSelectionChanged, this, &MainWindow::on_tree_selected);
    // Right click in the model tree or the rig list: show or hide all the parts of one role.
    for (QAbstractItemView* view : {static_cast<QAbstractItemView*>(tree_), static_cast<QAbstractItemView*>(rig_table_)}) {
        view->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(view, &QWidget::customContextMenuRequested, this, [this, view](const QPoint& pos) {
            if (!scene_) return;
            QMenu* menu = visibility_menu(view);
            menu->exec(view->viewport()->mapToGlobal(pos));
            menu->deleteLater();
        });
    }
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
    auto* mesh_row = new QHBoxLayout;
    mesh_box_ = new QComboBox;
    auto* declare = new QPushButton(tr("Declare that the selected part meshes with this one (undoable)"));
    mesh_row->addWidget(mesh_box_);
    mesh_row->addWidget(declare, 1);
    layout->addLayout(mesh_row);
    connect(declare, &QPushButton::clicked, this, &MainWindow::on_declare_mesh);
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

    auto* overlay_row = new QHBoxLayout;
    forces_check_ = new QCheckBox(tr("Tooth force arrows"));
    trails_check_ = new QCheckBox(tr("Motion trails over"));
    trail_seconds_ = new QDoubleSpinBox;
    trail_seconds_->setRange(0.1, 3600);
    trail_seconds_->setValue(10.0);
    trail_seconds_->setSuffix(tr(" s"));
    overlay_row->addWidget(forces_check_);
    overlay_row->addWidget(trails_check_);
    overlay_row->addWidget(trail_seconds_);
    layout->addLayout(overlay_row);

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
    connect(forces_check_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(trails_check_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(trail_seconds_, &QDoubleSpinBox::valueChanged, this, &MainWindow::on_view_changed);
    return page;
}

QWidget* MainWindow::build_analysis_page(const QString& title, const std::vector<std::pair<QString, QStringList>>& commands) {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    for (const auto& [label, args] : commands) {
        auto* b = new QPushButton(label);
        b->setProperty("ohs_args", args);
        connect(b, &QPushButton::clicked, this, &MainWindow::on_run_analysis);
        layout->addWidget(b);
    }
    if (title == tr("Analyse")) {
        auto* row = new QHBoxLayout;
        field_box_ = new QComboBox;
        field_box_->addItem(tr("von Mises stress"), "von_mises");
        field_box_->addItem(tr("Displacement"), "displacement");
        auto* clear = new QPushButton(tr("Clear the field plot"));
        row->addWidget(new QLabel(tr("Plot on the part:")));
        row->addWidget(field_box_, 1);
        row->addWidget(clear);
        layout->addLayout(row);
        connect(field_box_, &QComboBox::currentIndexChanged, this, [this] {
            if (!field_vtu_.isEmpty()) show_field(field_vtu_, field_part_, field_box_->currentData().toString());
        });
        connect(clear, &QPushButton::clicked, this, [this] { clear_field(); });
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

MainWindow::~MainWindow() {
    if (import_thread_) import_thread_->wait();  // a running import cannot be abandoned mid-write
}

void MainWindow::append_log(const QString& text) {
    log_->appendPlainText(text);
}

int MainWindow::run_command(const QStringList& args, QString* out) {
    if (importing_) {
        statusBar()->showMessage(tr("Wait for the project to finish opening"), 4000);
        return 2;
    }
    // Waiting for the queue keeps the window running, so a second click could start a command inside this
    // one; it would interleave with this edit's undo snapshot, so it is refused instead.
    if (in_command_) {
        statusBar()->showMessage(tr("Another command is waiting for the queued jobs; try again when it has run"), 5000);
        return 2;
    }
    in_command_ = true;
    if (const int queued = unfinished_jobs(); queued > 0) {
        statusBar()->showMessage(tr("Waiting for %1 queued job(s) to finish before running %2").arg(queued).arg(args.value(0)));
        wait_for_jobs();  // the command line runs one command at a time
        statusBar()->clearMessage();
    }
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
    in_command_ = false;
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
    undo_->clear();  // the history belongs to one project
    return load_project(project, step);
}

bool MainWindow::load_project(const QString& project, const QString& step) {
    try {
        return install_scene(project, step, std::make_shared<Scene>(Scene::load(project.toStdString(), step.toStdString())));
    } catch (const std::exception& e) {
        append_log(tr("Cannot open %1: %2").arg(project, e.what()));
        statusBar()->showMessage(tr("Cannot open the project; see the log"), 5000);
        return false;
    }
}

bool MainWindow::reload_rig() {
    if (!scene_) return false;
    const QString selected = QString::fromStdString(viewport_->selected());
    bool reused = false;
    try {
        reused = scene_->reload_rig(project_.toStdString());  // metadata only: the geometry is kept
    } catch (const std::exception& e) {
        append_log(tr("Cannot open %1: %2").arg(project_, e.what()));
        return false;
    }
    if (!reused) return load_project(project_, step_);
    refresh_panels();
    for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
        if ((*it)->parent()) {
            const QSignalBlocker block(tree_);
            (*it)->setCheckState(0, scene_->visible((*it)->text(0).toStdString()) ? Qt::Checked : Qt::Unchecked);
        }
    }
    if (!selected.isEmpty()) select_part(selected);  // the part that was changed stays selected in every view
    viewport_->update();
    return true;
}

bool MainWindow::install_scene(const QString& project, const QString& step, std::shared_ptr<Scene> scene) {
    scene_ = std::move(scene);
    const bool new_project = project != project_;
    project_ = project;
    step_ = step;
    if (new_project) attach_job_journal();
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

void MainWindow::import_step_async(const QString& step, const QString& project, double beat_rate_vph) {
    QStringList args{"import", step, "-o", project};
    if (beat_rate_vph > 0) args << "--beat-rate" << QString::number(beat_rate_vph);
    load_async(tr("Importing %1...").arg(QFileInfo(step).fileName()), {args}, project, step);
}

void MainWindow::open_project_async(const QString& project) {
    load_async(tr("Opening %1...").arg(QFileInfo(project).fileName()), {}, project, {});
}

void MainWindow::open_demo_async(const QString& dir) {
    QDir().mkpath(dir);
    const QString step = QDir(dir).filePath("demo.step"), project = QDir(dir).filePath("demo.hsproj");
    load_async(tr("Building the demonstration movement..."),
               {{"dev", "gen-movement", "-o", step}, {"import", step, "-o", project, "--beat-rate", "28800"}}, project, step);
}

void MainWindow::load_async(const QString& message, const std::vector<QStringList>& commands, const QString& project,
                            const QString& step) {
    if (importing_) {
        statusBar()->showMessage(tr("A project is already being opened"), 5000);
        return;
    }
    importing_ = true;
    statusBar()->showMessage(message);

    // The commands, the STEP read and the tessellation run off the UI thread; the window only installs the result.
    struct Result {
        std::vector<cli::Captured> captured;  ///< one per command run, stopping at the first that fails
        std::shared_ptr<Scene> scene;
        QString error;
    };
    auto result = std::make_shared<Result>();
    std::vector<std::vector<std::string>> cli_args;
    for (const QStringList& args : commands) {
        cli_args.emplace_back();
        for (const QString& a : args) cli_args.back().push_back(a.toStdString());
    }
    QThread* thread = QThread::create([result, cli_args, project, step] {
        for (const auto& args : cli_args) {
            result->captured.push_back(cli::run_captured(args));
            if (result->captured.back().exit_code != 0) return;
        }
        try {
            result->scene = std::make_shared<Scene>(Scene::load(project.toStdString(), step.toStdString()));
        } catch (const std::exception& e) {
            result->error = QString::fromUtf8(e.what());
        }
    });
    connect(thread, &QThread::finished, this, [this, thread, result, commands, step, project] {
        thread->deleteLater();
        import_thread_ = nullptr;
        importing_ = false;
        for (std::size_t i = 0; i < result->captured.size(); ++i) {
            QStringList quoted;
            for (const QString& a : commands[i]) quoted << shell_quote(a);
            const QString line = "ohs " + quoted.join(' ');
            history_ << line;
            append_log("$ " + line);
            const auto& captured = result->captured[i];
            if (!captured.err.empty()) append_log(QString::fromStdString(captured.err).trimmed());
            append_log(tr("exit code %1").arg(captured.exit_code));
            if (captured.exit_code != 0) {
                statusBar()->showMessage(tr("%1 failed; see the log").arg(commands[i].value(0)), 8000);
                return;
            }
        }
        if (!result->scene) {
            append_log(tr("Cannot open %1: %2").arg(project, result->error));
            statusBar()->showMessage(tr("Cannot open the project; see the log"), 5000);
            return;
        }
        undo_->clear();  // the history belongs to one project
        install_scene(project, step, result->scene);
        statusBar()->showMessage(tr("Opened %1").arg(QFileInfo(project).fileName()), 5000);
    });
    import_thread_ = thread;
    thread->start();
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
    viewport_->select(part.toStdString());
    return reload_rig();
}

void MainWindow::attach_job_journal() {
    jobs_->clear();
    const int resumed = queue_->load(project_ + ".jobs.json");
    if (resumed > 0) {
        append_log(tr("Resuming %1 job(s) that were unfinished when the application last ended").arg(resumed));
        queue_->resume();
    }
}

bool MainWindow::restore_project(const QByteArray& text) {
    if (project_.isEmpty()) return false;
    QFile f(project_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        append_log(tr("Cannot write %1").arg(project_));
        return false;
    }
    f.write(text);
    f.close();
    history_ << "# the project file was restored by undo or redo";
    return reload_rig();
}

bool MainWindow::edit_project(const QString& label, const QStringList& args) {
    if (project_.isEmpty()) return false;
    const QByteArray before = read_file(project_);
    if (run_command(args) != 0) return false;
    const QByteArray after = read_file(project_);
    if (!reload_rig()) {
        restore_project(before);
        return false;
    }
    if (after != before) undo_->push(new ProjectEdit(this, label, before, after));
    return true;
}

void MainWindow::refresh_panels() {
    rebuilding_ = true;
    const auto& m = scene_->movement();
    // One repaint of each view at the end instead of one per inserted row.
    tree_->setUpdatesEnabled(false);
    rig_table_->setUpdatesEnabled(false);

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
    const QString partner = mesh_box_->currentText();
    mesh_box_->clear();
    for (const auto& c : m.components) {
        if (c.features.teeth > 0) mesh_box_->addItem(QString::fromStdString(c.features.name));
    }
    if (const int keep = mesh_box_->findText(partner); keep >= 0) mesh_box_->setCurrentIndex(keep);
    tree_->setUpdatesEnabled(true);
    rig_table_->setUpdatesEnabled(true);
    rebuilding_ = false;

    QString s = tr("%1 | %2 parts").arg(QFileInfo(project_).fileName()).arg(m.components.size());
    if (!scene_->train_error().empty()) append_log(tr("Train not analysed: %1").arg(QString::fromStdString(scene_->train_error())));
    if (!scene_->can_animate()) s += tr(" | the animation is off (no train or no beat rate)");
    status_->setText(s);
    timeline_->setEnabled(scene_->can_animate());
    play_->setEnabled(scene_->can_animate());
}

bool MainWindow::show_field(const QString& vtu, const QString& part, const QString& field) {
    if (!scene_) return false;
    const int index = scene_->movement().find_component(part.toStdString());
    if (index < 0) {
        append_log(tr("Cannot plot a field: no part called %1").arg(part));
        return false;
    }
    const int mobile = scene_->movement().mobile_of(index);
    const double px = mobile >= 0 ? scene_->movement().mobiles[static_cast<std::size_t>(mobile)].center.x : 0.0;
    const double py = mobile >= 0 ? scene_->movement().mobiles[static_cast<std::size_t>(mobile)].center.y : 0.0;
    try {
        scene_->set_field(field_overlay_from_vtu(vtu.toStdString(), part.toStdString(), field.toStdString(), px, py));
    } catch (const std::exception& e) {
        append_log(tr("Cannot plot %1 from %2: %3").arg(field, vtu, e.what()));
        return false;
    }
    field_vtu_ = vtu;
    field_part_ = part;
    if (field_box_) {
        const int at = field_box_->findData(field);
        if (at >= 0 && at != field_box_->currentIndex()) {
            const QSignalBlocker block(field_box_);
            field_box_->setCurrentIndex(at);
        }
    }
    const FieldOverlay* f = scene_->field();
    append_log(tr("Plotting %1 on %2: %3 to %4").arg(QString::fromStdString(f->label), part).arg(f->lo, 0, 'g', 4).arg(f->hi, 0, 'g', 4));
    viewport_->update();
    return true;
}

void MainWindow::clear_field() {
    field_vtu_.clear();
    field_part_.clear();
    if (scene_) scene_->clear_field();
    viewport_->update();
}

QString MainWindow::export_video(const QString& path, const VideoOptions& options) {
    append_log(tr("Exporting %1 (%2 s of video, %3 s of movement time)").arg(path).arg(options.video_seconds).arg(options.movement_seconds));
    const QString error = gui::export_video(*viewport_, path, options, [this](int frame, int frames) {
        statusBar()->showMessage(tr("Rendering frame %1 of %2").arg(frame).arg(frames));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        return true;
    });
    statusBar()->showMessage(error.isEmpty() ? tr("Wrote %1").arg(path) : error, 8000);
    append_log(error.isEmpty() ? tr("Wrote %1").arg(path) : tr("Video export failed: %1").arg(error));
    return error;
}

void MainWindow::select_part(const QString& name) {
    if (!scene_) return;
    viewport_->select(name.toStdString());
    if (name.isEmpty()) {
        explain_->clear();
        return;
    }
    // The same text as `ohs explain`, from the rig and train already in memory: no waiting for queued jobs
    // (run_command would) and no re-reading of the project file on every click.
    QString text;
    if (scene_->train_error().empty()) {
        try {
            text = QString::fromStdString(explain::to_text(explain::explain_part(scene_->movement(), scene_->train(), name.toStdString())));
        } catch (const std::exception&) {
        }
    }
    explain_->setPlainText(!text.isEmpty() ? text : tr("No explanation for '%1'.").arg(name));
    rebuilding_ = true;
    for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
        if ((*it)->text(0) == name) tree_->setCurrentItem(*it);
    }
    for (int row = 0; row < rig_table_->rowCount(); ++row) {
        const auto* item = rig_table_->item(row, 0);
        if (item && item->text() == name) {
            rig_table_->selectRow(row);
            rig_table_->scrollToItem(item);
            break;
        }
    }
    rebuilding_ = false;
}

bool MainWindow::set_role_visible(const QString& role, bool visible) {
    if (!scene_) return false;
    bool any = false;
    for (const ScenePart& part : scene_->parts()) {
        if (QString::fromUtf8(model::to_string(part.role).data()) != role) continue;
        scene_->set_visible(part.name, visible);
        any = true;
    }
    if (!any) return false;
    {
        const QSignalBlocker block(tree_);  // the scene is already set: no per-item round trip
        for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
            if ((*it)->parent() && (*it)->text(1) == role)
                (*it)->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
        }
    }
    viewport_->update();
    return true;
}

QMenu* MainWindow::visibility_menu(QWidget* parent) {
    auto* menu = new QMenu(parent);
    menu->addSection(tr("Show parts by role"));
    if (!scene_) return menu;
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::Role::Screw); ++r) {
        const auto role = static_cast<model::Role>(r);
        int count = 0, shown = 0;
        for (const ScenePart& part : scene_->parts()) {
            if (part.role != role) continue;
            ++count;
            if (scene_->visible(part.name)) ++shown;
        }
        if (count == 0) continue;
        const QString name = QString::fromUtf8(model::to_string(role).data());
        const QString label = shown == count || shown == 0 ? tr("%1 (%2)").arg(name).arg(count)
                                                          : tr("%1 (%2 of %3 shown)").arg(name).arg(shown).arg(count);
        QAction* action = menu->addAction(label);
        action->setData(name);
        action->setCheckable(true);
        action->setChecked(shown == count);
        connect(action, &QAction::toggled, this, [this, name](bool on) { set_role_visible(name, on); });
    }
    return menu;
}

void MainWindow::on_tree_selected() {
    if (rebuilding_ || !scene_) return;
    const auto items = tree_->selectedItems();
    if (items.isEmpty() || !items.first()->parent()) return;
    select_part(items.first()->text(0));
}

void MainWindow::on_tree_changed(QTreeWidgetItem* item) {
    if (rebuilding_ || !scene_ || !item || !item->parent()) return;
    // Only the item that changed: ticking one part used to re-apply every part's check state.
    const std::string name = item->text(0).toStdString();
    const bool visible = item->checkState(0) == Qt::Checked;
    if (scene_->visible(name) == visible) return;  // a change other than the tick
    scene_->set_visible(name, visible);
    viewport_->update();
}

void MainWindow::set_time(double seconds) {
    time_ = seconds;
    if (scene_) scene_->set_time(seconds);
    viewport_->interactive_update();
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
    if (forces_check_->isChecked() && scene_->forces().empty()) {
        try {
            scene_->compute_forces();
        } catch (const std::exception& e) {
            append_log(tr("No force arrows: %1").arg(e.what()));
            const QSignalBlocker block(forces_check_);
            forces_check_->setChecked(false);
        }
    }
    scene_->set_show_forces(forces_check_->isChecked());
    scene_->set_show_trails(trails_check_->isChecked(), trail_seconds_->value());
    viewport_->interactive_update();
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
    // The shock case also writes its result for the viewport's field plot.
    const int shock = args.indexOf("--case");
    if (shock >= 0 && args.value(shock + 1) == "shock" && !args.contains("--vtu")) {
        const QString part = args.value(args.indexOf("--part") + 1);
        args << "--vtu" << QFileInfo(project_).dir().filePath(part + ".shock.vtu");
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
    return edit_project(tr("Role of %1: %2 to %3").arg(part, from, role), {"rig", "set", project_, part, "--role", role});
}

bool MainWindow::declare_mesh(const QString& a, const QString& b) {
    if (!scene_ || a == b) return false;
    return edit_project(tr("Mesh %1 with %2").arg(a, b), {"rig", "mesh", project_, a, b});
}

void MainWindow::on_declare_mesh() {
    if (viewport_->selected().empty() || mesh_box_->currentText().isEmpty()) {
        statusBar()->showMessage(tr("Select a part and a partner first"), 4000);
        return;
    }
    declare_mesh(QString::fromStdString(viewport_->selected()), mesh_box_->currentText());
}

void MainWindow::on_apply_role() {
    if (viewport_->selected().empty()) {
        statusBar()->showMessage(tr("Select a part first"), 4000);
        return;
    }
    change_role(QString::fromStdString(viewport_->selected()), role_box_->currentText());
}

QString MainWindow::worker_program() {
    const QString env = qEnvironmentVariable("OHS_EXE");
    if (!env.isEmpty()) return env;
    const QString beside = QCoreApplication::applicationDirPath() + "/ohs";  // an installed layout
    if (QFileInfo::exists(beside)) return beside;
#ifdef OHS_CLI_EXECUTABLE
    return QString::fromUtf8(OHS_CLI_EXECUTABLE);  // the build tree
#else
    return "ohs";
#endif
}

int MainWindow::unfinished_jobs() const { return queue_->unfinished(); }

void MainWindow::submit(const QString& label, const QStringList& args) { queue_->submit(label, args); }

bool MainWindow::cancel_job(int index) { return queue_->cancel(index); }

void MainWindow::on_job_changed(int index) {
    const auto& job = queue_->jobs()[static_cast<std::size_t>(index)];
    while (jobs_->count() <= index) jobs_->addItem(QString());
    QString text;
    switch (job.state) {
        case JobQueue::State::Queued: text = job.attempts > 0 ? tr("queued again after a restart: %1") : tr("queued: %1"); text = text.arg(job.label); break;
        case JobQueue::State::Running: text = tr("running: %1").arg(job.label); break;
        case JobQueue::State::Done: text = tr("done: %1").arg(job.label); break;
        case JobQueue::State::Failed: text = tr("failed (%2): %1").arg(job.label).arg(job.exit_code); break;
        case JobQueue::State::Cancelled: text = tr("cancelled: %1").arg(job.label); break;
    }
    jobs_->item(index)->setText(text);
}

void MainWindow::on_job_finished(int index) {
    const auto& job = queue_->jobs()[static_cast<std::size_t>(index)];
    QStringList quoted;
    for (const QString& s : job.args) quoted << shell_quote(s);
    history_ << "ohs " + quoted.join(' ');
    append_log("$ ohs " + quoted.join(' '));
    if (!job.err.isEmpty()) append_log(job.err.trimmed());
    append_log(tr("exit code %1").arg(job.exit_code));
    show_result(job.args, job.out, job.exit_code);
    const int vtu = job.args.indexOf("--vtu");
    if (vtu >= 0 && job.exit_code == 0 && QFileInfo::exists(job.args.value(vtu + 1))) {
        const QString field = field_box_ ? field_box_->currentData().toString() : QString("von_mises");
        show_field(job.args.value(vtu + 1), job.args.value(job.args.indexOf("--part") + 1), field);
    }
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

void MainWindow::wait_for_jobs() { queue_->wait(); }

void MainWindow::on_open() {
    const QString path = QFileDialog::getOpenFileName(this, tr("Open project"), {}, tr("OHS projects (*.hsproj *.toml)"));
    if (!path.isEmpty()) open_project_async(path);
}

void MainWindow::on_import() {
    const QString step = QFileDialog::getOpenFileName(this, tr("Import CAD file"), {}, tr("CAD files (*.step *.stp *.iges *.igs *.dxf);;STEP (*.step *.stp);;IGES (*.iges *.igs);;DXF (*.dxf)"));
    if (step.isEmpty()) return;
    import_step_async(step, QFileInfo(step).dir().filePath(QFileInfo(step).completeBaseName() + ".hsproj"), 0);
}

void MainWindow::on_demo() {
    open_demo_async(QDir::temp().filePath("ohs-demo"));
}

}  // namespace ohs::gui
