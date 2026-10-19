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
#include <QCloseEvent>
#include <QFontDatabase>
#include <QGridLayout>
#include <QGroupBox>
#include <QMenu>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QSettings>
#include <QStyle>
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
#include <QToolBar>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <nlohmann/json.hpp>

#include "cli/cli_run.hpp"
#include "core/error.hpp"
#include "explain/explain.hpp"
#include "geometry/procedural.hpp"
#include "geometry/step_io.hpp"
#include "gui/theme.hpp"

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

/// A play triangle or pause bars in the given colour. The style's own media icons are drawn light, so they
/// vanish on the light theme.
QIcon media_icon(bool pause, const QColor& ink) {
    QPixmap pixmap(32, 32);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(ink);
    if (pause) {
        painter.drawRoundedRect(QRectF(8, 6, 6, 20), 1.5, 1.5);
        painter.drawRoundedRect(QRectF(18, 6, 6, 20), 1.5, 1.5);
    } else {
        const QPointF triangle[3] = {{9, 5}, {27, 16}, {9, 27}};
        painter.drawPolygon(triangle, 3);
    }
    return QIcon(pixmap);
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(tr("Open Horology Studio"));
    resize(1440, 900);
    build_ui();
    restore_layout();
}

void MainWindow::build_ui() {
    setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks | QMainWindow::AllowNestedDocks);
    viewport_ = new Viewport(this);
    workspaces_ = new QTabWidget(this);
    workspaces_->setMinimumWidth(540);  // all six tabs without scroll arrows
    workspaces_->setDocumentMode(true);
    const QString queued = tr("Runs in the background as a job (see Jobs); the result appears in the Results panel.");
    workspaces_->addTab(build_rig_page(), tr("Rig"));
    workspaces_->addTab(build_animate_page(), tr("Animate"));
    workspaces_->addTab(build_analysis_page(tr("Simulate"), queued,
                                            {{tr("Torque chain"), {"torque", "@project"},
                                              tr("Torque at each mobile from the barrel to the escape wheel.")},
                                             {tr("Timekeeping"), {"timekeeping", "@project"},
                                              tr("Amplitude, rate, isochronism, positions, power reserve and energy budget; "
                                                 "plots rate against amplitude.")},
                                             {tr("Timegrapher (60 s)"), {"timegrapher", "@project"},
                                              tr("A virtual timegrapher: rate, amplitude, beat error and the trace.")},
                                             {tr("Check the models agree"), {"verify", "levels", "@project"},
                                              tr("Compares the detailed contact model with the reduced event-driven one.")}}),
                        tr("Simulate"));
    workspaces_->addTab(build_analysis_page(tr("Analyse"), queued,
                                            {{tr("Tooth bending of the selected part"), {"fea", "@project", "--part", "@part"},
                                              tr("Finite-element check of the teeth under the train's torque.")},
                                             {tr("Shock and modes of the selected part"),
                                              {"fea", "@project", "--part", "@part", "--case", "shock"},
                                              tr("Stress from a drop and the natural frequencies; the stress is drawn on the part.")},
                                             {tr("Materials"), {"materials"}, tr("The material catalogue the analyses use.")}}),
                        tr("Analyse"));
    workspaces_->addTab(build_analysis_page(tr("Manufacture"), queued,
                                            {{tr("Design for manufacture"), {"dfm", "@project"},
                                              tr("Checks every part against the rules of your manufacturing processes.")},
                                             {tr("Collisions in motion"), {"collisions", "@project"},
                                              tr("Sweeps the moving train for overlaps and close approaches.")},
                                             {tr("Tolerance stack-up"), {"tolerance", "@project"},
                                              tr("Monte Carlo stack-up: backlash, engagement, end and side shake.")}}),
                        tr("Manufacture"));
    workspaces_->addTab(build_analysis_page(tr("Report"), queued,
                                            {{tr("Write the report (HTML and PDF)"), {"report", "@project", "-o", "@report"},
                                              tr("Every analysis in one document, written next to the project as report.html "
                                                 "and report.pdf.")}}),
                        tr("Report"));

    // The model is the focus: the viewport takes the spare width, the workspace keeps a steady column.
    auto* split = new QSplitter(Qt::Horizontal, this);
    split->addWidget(viewport_);
    split->addWidget(workspaces_);
    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 0);
    split->setSizes({600, 540});
    split->setChildrenCollapsible(false);
    setCentralWidget(split);

    const QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);

    // Model tree: mobiles with their parts; ticking a part shows or hides it.
    tree_ = new QTreeWidget(this);
    tree_->setHeaderLabels({tr("Part"), tr("Role")});
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tree_->setUniformRowHeights(true);
    tree_->setToolTip(tr("Tick a part to show or hide it. Right-click to show or hide every part of a role."));
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
    log_->setFont(mono);
    log_->setMaximumBlockCount(20000);  // a long session's log would otherwise grow without end
    log_->setPlaceholderText(tr("Every command the window runs is listed here, as you could type it at the command line."));
    auto* log_dock = new QDockWidget(tr("Log"), this);
    log_dock->setObjectName("log_dock");
    log_dock->setWidget(log_);
    addDockWidget(Qt::BottomDockWidgetArea, log_dock);

    jobs_ = new QListWidget(this);
    jobs_->setContextMenuPolicy(Qt::CustomContextMenu);
    jobs_->setToolTip(tr("Analyses run here one at a time. Select one to see its result again; right-click or "
                         "double-click a queued or running job to cancel it."));
    auto* jobs_dock = new QDockWidget(tr("Jobs"), this);
    jobs_dock->setObjectName("jobs_dock");
    jobs_dock->setWidget(jobs_);
    addDockWidget(Qt::BottomDockWidgetArea, jobs_dock);
    queue_ = new JobQueue(worker_program(), this);
    connect(queue_, &JobQueue::changed, this, &MainWindow::on_job_changed);
    connect(queue_, &JobQueue::finished, this, &MainWindow::on_job_finished);
    connect(jobs_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { cancel_job(jobs_->currentRow()); });
    connect(jobs_, &QListWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        const int row = jobs_->indexAt(pos).row();
        if (row < 0 || row >= static_cast<int>(queue_->jobs().size())) return;
        const auto state = queue_->jobs()[static_cast<std::size_t>(row)].state;
        QMenu menu(this);
        QAction* cancel = menu.addAction(tr("Cancel this job"));
        cancel->setEnabled(state == JobQueue::State::Queued || state == JobQueue::State::Running);
        if (menu.exec(jobs_->viewport()->mapToGlobal(pos)) == cancel) cancel_job(row);
    });
    connect(jobs_, &QListWidget::currentRowChanged, this, [this](int row) {
        // Selecting a job shows its output again, which also works for jobs finished before a restart.
        if (row < 0 || row >= static_cast<int>(queue_->jobs().size())) return;
        const auto& job = queue_->jobs()[static_cast<std::size_t>(row)];
        if (job.state == JobQueue::State::Done || job.state == JobQueue::State::Failed) show_result(job.args, job.out, job.exit_code);
    });

    undo_ = new QUndoStack(this);
    auto* history_dock = new QDockWidget(tr("Edit history"), this);
    history_dock->setObjectName("history_dock");
    auto* history = new QUndoView(undo_, this);  // click an entry to go back to that state
    history->setEmptyLabel(tr("Project as opened"));
    history_dock->setWidget(history);
    addDockWidget(Qt::BottomDockWidgetArea, history_dock);

    // The four output panels share one tabbed strip under the model instead of four columns.
    tabifyDockWidget(results_dock_, log_dock);
    tabifyDockWidget(log_dock, jobs_dock);
    tabifyDockWidget(jobs_dock, history_dock);
    results_dock_->raise();
    resizeDocks({tree_dock, explain_dock}, {300, 300}, Qt::Horizontal);
    resizeDocks({tree_dock, explain_dock}, {320, 220}, Qt::Vertical);
    resizeDocks({results_dock_}, {220}, Qt::Vertical);

    // Status bar: what is open on the left; the selection and whether work is running on the right.
    status_ = new QLabel(tr("No project open"), this);
    statusBar()->addWidget(status_, 1);
    selection_label_ = new QLabel(this);
    statusBar()->addPermanentWidget(selection_label_);
    busy_ = new QProgressBar(this);
    busy_->setRange(0, 0);  // indeterminate
    busy_->setMaximumWidth(120);
    busy_->setMaximumHeight(14);
    busy_->setTextVisible(false);
    busy_->hide();
    statusBar()->addPermanentWidget(busy_);

    // Menus in the usual order, and a toolbar for what is used all the time.
    auto* file = menuBar()->addMenu(tr("&File"));
    QAction* open = file->addAction(tr("&Open project..."), QKeySequence::Open, this, &MainWindow::on_open);
    QAction* import = file->addAction(tr("&Import STEP..."), QKeySequence(tr("Ctrl+I")), this, &MainWindow::on_import);
    QAction* demo = file->addAction(tr("Open the &demonstration movement"), this, &MainWindow::on_demo);
    open->setToolTip(tr("Open a project (.hsproj) made by an earlier import"));
    import->setToolTip(tr("Import a STEP, IGES or DXF file: the parts are found and rigged automatically"));
    demo->setToolTip(tr("Build and open the procedural demonstration movement"));
    open->setIconText(tr("Open"));  // the toolbar's shorter labels
    import->setIconText(tr("Import STEP"));
    demo->setIconText(tr("Demo movement"));
    file->addSeparator();
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

    auto* edit = menuBar()->addMenu(tr("&Edit"));
    QAction* undo = undo_->createUndoAction(this, tr("&Undo"));
    QAction* redo = undo_->createRedoAction(this, tr("&Redo"));
    undo->setShortcut(QKeySequence::Undo);
    redo->setShortcut(QKeySequence::Redo);
    edit->addAction(undo);
    edit->addAction(redo);

    auto* view = menuBar()->addMenu(tr("&View"));
    struct Preset {
        QString name, tip;
        double yaw, pitch;
        const char* key;
    };
    const std::vector<Preset> presets{{tr("Top"), tr("Look down on the dial side"), 0, 0, "Ctrl+1"},
                                      {tr("Isometric"), tr("Three-quarter view"), 35, -55, "Ctrl+2"},
                                      {tr("Side"), tr("Edge-on"), 0, -90, "Ctrl+3"},
                                      {tr("Bottom"), tr("Look up at the back of the movement"), 0, -180, "Ctrl+4"}};
    std::vector<QAction*> view_actions;
    for (const Preset& p : presets) {
        QAction* a = view->addAction(p.name, this, [this, p] { viewport_->set_view(p.yaw, p.pitch); });
        a->setShortcut(QKeySequence(QString::fromLatin1(p.key)));
        a->setToolTip(tr("%1 (%2)").arg(p.tip, a->shortcut().toString(QKeySequence::NativeText)));
        view_actions.push_back(a);
    }
    QAction* fit = view->addAction(tr("Fit"), this, [this] { viewport_->fit(); });
    fit->setShortcut(QKeySequence(tr("Ctrl+0")));
    fit->setToolTip(tr("Reset zoom and pan (%1)").arg(fit->shortcut().toString(QKeySequence::NativeText)));
    view->addSeparator();
    auto* panels = view->addMenu(tr("&Panels"));
    for (QDockWidget* d : {tree_dock, explain_dock, results_dock_, log_dock, jobs_dock, history_dock}) panels->addAction(d->toggleViewAction());
    view->addAction(tr("&Reset layout"), this, [this] { restoreState(default_state_); });

    auto* help = menuBar()->addMenu(tr("&Help"));
    help->addAction(tr("&Viewport controls"), this, [this] {
        QMessageBox::information(this, tr("Viewport controls"),
                                 tr("<b>Left drag</b> orbits the movement.<br><b>Right or middle drag</b> pans.<br>"
                                    "<b>Wheel</b> zooms.<br><b>Click</b> a part to select it and explain it.<br>"
                                    "<b>Space</b> plays or pauses the animation.<br><b>Ctrl+1 to Ctrl+4</b> pick a view, "
                                    "<b>Ctrl+0</b> fits."));
    });
    help->addAction(tr("&About Open Horology Studio"), this, [this] {
        QMessageBox::about(this, tr("About Open Horology Studio"),
                           tr("<b>Open Horology Studio</b><br>Import, rig, animate and analyse mechanical watch "
                              "movements. Every action is an <tt>ohs</tt> command, kept as a replayable script."));
    });

    auto* tools = addToolBar(tr("Main"));
    tools->setObjectName("main_toolbar");
    tools->setMovable(false);
    tools->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    tools->addAction(open);
    tools->addAction(import);
    tools->addAction(demo);
    tools->addSeparator();
    for (QAction* a : view_actions) tools->addAction(a);
    tools->addAction(fit);
    tools->addSeparator();
    play_action_ = tools->addAction(media_icon(false, palette().buttonText().color()), tr("Play"));
    play_action_->setCheckable(true);
    play_action_->setToolTip(tr("Play or pause the animation (Space in the viewport)"));
    connect(play_action_, &QAction::toggled, this, [this](bool on) { set_playing(on); });
    auto* space = new QAction(viewport_);
    space->setShortcut(Qt::Key_Space);
    space->setShortcutContext(Qt::WidgetShortcut);
    connect(space, &QAction::triggered, this, [this] { set_playing(!timer_->isActive()); });
    viewport_->addAction(space);

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &MainWindow::on_timer);
    connect(viewport_, &Viewport::part_picked, this, &MainWindow::select_part);
    connect(tree_, &QTreeWidget::itemChanged, this, &MainWindow::on_tree_changed);
    connect(tree_, &QTreeWidget::itemSelectionChanged, this, &MainWindow::on_tree_selected);
    // Right click in the model tree or the rig list: show or hide all the parts of one role.
    for (QAbstractItemView* v : {static_cast<QAbstractItemView*>(tree_), static_cast<QAbstractItemView*>(rig_table_)}) {
        v->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(v, &QWidget::customContextMenuRequested, this, [this, v](const QPoint& pos) {
            if (!scene_) return;
            QMenu* menu = visibility_menu(v);
            menu->exec(v->viewport()->mapToGlobal(pos));
            menu->deleteLater();
        });
    }
    set_playing(false);
    update_busy();
    update_selection_ui();
}

void MainWindow::restore_layout() {
    default_state_ = saveState();
    QSettings settings;
    restoreGeometry(settings.value("window/geometry").toByteArray());
    restoreState(settings.value("window/state").toByteArray());
}

void MainWindow::closeEvent(QCloseEvent* event) {
    QSettings settings;
    settings.setValue("window/geometry", saveGeometry());
    settings.setValue("window/state", saveState());
    QMainWindow::closeEvent(event);
}

void MainWindow::update_busy() {
    const bool busy = importing_ || unfinished_jobs() > 0;
    busy_->setVisible(busy);
    busy_->setToolTip(importing_ ? tr("Opening the project") : tr("%n job(s) queued or running", nullptr, unfinished_jobs()));
}

QWidget* MainWindow::build_rig_page() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(make_hint(new QLabel(tr("The parts found in the imported file, least certain first. Check the top "
                                              "rows: select a part, then correct its role or say which part it meshes "
                                              "with. Every change can be undone."))));

    rig_table_ = new QTableWidget(0, 4, page);
    rig_table_->setHorizontalHeaderLabels({tr("Part"), tr("Role"), tr("Confidence"), tr("Decided by")});
    rig_table_->verticalHeader()->hide();
    rig_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int c = 1; c < 4; ++c) rig_table_->horizontalHeader()->setSectionResizeMode(c, QHeaderView::ResizeToContents);
    rig_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    rig_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    rig_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    rig_table_->setAlternatingRowColors(true);
    rig_table_->setShowGrid(false);
    layout->addWidget(rig_table_, 1);

    // The editor works on the selected part, so it says which part that is and is off until there is one.
    selected_box_ = new QGroupBox(tr("Selected part"));
    auto* form = new QGridLayout(selected_box_);
    role_box_ = new QComboBox;
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::kLastRole); ++r)
        role_box_->addItem(QString::fromUtf8(model::to_string(static_cast<model::Role>(r)).data()));
    apply_role_ = new QPushButton(tr("Set role"));
    apply_role_->setToolTip(tr("Give the selected part this role and re-rig the movement (undoable)"));
    mesh_box_ = new QComboBox;
    declare_mesh_ = new QPushButton(tr("Declare mesh"));
    declare_mesh_->setToolTip(tr("Say that the selected part's teeth mesh with this part's (undoable)"));
    form->addWidget(new QLabel(tr("Role")), 0, 0);
    form->addWidget(role_box_, 0, 1);
    form->addWidget(apply_role_, 0, 2);
    form->addWidget(new QLabel(tr("Meshes with")), 1, 0);
    form->addWidget(mesh_box_, 1, 1);
    form->addWidget(declare_mesh_, 1, 2);
    form->setColumnStretch(1, 1);
    layout->addWidget(selected_box_);

    connect(declare_mesh_, &QPushButton::clicked, this, &MainWindow::on_declare_mesh);
    connect(apply_role_, &QPushButton::clicked, this, &MainWindow::on_apply_role);
    connect(rig_table_, &QTableWidget::itemSelectionChanged, this, [this] {
        const auto rows = rig_table_->selectionModel()->selectedRows();
        if (!rows.isEmpty() && !rebuilding_) select_part(rig_table_->item(rows.first().row(), 0)->text());
    });
    return page;
}

QWidget* MainWindow::build_animate_page() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);

    auto* playback = new QGroupBox(tr("Playback"));
    auto* pl = new QGridLayout(playback);
    play_ = new QPushButton(tr("Play"));
    play_->setCheckable(true);
    play_->setToolTip(tr("Play or pause the animation (Space in the viewport)"));
    speed_ = new QDoubleSpinBox;
    speed_->setRange(0.01, 100000);
    speed_->setValue(1.0);
    speed_->setSuffix(tr(" x"));
    speed_->setToolTip(tr("Movement seconds per real second"));
    stepped_ = new QCheckBox(tr("One escapement step per beat"));
    stepped_->setToolTip(tr("Advance the train in the escapement's steps instead of smoothly"));
    timeline_ = new QSlider(Qt::Horizontal);
    timeline_->setRange(0, 10000);  // 0..60 s of movement time
    timeline_->setToolTip(tr("Movement time, 0 to 60 s"));
    time_label_ = new QLabel;
    time_label_->setMinimumWidth(QFontMetrics(time_label_->font()).horizontalAdvance("00.0 s of 60 s") + 4);
    time_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    pl->addWidget(play_, 0, 0);
    pl->addWidget(new QLabel(tr("Speed")), 0, 1, Qt::AlignRight);
    pl->addWidget(speed_, 0, 2);
    pl->addWidget(timeline_, 1, 0, 1, 2);
    pl->addWidget(time_label_, 1, 2);
    pl->addWidget(stepped_, 2, 0, 1, 3);
    pl->setColumnStretch(1, 1);
    layout->addWidget(playback);

    auto* views = new QGroupBox(tr("View"));
    auto* vl = new QGridLayout(views);
    explode_ = new QSlider(Qt::Horizontal);
    explode_->setRange(0, 100);
    explode_->setToolTip(tr("Lift the parts apart along the dial axis"));
    explode_label_ = new QLabel;
    section_ = new QCheckBox(tr("Cut-away"));
    section_->setToolTip(tr("Remove everything above a plane across the dial axis"));
    section_offset_ = new QSlider(Qt::Horizontal);
    section_offset_->setRange(0, 1000);
    section_offset_->setValue(500);
    section_offset_->setEnabled(false);
    section_offset_->setToolTip(tr("Height of the cut-away plane"));
    vl->addWidget(new QLabel(tr("Exploded")), 0, 0);
    vl->addWidget(explode_, 0, 1);
    vl->addWidget(explode_label_, 0, 2);
    vl->addWidget(section_, 1, 0);
    vl->addWidget(section_offset_, 1, 1, 1, 2);
    vl->setColumnStretch(1, 1);
    layout->addWidget(views);

    auto* overlays = new QGroupBox(tr("Overlays"));
    auto* ol = new QGridLayout(overlays);
    forces_check_ = new QCheckBox(tr("Tooth force arrows"));
    forces_check_->setToolTip(tr("The tangential force at each gear mesh from the barrel to the escape wheel, at full wind"));
    trails_check_ = new QCheckBox(tr("Motion trails over"));
    trails_check_->setToolTip(tr("Where a point on each wheel's rim has been"));
    trail_seconds_ = new QDoubleSpinBox;
    trail_seconds_->setRange(0.1, 3600);
    trail_seconds_->setValue(10.0);
    trail_seconds_->setSuffix(tr(" s"));
    trail_seconds_->setEnabled(false);
    ol->addWidget(forces_check_, 0, 0, 1, 2);
    ol->addWidget(trails_check_, 1, 0);
    ol->addWidget(trail_seconds_, 1, 1);
    ol->setColumnStretch(2, 1);
    layout->addWidget(overlays);
    layout->addWidget(make_hint(new QLabel(tr("The overlays follow the assembled movement; they are hidden while it is exploded."))));
    layout->addStretch();

    connect(play_, &QPushButton::toggled, this, [this](bool on) { set_playing(on); });
    connect(timeline_, &QSlider::valueChanged, this, &MainWindow::on_timeline);
    for (auto* s : {explode_, section_offset_}) connect(s, &QSlider::valueChanged, this, &MainWindow::on_view_changed);
    connect(section_, &QCheckBox::toggled, section_offset_, &QWidget::setEnabled);
    connect(trails_check_, &QCheckBox::toggled, trail_seconds_, &QWidget::setEnabled);
    connect(section_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(stepped_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(forces_check_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(trails_check_, &QCheckBox::toggled, this, &MainWindow::on_view_changed);
    connect(trail_seconds_, &QDoubleSpinBox::valueChanged, this, &MainWindow::on_view_changed);
    update_animate_labels();
    return page;
}

void MainWindow::update_animate_labels() {
    time_label_->setText(tr("%1 s of 60 s").arg(time_, 0, 'f', 1));
    explode_label_->setText(tr("%1 %").arg(explode_->value()));
}

QWidget* MainWindow::build_analysis_page(const QString& title, const QString& intro, const std::vector<Analysis>& analyses) {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(make_hint(new QLabel(intro)));
    if (title == tr("Analyse")) {
        analyse_part_label_ = new QLabel;
        analyse_part_label_->setWordWrap(true);
        layout->addWidget(analyse_part_label_);
    }
    // Each analysis is a button with a line under it saying what it works out.
    for (const Analysis& a : analyses) {
        auto* b = new QPushButton(a.label);
        b->setProperty("ohs_args", a.args);
        b->setToolTip(a.description);
        b->setMinimumHeight(30);
        connect(b, &QPushButton::clicked, this, &MainWindow::on_run_analysis);
        layout->addSpacing(4);
        layout->addWidget(b);
        auto* what = make_hint(new QLabel(a.description));
        what->setContentsMargins(4, 0, 4, 0);
        layout->addWidget(what);
    }
    if (title == tr("Analyse")) {
        auto* plot_box = new QGroupBox(tr("Result on the part"));
        auto* row = new QHBoxLayout(plot_box);
        field_box_ = new QComboBox;
        field_box_->addItem(tr("von Mises stress"), "von_mises");
        field_box_->addItem(tr("Displacement"), "displacement");
        field_box_->setToolTip(tr("Which field of the last shock analysis colours the part"));
        auto* clear = new QPushButton(tr("Clear"));
        clear->setToolTip(tr("Draw the part normally again"));
        row->addWidget(field_box_, 1);
        row->addWidget(clear);
        layout->addSpacing(8);
        layout->addWidget(plot_box);
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
        results_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));  // the analyses print tables
        plot_->hide();  // until an analysis has something to plot
        results_dock_ = new QDockWidget(tr("Results"), this);
        results_dock_->setObjectName("results_dock");
        results_dock_->setWidget(holder);
        addDockWidget(Qt::BottomDockWidgetArea, results_dock_);
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
    setWindowTitle(tr("%1 — Open Horology Studio").arg(QFileInfo(project).fileName()));
    refresh_panels();
    update_selection_ui();
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
    update_busy();
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
        update_busy();
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
        auto* confidence = new QTableWidgetItem(QString::number(c.role_confidence, 'f', 2));
        confidence->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        // The ones to check stand out: red below one half, amber below 0.8.
        if (c.role_confidence < 0.5) {
            confidence->setForeground(QColor(225, 95, 80));
            confidence->setToolTip(tr("Low confidence: check this role"));
        } else if (c.role_confidence < 0.8) {
            confidence->setForeground(QColor(220, 165, 60));
            confidence->setToolTip(tr("Medium confidence: worth a look"));
        }
        rig_table_->setItem(static_cast<int>(row), 2, confidence);
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
    play_action_->setEnabled(scene_->can_animate());
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

void MainWindow::update_selection_ui() {
    const QString part = QString::fromStdString(viewport_->selected());
    const int index = scene_ && !part.isEmpty() ? scene_->movement().find_component(part.toStdString()) : -1;
    const bool has = index >= 0;
    selection_label_->setText(has ? tr("Selected: %1").arg(part) : QString());
    selected_box_->setTitle(has ? tr("Selected part: %1").arg(part) : tr("Selected part (click one in the list or the viewport)"));
    for (QWidget* w : {static_cast<QWidget*>(role_box_), static_cast<QWidget*>(apply_role_), static_cast<QWidget*>(mesh_box_),
                       static_cast<QWidget*>(declare_mesh_)})
        w->setEnabled(has);
    if (has) {
        // The role box starts at the part's current role, so Set role changes only what is changed.
        const auto& c = scene_->movement().components[static_cast<std::size_t>(index)];
        role_box_->setCurrentText(QString::fromUtf8(model::to_string(c.role).data()));
    }
    analyse_part_label_->setText(has ? tr("Selected part: <b>%1</b>").arg(part.toHtmlEscaped())
                                     : tr("<i>No part selected. The part analyses work on the part selected in the "
                                          "viewport, the model tree or the rig list.</i>"));
}

void MainWindow::select_part(const QString& name) {
    if (!scene_) return;
    viewport_->select(name.toStdString());
    update_selection_ui();
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
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::kLastRole); ++r) {
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
    update_animate_labels();
    if (scene_) scene_->set_time(seconds);
    viewport_->interactive_update();
}

void MainWindow::set_playing(bool playing) {
    if (playing && scene_ && scene_->can_animate()) timer_->start();
    else timer_->stop();
    playing = timer_->isActive();  // nothing to play: both buttons stay up
    // The Animate page's button and the toolbar's are the same control.
    const QIcon icon = media_icon(playing, palette().buttonText().color());
    {
        const QSignalBlocker block(play_);
        play_->setChecked(playing);
        play_->setIcon(icon);
        play_->setText(playing ? tr("Pause") : tr("Play"));
    }
    if (play_action_) {
        const QSignalBlocker block(play_action_);
        play_action_->setChecked(playing);
        play_action_->setIcon(icon);
        play_action_->setText(playing ? tr("Pause") : tr("Play"));
    }
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
    update_animate_labels();
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
    QListWidgetItem* item = jobs_->item(index);
    item->setText(text);
    // The state at a glance: the accent while running, red when failed, muted when finished or cancelled.
    QColor ink = palette().text().color();
    if (job.state == JobQueue::State::Running) ink = palette().highlight().color();
    else if (job.state == JobQueue::State::Failed) ink = QColor(220, 90, 80);
    else if (job.state == JobQueue::State::Done || job.state == JobQueue::State::Cancelled) ink = palette().placeholderText().color();
    item->setForeground(ink);
    QFont f = jobs_->font();
    f.setBold(job.state == JobQueue::State::Running);
    item->setFont(f);
    update_busy();
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
            plot_->show();
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
