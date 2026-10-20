// SPDX-License-Identifier: Apache-2.0
#include "gui/main_window.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QListWidget>
#include <QThread>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
#include <QPointer>
#include <QSaveFile>
#include <QProgressBar>
#include <QActionGroup>
#include <QSettings>
#include <QStandardPaths>
#include <QToolButton>
#include <QStyle>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTabWidget>
#include <QTextStream>
#include <QIntValidator>
#include <QLineEdit>
#include <QShortcut>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <nlohmann/json.hpp>

#include "cli/cli_run.hpp"
#include "cli/daemon_protocol.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"
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
    tree_->setEditTriggers(QAbstractItemView::EditKeyPressed);
    tree_->setToolTip(tr("Tick a part to show or hide it. Press F2 to label it; hover for its part name. Right-click to show or hide every part of a role."));
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
    queue_->set_max_parallel(2);  // a quick analysis need not wait behind a long one
    // A resident `ohs serve` that answers takes the jobs: no process start-up, warm geometry, output as it runs.
    // OHS_JOB_DAEMON=0 keeps one process per job.
    if (qEnvironmentVariable("OHS_JOB_DAEMON") != "0") {
        const std::string socket = cli::daemon::default_socket_path();
        const auto ping = cli::daemon::run_remote({"--version"}, socket, "");
        if (ping && !ping->crashed) queue_->set_daemon(QString::fromStdString(socket));
    }
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
    cancel_load_ = new QToolButton(this);
    cancel_load_->setText(tr("Cancel"));
    cancel_load_->setToolTip(tr("Stop opening or importing the project"));
    cancel_load_->hide();
    connect(cancel_load_, &QToolButton::clicked, this, [this] {
        import_cancel_ = true;
        cancel_load_->setEnabled(false);
    });
    statusBar()->addPermanentWidget(cancel_load_);

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
    edit->addSeparator();
    edit->addAction(tr("Re-run auto-&rig"), this, [this] { rerig(false); })
        ->setToolTip(tr("Classify and rig the parts again, keeping the roles you set and the meshes you declared"));
    edit->addAction(tr("Re-rig from &scratch..."), this, [this] {
        if (!scene_) return;
        if (QMessageBox::question(this, tr("Re-rig from scratch"),
                                  tr("Decide every role and mesh afresh, forgetting the roles you set and the meshes you "
                                     "declared? This can be undone.")) == QMessageBox::Yes)
            rerig(true);
    });

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
    rig_overlay_action_ = view->addAction(tr("Show the &rig"));
    rig_overlay_action_->setCheckable(true);
    rig_overlay_action_->setToolTip(tr("Colour the mobiles and draw the gear meshes, suggested meshes and parts to check"));
    connect(rig_overlay_action_, &QAction::toggled, this, [this](bool on) { rig_overlay_->setChecked(on); });
    view->addSeparator();
    // How the viewport draws: the graphics card when there is one (Auto), or the painter, which runs anywhere.
    auto* renderer = view->addMenu(tr("Rend&erer"));
    auto* renderers = new QActionGroup(this);
    struct Choice {
        Renderer value;
        QString name, tip;
    };
    const Choice choices[] = {
        {Renderer::Auto, tr("&Automatic"), tr("Use OpenGL when a graphics card is there to draw with, the painter otherwise")},
        {Renderer::OpenGL, tr("&OpenGL"), tr("Draw on the graphics card: smoother with a large movement")},
        {Renderer::Software, tr("&Software"), tr("Draw with Qt's painter on the processor: works on every machine")}};
    for (const Choice& c : choices) {
        QAction* a = renderer->addAction(c.name);
        a->setCheckable(true);
        a->setActionGroup(renderers);
        a->setChecked(viewport_->renderer() == c.value);
        a->setToolTip(c.tip);
        a->setStatusTip(c.tip);
        connect(a, &QAction::triggered, this, [this, c] {
            Viewport::save_renderer(c.value);
            viewport_->set_renderer(c.value);
            statusBar()->showMessage(viewport_->using_opengl() ? tr("Drawing with OpenGL (%1)").arg(Viewport::gpu().name)
                                                               : tr("Drawing with the painter"), 5000);
        });
    }
    view->addSeparator();
    auto* panels = view->addMenu(tr("&Panels"));
    for (QDockWidget* d : {tree_dock, explain_dock, results_dock_, log_dock, jobs_dock, history_dock}) panels->addAction(d->toggleViewAction());
    view->addAction(tr("&Reset layout"), this, [this] { restoreState(default_state_); });

    auto* help = menuBar()->addMenu(tr("&Help"));
    help->addAction(tr("&Viewport controls"), this, [this] {
        QMessageBox::information(this, tr("Viewport controls"),
                                 tr("<b>Drag on a part</b> turns the movement about its central axis (up and down "
                                    "tilts it).<br><b>Drag beside the movement</b>, or right or middle drag anywhere, "
                                    "pans.<br><b>Wheel</b> zooms; with a part selected it zooms toward that part.<br>"
                                    "<b>Click</b> a part to select it and explain it; click beside the movement to "
                                    "clear the selection.<br>"
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
    connect(viewport_, &Viewport::part_picked, this, &MainWindow::on_part_picked);
    connect(workspaces_, &QTabWidget::currentChanged, this, [this](int) { update_walk_view(); });
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
    cancel_load_->setVisible(importing_);
    busy_->setToolTip(importing_ ? tr("Opening the project") : tr("%n job(s) queued or running", nullptr, unfinished_jobs()));
}

QWidget* MainWindow::build_rig_page() {
    auto* page = new QWidget;
    auto* layout = new QVBoxLayout(page);
    layout->addWidget(make_hint(new QLabel(tr("Roles, mobiles and gear meshes are found automatically. Walk the train from "
                                              "the barrel, one mesh at a time, to check each link and fill any gap. "
                                              "Every change can be undone."))));

    // The condition for the animation and the simulations is a train joined up from the barrel to the escape wheel:
    // the walk goes through its meshes in order and stops at the first one that is missing.
    auto* train_box = new QGroupBox(tr("Walk the train"));
    auto* train_layout = new QVBoxLayout(train_box);
    chain_label_ = new QLabel(tr("No movement open"));
    chain_label_->setWordWrap(true);
    chain_label_->setTextFormat(Qt::RichText);
    train_layout->addWidget(chain_label_);

    // One chip for each step; the current one is pressed in, and clicking another goes to it.
    auto* strip = chips_scroll_ = new QScrollArea;
    strip->setFrameShape(QFrame::NoFrame);
    strip->setWidgetResizable(true);
    strip->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    strip->setFixedHeight(54);
    chips_host_ = new QWidget;
    chips_layout_ = new QHBoxLayout(chips_host_);
    chips_layout_->setContentsMargins(0, 0, 0, 0);
    strip->setWidget(chips_host_);
    train_layout->addWidget(strip);

    step_title_ = new QLabel;
    QFont heading = step_title_->font();
    heading.setBold(true);
    step_title_->setFont(heading);
    step_text_ = new QLabel;
    step_text_->setWordWrap(true);
    step_text_->setTextFormat(Qt::RichText);
    step_text_->setMinimumHeight(3 * step_text_->fontMetrics().lineSpacing());
    step_text_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    train_layout->addWidget(step_title_);
    train_layout->addWidget(step_text_);

    // What can be done where the chain stops, or when the barrel or the escape wheel is missing.
    gap_box_ = new QWidget;
    auto* gap_layout = new QVBoxLayout(gap_box_);
    gap_layout->setContentsMargins(0, 0, 0, 0);
    candidates_ = new QListWidget;
    candidates_->setMaximumHeight(84);
    candidates_->setToolTip(tr("Pairs that probably mesh and would join the chain. Click one to see it in the viewport"));
    gap_layout->addWidget(candidates_);
    auto* gap_row = new QHBoxLayout;
    declare_candidate_ = new QPushButton(tr("Declare the mesh"));
    declare_candidate_->setToolTip(tr("Say that the selected pair meshes (undoable)"));
    pick_partner_ = new QPushButton;
    change_role_ = new QPushButton(tr("Change its role"));
    change_role_->setToolTip(tr("Select the part so that its role can be corrected below"));
    gap_row->addWidget(declare_candidate_);
    gap_row->addWidget(pick_partner_);
    gap_row->addWidget(change_role_);
    gap_layout->addLayout(gap_row);
    train_layout->addWidget(gap_box_);

    auto* beat_row = new QHBoxLayout;
    beat_row->addWidget(new QLabel(tr("Beat rate")));
    beat_box_ = new QComboBox;
    beat_box_->setEditable(true);
    beat_box_->setInsertPolicy(QComboBox::NoInsert);
    beat_box_->setValidator(new QIntValidator(1, 200000, beat_box_));
    for (const int vph : {14400, 18000, 19800, 21600, 25200, 28800, 36000}) {
        beat_box_->addItem(QString::number(vph), vph);
    }
    beat_box_->lineEdit()->setPlaceholderText(tr("not set: type vibrations per hour"));
    beat_box_->setToolTip(tr("Vibrations per hour of the balance (28800 is 4 Hz, 21600 is 3 Hz). The animation, the simulations and "
                             "Solve the train need it. Pick one or type a number and press Enter (undoable)"));
    beat_row->addWidget(beat_box_, 1);
    beat_row->addWidget(new QLabel(tr("vph")));
    train_layout->addLayout(beat_row);

    auto* step_row = new QHBoxLayout;
    prev_step_ = new QPushButton(tr("◀ Previous"));
    next_step_ = new QPushButton(tr("Next ▶"));
    prev_step_->setShortcut(QKeySequence(tr("Alt+Left")));
    next_step_->setShortcut(QKeySequence(tr("Alt+Right")));
    prev_step_->setToolTip(tr("The previous mesh (Alt+Left)"));
    next_step_->setToolTip(tr("The next mesh along the train (Alt+Right)"));
    step_row->addWidget(prev_step_);
    step_row->addWidget(next_step_);
    leave_out_ = new QPushButton(tr("Not in the train"));
    leave_out_->setToolTip(tr("This mesh is the winding work, or not a link from the barrel to the escape wheel: leave it "
                              "out so that the walk goes another way (undoable)"));
    step_row->addWidget(leave_out_);
    step_row->addStretch(1);
    rig_overlay_ = new QCheckBox(tr("Show the whole rig instead"));
    rig_overlay_->setToolTip(tr("Colour each mobile and draw every gear mesh (green where the barrel feeds it), the "
                                "suggested meshes and a ring round the parts to check, instead of one step"));
    rerig_button_ = new QPushButton(tr("Re-run auto-rig"));
    rerig_button_->setToolTip(tr("Classify and rig the parts again: newly recognised parts join their mobiles and "
                                 "meshes. The roles you set and the meshes you declared are kept (undoable)"));
    solve_button_ = new QPushButton(tr("Solve the train"));
    solve_button_->setToolTip(tr("Work back from the escape wheel with the beat rate to find what drives the loose end of the "
                                 "train (a barrel whose teeth were not found, say) and which wheels turn the hands. It asks "
                                 "before declaring anything (undoable)"));
    step_row->addWidget(solve_button_);
    step_row->addWidget(rerig_button_);
    train_layout->addLayout(step_row);
    train_layout->addWidget(rig_overlay_);
    layout->addWidget(train_box);
    auto* escape = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(escape, &QShortcut::activated, this, [this] { cancel_pick(); });
    connect(rig_overlay_, &QCheckBox::toggled, this, [this](bool on) { set_show_rig(on); });
    connect(rerig_button_, &QPushButton::clicked, this, [this] { rerig(false); });
    connect(solve_button_, &QPushButton::clicked, this, [this] { solve_train(true); });
    const auto beat_entered = [this] {
        if (!scene_ || rebuilding_) return;
        const double vph = beat_box_->lineEdit()->text().trimmed().toDouble();
        if (vph > 0) set_beat_rate(vph);
    };
    connect(beat_box_, QOverload<int>::of(&QComboBox::activated), this, beat_entered);
    connect(beat_box_->lineEdit(), &QLineEdit::editingFinished, this, beat_entered);
    connect(leave_out_, &QPushButton::clicked, this, [this] {
        if (!scene_) return;
        const auto& walk = scene_->review().walk;
        if (walk.steps.empty()) return;
        const auto& step = walk.steps[static_cast<std::size_t>(std::clamp(walk_pos_, 0, static_cast<int>(walk.steps.size()) - 1))];
        if (step.kind != rigging::WalkStep::Kind::Mesh) return;
        const auto& m = scene_->movement();
        set_winding(QString::fromStdString(m.components[static_cast<std::size_t>(step.driver)].features.name),
                    QString::fromStdString(m.components[static_cast<std::size_t>(step.driven)].features.name), true);
    });
    connect(prev_step_, &QPushButton::clicked, this, [this] { go_to_step(walk_pos_ - 1); });
    connect(next_step_, &QPushButton::clicked, this, [this] { go_to_step(walk_pos_ + 1); });
    connect(candidates_, &QListWidget::currentRowChanged, this, [this](int) { update_walk_view(); });
    connect(declare_candidate_, &QPushButton::clicked, this, [this] {
        const QListWidgetItem* item = candidates_->currentItem();
        if (!item) {
            statusBar()->showMessage(tr("Select a pair first"), 4000);
            return;
        }
        const QStringList pair = item->data(Qt::UserRole).toStringList();
        declare_mesh(pair.value(0), pair.value(1));
    });
    connect(pick_partner_, &QPushButton::clicked, this, [this] {
        if (!scene_) return;
        if (picking_) {
            cancel_pick();
            return;
        }
        const auto& walk = scene_->review().walk;
        if (walk.steps.empty()) return;
        const auto& step = walk.steps[static_cast<std::size_t>(std::clamp(walk_pos_, 0, static_cast<int>(walk.steps.size()) - 1))];
        if (step.kind == rigging::WalkStep::Kind::Missing) {
            pick_part_for_role(QString::fromUtf8(model::to_string(step.needs_role).data()));
        } else if (step.driver >= 0) {
            pick_partner_for(QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(step.driver)].features.name));
        }
    });
    connect(change_role_, &QPushButton::clicked, this, [this] {
        if (!scene_) return;
        const auto& walk = scene_->review().walk;
        if (walk.steps.empty()) return;
        const auto& step = walk.steps[static_cast<std::size_t>(std::clamp(walk_pos_, 0, static_cast<int>(walk.steps.size()) - 1))];
        if (step.driver < 0) return;
        select_part(QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(step.driver)].features.name));
        role_box_->setFocus();
    });

    rig_tabs_ = new QTabWidget;
    rig_tabs_->setDocumentMode(true);
    rig_table_ = new QTableWidget(0, 4, page);
    rig_table_->setHorizontalHeaderLabels({tr("Part"), tr("Role"), tr("Confidence"), tr("Decided by")});
    rig_table_->verticalHeader()->hide();
    rig_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int c = 1; c < 4; ++c) rig_table_->horizontalHeader()->setSectionResizeMode(c, QHeaderView::ResizeToContents);
    rig_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    rig_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    rig_table_->setSelectionMode(QAbstractItemView::ExtendedSelection);  // several parts can be given a role at once
    rig_table_->setAlternatingRowColors(true);
    rig_table_->setShowGrid(false);
    auto* parts_page = new QWidget;
    auto* parts_layout = new QVBoxLayout(parts_page);
    parts_layout->setContentsMargins(0, 4, 0, 0);
    parts_filter_ = new QLineEdit;
    parts_filter_->setPlaceholderText(tr("Search parts by name, label or role"));
    parts_filter_->setClearButtonEnabled(true);
    parts_layout->addWidget(parts_filter_);
    parts_layout->addWidget(rig_table_, 1);
    connect(parts_filter_, &QLineEdit::textChanged, this, [this] { apply_parts_filter(); });
    rig_tabs_->addTab(parts_page, tr("Parts"));

    issue_list_ = new QListWidget;
    issue_list_->setWordWrap(true);
    issue_list_->setToolTip(tr("Click an entry to select the part it is about"));
    rig_tabs_->addTab(issue_list_, tr("To do"));

    auto* suggestions_page = new QWidget;
    auto* suggestions_layout = new QVBoxLayout(suggestions_page);
    suggestions_layout->setContentsMargins(0, 4, 0, 0);
    suggestion_list_ = new QListWidget;
    suggestion_list_->setToolTip(tr("Toothed parts that probably mesh but did not pass the strict test. Click one to "
                                    "select the pair; the number is how likely it is (at most 0.5)"));
    suggestions_layout->addWidget(suggestion_list_, 1);
    auto* suggestion_row = new QHBoxLayout;
    accept_suggestion_ = new QPushButton(tr("Declare selected"));
    accept_suggestion_->setToolTip(tr("Say that the selected pair meshes (undoable)"));
    accept_all_ = new QPushButton(tr("Declare all"));
    accept_all_->setToolTip(tr("Say that every suggested pair meshes, as one undoable edit"));
    suggestion_row->addStretch(1);
    suggestion_row->addWidget(accept_suggestion_);
    suggestion_row->addWidget(accept_all_);
    suggestions_layout->addLayout(suggestion_row);
    rig_tabs_->addTab(suggestions_page, tr("Suggested meshes"));
    layout->addWidget(rig_tabs_, 1);

    connect(issue_list_, &QListWidget::currentRowChanged, this, [this](int row) {
        if (row < 0 || rebuilding_) return;
        const QStringList parts = issue_list_->item(row)->data(Qt::UserRole).toStringList();
        if (parts.isEmpty()) return;
        select_part(parts.first());
        if (parts.size() > 1 && mesh_box_->findText(parts.at(1)) >= 0) mesh_box_->setCurrentText(parts.at(1));
    });
    connect(suggestion_list_, &QListWidget::currentRowChanged, this, [this](int row) {
        if (row < 0 || rebuilding_) return;
        const QStringList pair = suggestion_list_->item(row)->data(Qt::UserRole).toStringList();
        select_part(pair.value(0));
        if (mesh_box_->findText(pair.value(1)) >= 0) mesh_box_->setCurrentText(pair.value(1));
    });
    connect(accept_suggestion_, &QPushButton::clicked, this, &MainWindow::on_accept_suggestion);
    connect(accept_all_, &QPushButton::clicked, this, [this] { accept_suggested_meshes(); });

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
    label_edit_ = new QLineEdit;
    label_edit_->setPlaceholderText(tr("none: the part name is shown"));
    apply_label_ = new QPushButton(tr("Set label"));
    apply_label_->setToolTip(tr("Show this text in place of the part name in the model tree, the part list and the viewport "
                                "(undoable). Leave it empty to go back to the part name."));
    form->addWidget(new QLabel(tr("Label")), 1, 0);
    form->addWidget(label_edit_, 1, 1);
    form->addWidget(apply_label_, 1, 2);
    form->addWidget(new QLabel(tr("Meshes with")), 2, 0);
    form->addWidget(mesh_box_, 2, 1);
    form->addWidget(declare_mesh_, 2, 2);
    winding_button_ = new QPushButton(tr("In or out of the train"));
    winding_button_->setToolTip(tr("Take the mesh between the selected part and this partner out of the going train (the "
                                   "winding work, say), or put it back in (undoable)"));
    form->addWidget(winding_button_, 3, 2);
    remove_mesh_button_ = new QPushButton(tr("Remove mesh"));
    remove_mesh_button_->setToolTip(tr("Take away the mesh that was declared by hand between the selected part and this "
                                       "partner (undoable). A mesh found from the geometry cannot be removed: leave it out "
                                       "of the train instead"));
    form->addWidget(remove_mesh_button_, 3, 1);
    form->setColumnStretch(1, 1);
    layout->addWidget(selected_box_);

    connect(declare_mesh_, &QPushButton::clicked, this, &MainWindow::on_declare_mesh);
    connect(apply_role_, &QPushButton::clicked, this, &MainWindow::on_apply_role);
    connect(apply_label_, &QPushButton::clicked, this, &MainWindow::on_apply_label);
    connect(remove_mesh_button_, &QPushButton::clicked, this, [this] {
        if (!scene_ || viewport_->selected().empty() || mesh_box_->currentText().isEmpty()) {
            statusBar()->showMessage(tr("Select a part and the part it meshes with first"), 4000);
            return;
        }
        if (!remove_mesh(QString::fromStdString(viewport_->selected()), mesh_box_->currentText())) {
            statusBar()->showMessage(tr("Only a mesh declared by hand can be removed; use \"In or out of the train\" for the others"), 8000);
        }
    });
    connect(winding_button_, &QPushButton::clicked, this, [this] {
        if (!scene_ || viewport_->selected().empty() || mesh_box_->currentText().isEmpty()) {
            statusBar()->showMessage(tr("Select a part and the part it meshes with first"), 4000);
            return;
        }
        const auto& m = scene_->movement();
        const int a = m.find_component(viewport_->selected()), b = m.find_component(mesh_box_->currentText().toStdString());
        for (const auto& j : m.meshes) {
            if ((j.a == a && j.b == b) || (j.a == b && j.b == a)) {
                set_winding(QString::fromStdString(viewport_->selected()), mesh_box_->currentText(), m.in_train(j));
                return;
            }
        }
        statusBar()->showMessage(tr("Those two parts are not meshed"), 4000);
    });
    connect(label_edit_, &QLineEdit::returnPressed, this, &MainWindow::on_apply_label);
    connect(rig_table_, &QTableWidget::itemSelectionChanged, this, [this] {
        const auto rows = rig_table_->selectionModel()->selectedRows();
        if (rows.isEmpty() || rebuilding_) return;
        // The part clicked last is the one the other panels follow; the rest of the selection stays for "Set role".
        const int row = rig_table_->currentRow() >= 0 && rig_table_->selectionModel()->isRowSelected(rig_table_->currentRow())
                            ? rig_table_->currentRow() : rows.first().row();
        select_part(rig_table_->item(row, 0)->data(Qt::UserRole).toString());
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
        timegrapher_ = new TimegrapherScreen;
        timegrapher_->setToolTip(tr("The simulated beats of the timegrapher run. Double-click to run them again."));
        timegrapher_->hide();  // until a timegrapher run has been made
        hl->addWidget(timegrapher_, 3);
        hl->addWidget(plot_);
        hl->addWidget(results_, 2);
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
    if (import_thread_) {
        import_cancel_ = true;  // stops between parts rather than waiting for the whole import
        import_thread_->wait();  // a running import cannot be abandoned mid-write
    }
    cli::set_import_hooks({});
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
    // No wait for queued jobs: they run on their own copies of the project, and the project is saved atomically.
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

bool MainWindow::reload_rig(std::optional<model::Movement> rig) {
    if (!scene_) return false;
    const QString selected = QString::fromStdString(viewport_->selected());
    bool reused = false;
    try {
        // Metadata only: the geometry is kept.
        reused = rig ? scene_->reload_rig(std::move(*rig)) : scene_->reload_rig(project_.toStdString());
    } catch (const std::exception& e) {
        append_log(tr("Cannot open %1: %2").arg(project_, e.what()));
        return false;
    }
    if (!reused) return load_project(project_, step_);
    refresh_panels();
    for (QTreeWidgetItemIterator it(tree_); *it; ++it) {
        if ((*it)->parent()) {
            const QSignalBlocker block(tree_);
            (*it)->setCheckState(0, scene_->visible((*it)->data(0, Qt::UserRole).toString().toStdString()) ? Qt::Checked : Qt::Unchecked);
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
    walk_pos_ = 0;
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
    const QString pristine = QDir(dir).filePath("demo.pristine.hsproj");
    // Built once per directory: the movement is the same every time, so later opens only load it (from its cache).
    // The project is put back as it was imported, as opening the demonstration always gave a fresh movement.
    if (QFileInfo::exists(step) && QFileInfo::exists(pristine)) {
        QFile::remove(project);
        if (QFile::copy(pristine, project)) {
            load_async(tr("Opening the demonstration movement..."), {}, project, step);
            return;
        }
    }
    load_async(tr("Building the demonstration movement..."),
               {{"dev", "gen-movement", "-o", step}, {"import", step, "-o", project, "--beat-rate", "28800"}}, project, step,
               [project, pristine] {
                   QFile::remove(pristine);
                   QFile::copy(project, pristine);
               });
}

void MainWindow::load_async(const QString& message, const std::vector<QStringList>& commands, const QString& project,
                            const QString& step, std::function<void()> on_loaded) {
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
    import_cancel_ = false;
    cancel_load_->setEnabled(true);
    cli::ImportHooks hooks;
    hooks.cancel = &import_cancel_;
    hooks.progress = [this](std::size_t done, std::size_t total) {
        QMetaObject::invokeMethod(this, [this, done, total] {
            if (importing_) statusBar()->showMessage(tr("Importing: part %1 of %2").arg(done).arg(total));
        }, Qt::QueuedConnection);
    };
    cli::set_import_hooks(hooks);
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
    connect(thread, &QThread::finished, this, [this, thread, result, commands, step, project, on_loaded] {
        thread->deleteLater();
        import_thread_ = nullptr;
        importing_ = false;
        cli::set_import_hooks({});
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
                const bool cancelled = import_cancel_ && QString::fromStdString(captured.err).contains("cancelled");
                statusBar()->showMessage(cancelled ? tr("Import cancelled") : tr("%1 failed; see the log").arg(commands[i].value(0)), 8000);
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
        if (on_loaded) on_loaded();
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
    QSaveFile f(project_);  // a job reading the project sees the old file or the new one, never half of one
    if (!f.open(QIODevice::WriteOnly) || f.write(text) != text.size() || !f.commit()) {
        append_log(tr("Cannot write %1").arg(project_));
        return false;
    }
    history_ << "# the project file was restored by undo or redo";
    return reload_rig();
}

bool MainWindow::edit_project(const QString& label, const QStringList& args, QString* out) {
    if (project_.isEmpty()) return false;
    const QByteArray before = read_file(project_);
    if (run_command(args, out) != 0) return false;
    const QByteArray after = read_file(project_);
    if (!reload_rig()) {
        restore_project(before);
        return false;
    }
    if (after != before) undo_->push(new ProjectEdit(this, label, before, after));
    return true;
}

void MainWindow::set_show_rig(bool on) {
    viewport_->set_show_rig(on);
    update_walk_view();
    if (rig_overlay_->isChecked() != on) {
        const QSignalBlocker block(rig_overlay_);
        rig_overlay_->setChecked(on);
    }
    if (rig_overlay_action_->isChecked() != on) {
        const QSignalBlocker block(rig_overlay_action_);
        rig_overlay_action_->setChecked(on);
    }
}

bool MainWindow::rerig(bool reset) {
    if (!scene_) return false;
    QString out;
    QStringList args{"rig", "auto", project_};
    if (reset) args << "--reset";
    if (!edit_project(reset ? tr("Re-rig from scratch") : tr("Re-run auto-rig"), args, &out)) return false;
    append_log(out.trimmed());
    statusBar()->showMessage(out.section('\n', 0, 0), 8000);
    return true;
}

bool MainWindow::set_beat_rate(double vph) {
    if (!scene_ || !(vph > 0.0)) return false;
    if (scene_->movement().beat_rate_vph == vph) return true;
    return edit_project(tr("Beat rate %1 vph").arg(vph, 0, 'f', 0), {"rig", "beat-rate", project_, QString::number(vph, 'f', 0)});
}

bool MainWindow::solve_train(bool ask) {
    if (!scene_) return false;
    if (scene_->movement().beat_rate_vph <= 0.0) {
        statusBar()->showMessage(tr("The project has no beat rate yet: set it in the Beat rate box above, then solve again"), 10000);
        beat_box_->setFocus();
        return false;
    }
    QString out;
    if (run_command({"--json", "rig", "solve", project_}, &out) != 0) return false;
    const QJsonObject solution = QJsonDocument::fromJson(out.toUtf8()).object();
    QStringList report;
    for (const QJsonValue& h : solution["hands"].toArray()) {
        const QJsonObject hand = h.toObject();
        report << tr("%1 wheel: %2").arg(hand["hand"].toString(), hand["mobile"].toString());
    }
    for (const QJsonValue& n : solution["notes"].toArray()) report << n.toString();
    const QJsonArray candidates = solution["candidates"].toArray();
    bool joined = solution["connected"].toBool();
    if (!candidates.isEmpty()) {
        const QJsonObject best = candidates.first().toObject();
        const QString text = tr("%1 probably drives %2: %3 teeth%4, centre distance %5 mm, turning %6 times an hour.\n\nDeclare that mesh?")
                                 .arg(best["driver"].toString(), best["input"].toString())
                                 .arg(best["teeth"].toInt())
                                 .arg(best["estimated"].toBool() ? tr(" (worked out: none were found)") : QString())
                                 .arg(best["centre_distance"].toDouble(), 0, 'f', 2)
                                 .arg(best["driver_turns_per_hour"].toDouble(), 0, 'f', 3);
        if (!ask || QMessageBox::question(this, tr("Solve the train"), text) == QMessageBox::Yes) {
            QString applied;
            joined = edit_project(tr("Solve the train: %1 drives %2").arg(best["driver"].toString(), best["input"].toString()),
                                  {"rig", "solve", project_, "--apply"}, &applied);
            if (joined) {
                report.clear();
                QString again;
                if (run_command({"--json", "rig", "solve", project_}, &again) == 0) {
                    const QJsonObject after = QJsonDocument::fromJson(again.toUtf8()).object();
                    for (const QJsonValue& h : after["hands"].toArray()) {
                        const QJsonObject hand = h.toObject();
                        report << tr("%1 wheel: %2").arg(hand["hand"].toString(), hand["mobile"].toString());
                    }
                    for (const QJsonValue& n : after["notes"].toArray()) report << n.toString();
                    joined = after["connected"].toBool();
                }
            }
        }
    } else if (!solution["connected"].toBool() && report.isEmpty()) {
        report << tr("Nothing fits as the driver: declare the mesh by hand.");
    }
    for (const QString& line : report) append_log(line);
    statusBar()->showMessage(joined ? tr("The train is connected. %1").arg(report.join("; ")) : tr("The train is not connected. %1").arg(report.join("; ")), 15000);
    return joined;
}

bool MainWindow::accept_suggested_meshes() {
    if (!scene_ || scene_->review().suggestions.empty()) {
        statusBar()->showMessage(tr("No suggested meshes to declare"), 4000);
        return false;
    }
    QString out;
    const int n = static_cast<int>(scene_->review().suggestions.size());
    if (!edit_project(tr("Declare %n suggested mesh(es)", nullptr, n), {"rig", "suggest", project_, "--apply"}, &out)) return false;
    statusBar()->showMessage(out.trimmed().section('\n', -1), 6000);
    return true;
}

void MainWindow::on_accept_suggestion() {
    const int row = suggestion_list_->currentRow();
    if (row < 0) {
        statusBar()->showMessage(tr("Select a suggested mesh first"), 4000);
        return;
    }
    const QStringList pair = suggestion_list_->item(row)->data(Qt::UserRole).toStringList();
    declare_mesh(pair.value(0), pair.value(1));
}

void MainWindow::refresh_rig_review() {
    const Scene::RigReview& review = scene_->review();
    const model::Movement& m = scene_->movement();
    const QSignalBlocker issues_block(issue_list_), suggestions_block(suggestion_list_);

    // The train, as a line a person can read.
    const rigging::TrainChain& chain = review.chain;
    const QString dot = chain.connected ? "<span style='color:#3cc86e'>&#9679;</span> " : "<span style='color:#f0aa28'>&#9679;</span> ";
    const int meshes = static_cast<int>(review.walk.steps.size());
    chain_label_->setText(dot + (chain.connected ? tr("<b>Connected</b> from the barrel to the escape wheel, in %n mesh(es).", nullptr, meshes)
                                                 : tr("<b>Not connected</b> from the barrel to the escape wheel: step %1 is missing.").arg(meshes)));
    const auto left_out = std::count_if(m.meshes.begin(), m.meshes.end(), [&m](const model::MeshJoint& j) { return !m.in_train(j); });
    if (left_out > 0) chain_label_->setText(chain_label_->text() + tr(" %n mesh(es) left out of the train as winding work.", nullptr, static_cast<int>(left_out)));

    issue_list_->clear();
    int to_do = 0;
    for (const rigging::Issue& issue : review.issues) {
        QStringList parts;
        for (const std::string& p : issue.parts) parts << QString::fromStdString(p);
        const bool info = issue.severity == rigging::Severity::Info;
        auto* item = new QListWidgetItem((info ? tr("Note: ") : issue.severity == rigging::Severity::Error ? tr("Error: ") : tr("Warning: ")) +
                                         QString::fromStdString(issue.message).replace("->", QString::fromUtf8("→")).replace("[no mesh]", QString::fromUtf8("✗ no mesh")));
        item->setData(Qt::UserRole, parts);
        if (info) item->setForeground(palette().placeholderText());
        else item->setForeground(issue.severity == rigging::Severity::Error ? QColor(225, 95, 80) : QColor(220, 165, 60));
        issue_list_->addItem(item);
        to_do += info ? 0 : 1;
    }
    if (issue_list_->count() == 0) issue_list_->addItem(tr("Nothing left to check."));

    suggestion_list_->clear();
    for (const rigging::MeshSuggestion& s : review.suggestions) {
        const QString a = QString::fromStdString(m.components[static_cast<std::size_t>(s.a)].features.name);
        const QString b = QString::fromStdString(m.components[static_cast<std::size_t>(s.b)].features.name);
        const auto shown = [&m](int c) { return QString::fromStdString(m.components[static_cast<std::size_t>(c)].display_name()); };
        auto* item = new QListWidgetItem(tr("%1  ↔  %2   (module %3, likelihood %4)").arg(shown(s.a), shown(s.b)).arg(s.module, 0, 'f', 3).arg(s.confidence, 0, 'f', 2));
        item->setToolTip(tr("%1 ↔ %2").arg(a, b));
        item->setData(Qt::UserRole, QStringList{a, b});
        suggestion_list_->addItem(item);
    }
    accept_suggestion_->setEnabled(!review.suggestions.empty());
    accept_all_->setEnabled(!review.suggestions.empty());

    rig_tabs_->setTabText(1, to_do > 0 ? tr("To do (%1)").arg(to_do) : tr("To do"));
    rig_tabs_->setTabText(2, review.suggestions.empty() ? tr("Suggested meshes") : tr("Suggested meshes (%1)").arg(review.suggestions.size()));
    update_walk();
}

void MainWindow::go_to_step(int step) {
    if (!scene_) return;
    const int last = std::max(0, static_cast<int>(scene_->review().walk.steps.size()) - 1);
    walk_pos_ = std::clamp(step, 0, last);
    update_walk();
}

void MainWindow::pick_partner_for(const QString& from) {
    picking_ = true;
    pick_from_ = from;
    pick_role_.clear();
    update_pick_ui();
}

void MainWindow::pick_part_for_role(const QString& role) {
    picking_ = true;
    pick_role_ = role;
    pick_from_.clear();
    update_pick_ui();
}

void MainWindow::update_pick_ui() {
    if (!picking_) {
        viewport_->set_prompt({});
        statusBar()->clearMessage();
        return;
    }
    const auto shown = [this](const QString& part) {
        const int i = scene_ ? scene_->movement().find_component(part.toStdString()) : -1;
        return i >= 0 ? QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(i)].display_name()) : part;
    };
    QString role = pick_role_;
    const QString what = pick_role_.isEmpty() ? tr("Click the part that %1 drives, in the viewport").arg(shown(pick_from_))
                                              : tr("Click the %1 in the viewport").arg(role.replace('_', ' '));
    viewport_->set_prompt(what + tr("  (Esc cancels)"));
    statusBar()->showMessage(what + tr(". Click beside the movement or press Esc to cancel."));
    pick_partner_->setText(tr("Cancel: click a part in the viewport"));
}

void MainWindow::cancel_pick() {
    if (!picking_) return;
    picking_ = false;
    update_pick_ui();
    statusBar()->showMessage(tr("Cancelled"), 3000);
    update_walk();  // the button's text goes back
}

void MainWindow::on_part_picked(const QString& name) {
    if (!picking_) {
        select_part(name);
        return;
    }
    picking_ = false;
    update_pick_ui();
    if (name.isEmpty()) {
        statusBar()->showMessage(tr("Cancelled"), 3000);
        select_part(name);
        update_walk();
        return;
    }
    if (!pick_role_.isEmpty()) {
        change_role(name, pick_role_);
    } else if (name == pick_from_) {
        statusBar()->showMessage(tr("A part cannot mesh with itself: pick another one"), 6000);
    } else if (!declare_mesh(pick_from_, name)) {
        // The command says why in the log; the usual reasons are named here, as the log is easy to miss.
        const auto& m = scene_->movement();
        const int a = m.find_component(pick_from_.toStdString()), b = m.find_component(name.toStdString());
        QString why = tr("see the Log panel");
        if (a >= 0 && b >= 0) {
            if (m.components[static_cast<std::size_t>(b)].features.teeth <= 0) why = tr("%1 has no teeth found, so no tooth count to mesh with").arg(name);
            else if (m.components[static_cast<std::size_t>(a)].features.teeth <= 0) why = tr("%1 has no teeth found, so no tooth count to mesh with").arg(pick_from_);
            else if (m.mobile_of(a) >= 0 && m.mobile_of(a) == m.mobile_of(b)) why = tr("both parts turn together on one mobile");
            else if (m.mobile_of(b) < 0) why = tr("%1 does not rotate: give it a role such as wheel or pinion first").arg(name);
        }
        statusBar()->showMessage(tr("Could not declare that mesh: %1").arg(why), 10000);
    }
    select_part(name);
    update_walk();
}

void MainWindow::update_walk() {
    const auto& walk = scene_->review().walk;
    const int count = static_cast<int>(walk.steps.size());
    walk_pos_ = std::clamp(walk_pos_, 0, std::max(0, count - 1));
    const auto name_of = [this](int component) {  // the part name: what identifies a part
        return component >= 0 ? QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(component)].features.name) : QString();
    };
    const auto shown_of = [this](int component) {  // what to call it on screen: its label when it has one
        return component >= 0 ? QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(component)].display_name()) : QString();
    };
    using Kind = rigging::WalkStep::Kind;

    // The chips: the part the barrel starts with, then one for each step, with the one being looked at pressed in.
    while (QLayoutItem* item = chips_layout_->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    QToolButton* current_chip = nullptr;
    const auto add_chip = [this, &current_chip](const QString& text, bool current, int step, const QString& tip, const QString& style = {}) {
        auto* chip = new QToolButton;
        chip->setText(text);
        chip->setCheckable(true);
        chip->setChecked(current);
        if (current) current_chip = chip;
        chip->setToolTip(tip);
        if (!style.isEmpty()) chip->setStyleSheet(style);
        connect(chip, &QToolButton::clicked, this, [this, step] { go_to_step(step); });
        chips_layout_->addWidget(chip);
    };
    const QString amber = "QToolButton { color: #f0aa28; font-weight: bold; }";
    if (count > 0 && walk.steps[0].driver >= 0) {
        auto* start = new QLabel(shown_of(walk.steps[0].driver));
        start->setToolTip(tr("The start of the train: %1").arg(name_of(walk.steps[0].driver)));
        chips_layout_->addWidget(start);
    }
    for (int i = 0; i < count; ++i) {
        const auto& step = walk.steps[static_cast<std::size_t>(i)];
        auto* arrow = new QLabel(QString::fromUtf8("→"));
        chips_layout_->addWidget(arrow);
        if (step.kind == Kind::Mesh) {
            add_chip(shown_of(step.driven), i == walk_pos_, i,
                     tr("Step %1 (%2): %3").arg(i + 1).arg(name_of(step.driven), QString::fromStdString(step.text)));
        } else {
            add_chip(QString::fromUtf8("✗ ?"), i == walk_pos_, i, QString::fromStdString(step.text), amber);
        }
    }
    chips_layout_->addStretch(1);
    if (current_chip) {
        // The strip is laid out once it is shown, so scroll to the chip then.
        QTimer::singleShot(0, chips_scroll_, [scroll = chips_scroll_, chip = QPointer<QToolButton>(current_chip)] {
            if (chip) scroll->ensureWidgetVisible(chip, 40, 0);
        });
    }

    // The card for the step.
    const bool have = count > 0;
    prev_step_->setEnabled(have && walk_pos_ > 0);
    next_step_->setEnabled(have && walk_pos_ + 1 < count);
    gap_box_->setVisible(false);
    leave_out_->setVisible(have && walk.steps[static_cast<std::size_t>(walk_pos_)].kind == Kind::Mesh);
    const QSignalBlocker candidates_block(candidates_);  // the viewport is updated once, at the end
    candidates_->clear();
    if (!have) {
        step_title_->setText(tr("Nothing to walk"));
        step_text_->setText(tr("The rig has no parts to walk."));
    } else {
        const auto& step = walk.steps[static_cast<std::size_t>(walk_pos_)];
        QString text = QString::fromStdString(step.text).toHtmlEscaped();
        if (step.kind == Kind::Mesh) {
            step_title_->setText(tr("Step %1 of %2").arg(walk_pos_ + 1).arg(count));
            if (walk.complete && walk_pos_ + 1 == count) {
                text += tr("<br><b>That is the escape wheel: the train is complete.</b> ")
                        + (scene_->can_animate() ? tr("The animation and the simulations are ready.")
                                                 : tr("Set a beat rate for the animation and the simulations."));
            }
        } else {
            step_title_->setText(step.kind == Kind::Gap ? tr("Step %1: the chain stops here").arg(walk_pos_ + 1) : tr("Step %1: a part is missing").arg(walk_pos_ + 1));
            gap_box_->setVisible(true);
            const bool gap = step.kind == Kind::Gap;
            candidates_->setVisible(gap && !step.suggestions.empty());
            declare_candidate_->setVisible(gap && !step.suggestions.empty());
            change_role_->setVisible(gap && step.driver >= 0);
            pick_partner_->setText(picking_ ? tr("Cancel: click a part in the viewport")
                                   : gap     ? tr("Pick what it drives...")
                                             : tr("Pick the %1...").arg(QString::fromUtf8(model::to_string(step.needs_role).data()).replace('_', ' ')));
            pick_partner_->setToolTip(gap ? tr("Press this, then click in the 3D viewport the part that this one drives. The viewport "
                                               "shows a prompt and the cursor turns into a cross; click beside the movement or press "
                                               "Esc to cancel")
                                          : tr("Press this, then click that part in the 3D viewport; Esc cancels"));
            if (gap) {
                const auto& suggestions = scene_->review().suggestions;
                for (int k : step.suggestions) {
                    const auto& s = suggestions[static_cast<std::size_t>(k)];
                    auto* item = new QListWidgetItem(tr("%1  ↔  %2   (likelihood %3)").arg(shown_of(s.a), shown_of(s.b)).arg(s.confidence, 0, 'f', 2));
                    item->setToolTip(tr("%1 ↔ %2").arg(name_of(s.a), name_of(s.b)));
                    item->setData(Qt::UserRole, QStringList{name_of(s.a), name_of(s.b)});
                    candidates_->addItem(item);
                }
                if (candidates_->count() > 0) candidates_->setCurrentRow(0);
            }
        }
        step_text_->setText(text);
    }
    update_walk_view();
}

void MainWindow::update_walk_view() {
    if (!scene_ || !viewport_) return;
    const auto& walk = scene_->review().walk;
    if (walk.steps.empty() || workspaces_->currentIndex() != 0 || rig_overlay_->isChecked()) {
        viewport_->set_walk(std::nullopt);
        return;
    }
    const model::Movement& m = scene_->movement();
    Viewport::WalkView view;
    view.walked.assign(m.components.size(), 0);
    const auto walk_mobile = [&](int component) {
        const int mobile = component >= 0 ? m.mobile_of(component) : -1;
        if (mobile < 0) return;
        for (int idx : m.mobiles[static_cast<std::size_t>(mobile)].components) view.walked[static_cast<std::size_t>(idx)] = 1;
    };
    for (int i = 0; i <= walk_pos_ && i < static_cast<int>(walk.steps.size()); ++i) {
        walk_mobile(walk.steps[static_cast<std::size_t>(i)].driver);
        walk_mobile(walk.steps[static_cast<std::size_t>(i)].driven);
    }
    if (std::none_of(view.walked.begin(), view.walked.end(), [](char w) { return w != 0; })) {
        viewport_->set_walk(std::nullopt);  // nothing is reached yet (no barrel): fading everything would show nothing
        return;
    }
    const auto& step = walk.steps[static_cast<std::size_t>(walk_pos_)];
    view.driver = step.driver;
    view.driven = step.driven;
    view.gap = step.kind != rigging::WalkStep::Kind::Mesh;
    if (view.gap && candidates_->currentItem()) {
        const QStringList pair = candidates_->currentItem()->data(Qt::UserRole).toStringList();
        view.preview_a = m.find_component(pair.value(0).toStdString());
        view.preview_b = m.find_component(pair.value(1).toStdString());
    }
    viewport_->set_walk(view);
}

void MainWindow::refresh_panels() {
    rebuilding_ = true;
    const auto& m = scene_->movement();
    // One repaint of each view at the end instead of one per inserted row.
    tree_->setUpdatesEnabled(false);
    rig_table_->setUpdatesEnabled(false);
    // A ResizeToContents column measures every row on every insert, which makes filling the tables quadratic in
    // the part count. Switch to fixed widths while they are filled and size the columns once at the end.
    tree_->header()->setSectionResizeMode(1, QHeaderView::Fixed);
    for (int c = 1; c < 4; ++c) rig_table_->horizontalHeader()->setSectionResizeMode(c, QHeaderView::Fixed);

    tree_->clear();
    std::vector<QTreeWidgetItem*> mobile_items;
    for (const auto& mob : m.mobiles) {
        auto* item = new QTreeWidgetItem(tree_, {QString::fromStdString(mob.name), tr("mobile")});
        item->setExpanded(true);
        mobile_items.push_back(item);
    }
    auto* fixed = new QTreeWidgetItem(tree_, {tr("fixed parts"), QString()});
    fixed->setExpanded(true);
    const std::vector<int> mobile_at = m.mobile_index();
    for (std::size_t i = 0; i < m.components.size(); ++i) {
        const auto& c = m.components[i];
        const int mob = mobile_at[i];
        auto* parent = mob >= 0 ? mobile_items[mob] : fixed;
        auto* item = new QTreeWidgetItem(parent, {QString::fromStdString(c.display_name()),
                                                  QString::fromUtf8(model::to_string(c.role).data())});
        item->setData(0, Qt::UserRole, QString::fromStdString(c.features.name));
        item->setToolTip(0, QString::fromStdString(c.features.name));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEditable);
        item->setCheckState(0, Qt::Checked);
    }

    rig_table_->setRowCount(static_cast<int>(m.components.size()));
    std::vector<int> order(m.components.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return m.components[a].role_confidence < m.components[b].role_confidence; });
    for (std::size_t row = 0; row < order.size(); ++row) {
        const auto& c = m.components[order[row]];
        auto* part_item = new QTableWidgetItem(QString::fromStdString(c.display_name()));
        part_item->setData(Qt::UserRole, QString::fromStdString(c.features.name));
        part_item->setToolTip(QString::fromStdString(c.features.name));
        rig_table_->setItem(static_cast<int>(row), 0, part_item);
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
    {
        // The beat rate box shows the project's: a listed rate by its entry, any other as the number.
        const QSignalBlocker block(beat_box_);
        const double vph = m.beat_rate_vph;
        const int at = vph > 0 ? beat_box_->findData(static_cast<int>(vph)) : -1;
        if (at >= 0) beat_box_->setCurrentIndex(at);
        else {
            beat_box_->setCurrentIndex(-1);
            beat_box_->lineEdit()->setText(vph > 0 ? QString::number(vph, 'f', 0) : QString());
        }
    }
    apply_parts_filter();
    const QString partner = mesh_box_->currentText();
    mesh_box_->clear();
    for (const auto& c : m.components) {
        if (c.features.teeth > 0) mesh_box_->addItem(QString::fromStdString(c.features.name));
    }
    if (const int keep = mesh_box_->findText(partner); keep >= 0) mesh_box_->setCurrentIndex(keep);
    refresh_rig_review();
    tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    for (int c = 1; c < 4; ++c) rig_table_->horizontalHeader()->setSectionResizeMode(c, QHeaderView::ResizeToContents);
    tree_->setUpdatesEnabled(true);
    rig_table_->setUpdatesEnabled(true);
    rebuilding_ = false;

    QString s = tr("%1 | %2 parts").arg(QFileInfo(project_).fileName()).arg(m.components.size());
    if (!scene_->train_error().empty()) append_log(tr("Train not analysed: %1").arg(QString::fromStdString(scene_->train_error())));
    if (!scene_->can_animate()) {
        s += scene_->review().chain.connected ? tr(" | the animation is off (no beat rate)")
                                              : tr(" | the animation is off: the train is not connected from the barrel to the "
                                                   "escape wheel (see the Rig tab)");
    }
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
    const QString shown = has ? QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(index)].display_name()) : part;
    selection_label_->setText(has ? tr("Selected: %1").arg(shown) : QString());
    selection_label_->setToolTip(has ? part : QString());
    selected_box_->setTitle(has ? tr("Selected part: %1").arg(shown) : tr("Selected part (click one in the list or the viewport)"));
    for (QWidget* w : {static_cast<QWidget*>(role_box_), static_cast<QWidget*>(apply_role_), static_cast<QWidget*>(label_edit_),
                       static_cast<QWidget*>(apply_label_), static_cast<QWidget*>(mesh_box_),
                       static_cast<QWidget*>(declare_mesh_), static_cast<QWidget*>(winding_button_),
                       static_cast<QWidget*>(remove_mesh_button_)})
        w->setEnabled(has);
    if (has) {
        // The role box starts at the part's current role, so Set role changes only what is changed.
        const auto& c = scene_->movement().components[static_cast<std::size_t>(index)];
        role_box_->setCurrentText(QString::fromUtf8(model::to_string(c.role).data()));
        label_edit_->setText(QString::fromStdString(c.label));
    } else {
        label_edit_->clear();
    }
    analyse_part_label_->setText(has ? tr("Selected part: <b>%1</b>").arg(shown.toHtmlEscaped())
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
        if ((*it)->data(0, Qt::UserRole).toString() == name) tree_->setCurrentItem(*it);
    }
    for (int row = 0; row < rig_table_->rowCount(); ++row) {
        const auto* item = rig_table_->item(row, 0);
        if (item && item->data(Qt::UserRole).toString() == name) {
            if (!rig_table_->selectionModel()->isRowSelected(row)) rig_table_->selectRow(row);  // keep a multiple selection
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
    if (!scene_) return menu;
    // Show all unhides everything; Show only hides every role but the one picked.
    QStringList present;
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::kLastRole); ++r) {
        const QString name = QString::fromUtf8(model::to_string(static_cast<model::Role>(r)).data());
        for (const ScenePart& part : scene_->parts()) {
            if (part.role == static_cast<model::Role>(r)) {
                present << name;
                break;
            }
        }
    }
    connect(menu->addAction(tr("Show all")), &QAction::triggered, this, [this, present] {
        for (const QString& role : present) set_role_visible(role, true);
    });
    QMenu* only = menu->addMenu(tr("Show only"));
    for (const QString& name : present) {
        connect(only->addAction(name), &QAction::triggered, this, [this, present, name] {
            for (const QString& role : present) set_role_visible(role, role == name);
        });
    }
    menu->addSection(tr("Show parts by role"));
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
    select_part(items.first()->data(0, Qt::UserRole).toString());
}

void MainWindow::on_tree_changed(QTreeWidgetItem* item) {
    if (rebuilding_ || !scene_ || !item || !item->parent()) return;
    // Only the item that changed: ticking one part used to re-apply every part's check state.
    const std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    const int index = scene_->movement().find_component(name);
    if (index >= 0 && item->text(0).toStdString() != scene_->movement().components[static_cast<std::size_t>(index)].display_name()) {
        // Renamed in place with F2. The tree is rebuilt by the edit, so wait until this signal has returned.
        QTimer::singleShot(0, this, [this, part = QString::fromStdString(name), text = item->text(0)] { change_label(part, text); });
        return;
    }
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
    // One run gives the text and the data for the plot or the screen: the data is written beside the project as well.
    if (args.value(0) == "timegrapher") args << "--csv" << project_ + ".timegrapher.csv" << "--json-out" << project_ + ".timegrapher.json";
    if (args.value(0) == "timekeeping") args << "--json-out" << project_ + ".timekeeping.json";
    submit(static_cast<QPushButton*>(sender())->text(), args);
}

bool MainWindow::change_role(const QString& part, const QString& role) {
    if (!scene_) return false;
    const int index = scene_->movement().find_component(part.toStdString());
    if (index < 0) return false;
    return change_roles({part}, role);
}

bool MainWindow::change_roles(const QStringList& parts, const QString& role) {
    if (!scene_ || parts.isEmpty()) return false;
    const auto next = model::role_from_string(role.toStdString());
    if (!next) return false;
    std::vector<std::pair<std::string, model::Role>> roles;
    QStringList args{"rig", "set", project_};
    QString from;
    for (const QString& part : parts) {
        const int index = scene_->movement().find_component(part.toStdString());
        if (index < 0) return false;
        const model::Role current = scene_->movement().components[static_cast<std::size_t>(index)].role;
        if (current == *next) continue;
        roles.emplace_back(part.toStdString(), *next);
        args << part;
        from = QString::fromUtf8(model::to_string(current).data());
    }
    if (roles.empty()) return true;
    args << "--role" << role;
    const QString label = roles.size() == 1 ? tr("Role of %1: %2 to %3").arg(parts.first(), from, role)
                                            : tr("Role of %n part(s): %1", nullptr, static_cast<int>(roles.size())).arg(role);
    // Not while a command or a load is under way: those go through the command line as before.
    if (!in_command_ && !importing_) return change_roles_in_memory(roles, role, label);
    return edit_project(label, args);
}

bool MainWindow::change_roles_in_memory(const std::vector<std::pair<std::string, model::Role>>& roles, const QString& role,
                                        const QString& label) {
    const QByteArray before = read_file(project_);
    model::Movement rig = scene_->movement();
    QStringList args{"rig", "set", project_};
    for (const auto& entry : roles) args << QString::fromStdString(entry.first);
    args << "--role" << role;
    QStringList quoted;
    for (const QString& s : args) quoted << shell_quote(s);
    const QString line = "ohs " + quoted.join(' ');
    append_log("$ " + line);
    try {
        rigging::set_roles(rig, roles, {});
        model::save_rig(rig, project_.toStdString());
    } catch (const std::exception& e) {
        append_log(QString::fromUtf8(e.what()));
        append_log(tr("exit code %1").arg(2));
        return false;
    }
    append_log(tr("exit code %1").arg(0));
    history_ << line;
    const QByteArray after = read_file(project_);
    if (!reload_rig(std::move(rig))) {
        restore_project(before);
        return false;
    }
    if (after != before) undo_->push(new ProjectEdit(this, label, before, after));
    return true;
}

bool MainWindow::change_label(const QString& part, const QString& label) {
    if (!scene_) return false;
    const int index = scene_->movement().find_component(part.toStdString());
    if (index < 0) return false;
    const QString text = label.trimmed();
    if (QString::fromStdString(scene_->movement().components[static_cast<std::size_t>(index)].label) == text) return true;
    QStringList args{"rig", "label", project_, part};
    if (!text.isEmpty()) args << text;
    return edit_project(text.isEmpty() ? tr("Remove the label of %1").arg(part) : tr("Label %1: %2").arg(part, text), args);
}

bool MainWindow::set_winding(const QString& a, const QString& b, bool winding) {
    if (!scene_ || a == b) return false;
    QStringList args{"rig", "winding", project_, a, b};
    if (!winding) args << "--in-train";
    return edit_project(winding ? tr("Leave %1 / %2 out of the train").arg(a, b) : tr("Put %1 / %2 back in the train").arg(a, b), args);
}

bool MainWindow::remove_mesh(const QString& a, const QString& b) {
    if (!scene_ || a == b) return false;
    return edit_project(tr("Remove the mesh %1 / %2").arg(a, b), {"rig", "unmesh", project_, a, b});
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

void MainWindow::apply_parts_filter() {
    const QString text = parts_filter_->text().trimmed();
    for (int row = 0; row < rig_table_->rowCount(); ++row) {
        bool match = text.isEmpty();
        for (int col = 0; col < 2 && !match; ++col) {
            if (const auto* item = rig_table_->item(row, col)) match = item->text().contains(text, Qt::CaseInsensitive);
        }
        if (!match) {
            if (const auto* item = rig_table_->item(row, 0)) match = item->data(Qt::UserRole).toString().contains(text, Qt::CaseInsensitive);
        }
        rig_table_->setRowHidden(row, !match);
    }
}

void MainWindow::on_apply_label() {
    if (viewport_->selected().empty()) {
        statusBar()->showMessage(tr("Select a part first"), 4000);
        return;
    }
    change_label(QString::fromStdString(viewport_->selected()), label_edit_->text());
}

void MainWindow::on_apply_role() {
    if (viewport_->selected().empty()) {
        statusBar()->showMessage(tr("Select a part first"), 4000);
        return;
    }
    // Every row selected in the part list gets the role, as one edit; with a single selection it is the selected part.
    QStringList parts;
    for (const QModelIndex& row : rig_table_->selectionModel()->selectedRows()) {
        parts << rig_table_->item(row.row(), 0)->data(Qt::UserRole).toString();
    }
    if (parts.size() < 2) parts = {QString::fromStdString(viewport_->selected())};
    change_roles(parts, role_box_->currentText());
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

namespace {

// A job's copy of the project is `<stem>.job<N><ext>` beside it, so it finds the same CAD file and cache.
const QRegularExpression& snapshot_pattern() {
    static const QRegularExpression pattern(R"(\.job\d+(\.[^./\\]*)?$)");
    return pattern;
}

}  // namespace

QString MainWindow::snapshot_project() {
    static quint64 counter = static_cast<quint64>(QDateTime::currentMSecsSinceEpoch());
    const QFileInfo info(project_);
    const QString copy = info.dir().filePath(
        info.completeBaseName() + QString(".job%1").arg(++counter) + (info.suffix().isEmpty() ? QString() : "." + info.suffix()));
    QFile::remove(copy);
    return QFile::copy(project_, copy) ? copy : QString();
}

void MainWindow::submit(const QString& label, const QStringList& args) {
    // The job reads the project as it is now: a later rig edit changes the project, not this job's input.
    QStringList job_args = args;
    const int at = job_args.indexOf(project_);
    if (at >= 0 && !project_.isEmpty()) {
        if (const QString copy = snapshot_project(); !copy.isEmpty()) job_args[at] = copy;
    }
    queue_->submit(label, job_args);
    statusBar()->showMessage(tr("Started: %1. The result appears in the Results panel.").arg(label), 5000);
}

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
    // A finished job no longer needs its copy of the project.
    if (job.state == JobQueue::State::Done || job.state == JobQueue::State::Failed || job.state == JobQueue::State::Cancelled) {
        for (const QString& a : job.args) {
            if (snapshot_pattern().match(a).hasMatch() && QFileInfo(a).dir() == QFileInfo(project_).dir()) QFile::remove(a);
        }
    }
    update_busy();
}

void MainWindow::on_job_finished(int index) {
    const auto& job = queue_->jobs()[static_cast<std::size_t>(index)];
    QStringList quoted;
    // The session script names the project, not the job's copy of it.
    for (QString s : job.args) {
        if (snapshot_pattern().match(s).hasMatch() && QFileInfo(s).dir() == QFileInfo(project_).dir()) s = project_;
        quoted << shell_quote(s);
    }
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
    results_->setPlainText(out + (code == 0 ? QString() : tr("\n(exit code %1: see the log)").arg(code)));
    // The panel may be closed, or behind the Log or Jobs tab: bring it forward so that a run is seen to have finished.
    results_dock_->show();
    results_dock_->raise();
    statusBar()->showMessage(code == 0 ? tr("Finished: the result is in the Results panel") : tr("Failed (exit code %1): see the Results and Log panels").arg(code), 8000);

    // The data written by the same run, for the plot (rate against amplitude from the full, half and low wind) or the screen.
    const int data_at = args.indexOf("--json-out");
    if (code != 0 || data_at < 0) return;
    QFile data(args.value(data_at + 1));
    if (!data.open(QIODevice::ReadOnly)) {
        append_log(tr("The data of the run is missing: %1").arg(data.fileName()));
        return;
    }
    const QString json_text = QString::fromUtf8(data.readAll());
    if (args.value(0) == "timegrapher") {
        show_timegrapher(args, json_text);
    } else if (args.value(0) == "timekeeping") {
        try {
            const auto j = nlohmann::json::parse(json_text.toStdString());
            std::vector<std::pair<double, double>> pts;
            for (const char* key : {"full", "half", "low"})
                if (j.contains(key)) pts.emplace_back(j[key].at("amplitude_deg").get<double>(), j[key].at("rate_s_per_day").get<double>());
            plot_->set_data(tr("Isochronism: rate against amplitude"), tr("amplitude (degrees)"), tr("rate (s/day)"), std::move(pts));
            plot_->show();
        } catch (const std::exception& e) {
            append_log(tr("Plot data unreadable: %1").arg(e.what()));
        }
    }
}

void MainWindow::show_timegrapher(const QStringList& args, const QString& json_text) {
    try {
        const auto j = nlohmann::json::parse(json_text.toStdString());
        const int csv_at = args.indexOf("--csv");
        QFile file(csv_at >= 0 ? args.value(csv_at + 1) : QString());
        if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("the beats were not written");
        std::vector<ScreenBeat> beats;
        const QList<QByteArray> lines = file.readAll().split('\n');
        for (int i = 1; i < lines.size(); ++i) {  // index,time_s,tock,interval_ms,deviation_ms,lift_time_ms
            const QList<QByteArray> f = lines[i].split(',');
            if (f.size() < 6) continue;
            beats.push_back({f[0].toInt(), f[1].toDouble(), f[2].toInt() != 0, f[3].toDouble(), f[4].toDouble(), f[5].toDouble()});
        }
        if (beats.empty()) throw std::runtime_error("no beats");
        timegrapher_->set_run(std::move(beats), {j.at("rate_s_per_day").get<double>(), j.at("amplitude_deg").get<double>(), j.at("beat_error_ms").get<double>()},
                              j.value("beat_rate_vph", 0.0), j.value("lift_angle_deg", 52.0), QString::fromStdString(j.value("position", std::string{})));
        timegrapher_->show();
        resizeDocks({results_dock_}, {std::max(results_dock_->height(), 400)}, Qt::Vertical);  // room for the screen
        timegrapher_->play(4.0);
    } catch (const std::exception& e) {
        append_log(tr("Timegrapher screen data unreadable: %1").arg(e.what()));
    }
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
    open_demo_async(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) + "/demo-" + OHS_VERSION);
}

}  // namespace ohs::gui
