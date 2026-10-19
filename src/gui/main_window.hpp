// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>
#include <QString>
#include <QStringList>

#include "cli/cli_run.hpp"
#include "gui/job_queue.hpp"
#include "gui/plot.hpp"
#include "gui/video.hpp"
#include "gui/scene.hpp"
#include "gui/viewport.hpp"

class QCheckBox;
class QComboBox;
class QListWidget;
class QUndoStack;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTabWidget;
class QTimer;
class QThread;
class QTreeWidget;

namespace ohs::gui {

/// The desktop shell. Every action is an `ohs` command run in this process (the same code the
/// command line and Python run), and the commands are kept as a history that can be saved as a
/// replayable shell script.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /// Loads a project and its geometry. Returns false and logs the reason on failure.
    bool open_project(const QString& project, const QString& step = {});
    /// Imports a STEP file into `project` (auto-rig) and opens it.
    bool import_step(const QString& step, const QString& project, double beat_rate_vph);
    /// Writes the procedural demonstration movement into `dir` and opens it.
    bool open_demo(const QString& dir);
    /// What the menus and buttons use: the same as import_step, open_project and open_demo, but run on a
    /// worker thread (through load_async) so the window stays responsive while the STEP file is read and meshed.
    /// is_loading() is true until the scene is installed (or the failure logged).
    void import_step_async(const QString& step, const QString& project, double beat_rate_vph);
    void open_project_async(const QString& project);
    void open_demo_async(const QString& dir);
    bool is_loading() const { return importing_; }
    /// Runs `ohs <args>` in-process; logs the command and its output. Returns the exit code. Refused (exit
    /// code 2) while a project is being opened in the background: the project is about to be replaced, and
    /// the command line runs one command at a time, so it would block the window until the load ends.
    int run_command(const QStringList& args, QString* out = nullptr);
    /// Changes a part's role through `ohs rig set` and reloads. Not undoable; see change_role.
    bool set_role(const QString& part, const QString& role);
    /// The undoable role change the rig page uses: pushes onto the undo stack.
    bool change_role(const QString& part, const QString& role);
    /// Declares that two toothed parts mesh (`ohs rig mesh`), undoable.
    bool declare_mesh(const QString& part_a, const QString& part_b);
    /// Runs `ohs <args>`, a command that rewrites the project file, as one undoable edit: undo puts the
    /// project file back exactly as it was and reloads it, redo restores what the command wrote. Returns
    /// false (and changes nothing) when the command fails or the result cannot be opened.
    bool edit_project(const QString& label, const QStringList& args);
    /// Replaces the project file's text and reloads it; what the undo stack uses.
    bool restore_project(const QByteArray& text);
    QUndoStack* undo_stack() const { return undo_; }
    QComboBox* mesh_partner_box() const { return mesh_box_; }

    /// Queues `ohs <args>` to run in a worker process so the window stays responsive. Jobs run one at a
    /// time in order. A queued job, or the running one (its process is killed), can be cancelled. The
    /// queue is journalled beside the project (`<project>.jobs.json`): after a restart the jobs that
    /// were unfinished run again and the finished ones keep their results (select one to see them).
    void submit(const QString& label, const QStringList& args);
    bool cancel_job(int index);
    JobQueue* job_queue() const { return queue_; }
    /// The `ohs` program the worker processes run.
    static QString worker_program();
    /// Blocks (keeping the window alive) until every queued job has finished.
    void wait_for_jobs();
    int unfinished_jobs() const;
    QListWidget* job_list() const { return jobs_; }
    PlotWidget* plot() const { return plot_; }

    /// Renders the animation to a video file with ffmpeg (see export_video in video.hpp). Returns an empty
    /// string on success, else the reason.
    QString export_video(const QString& path, const VideoOptions& options = {});

    /// Draws a field of a finite element result (a VTU file written by `ohs fea --vtu`) on a part:
    /// "von_mises" or "displacement". Returns false and logs the reason when it cannot.
    bool show_field(const QString& vtu, const QString& part, const QString& field);
    void clear_field();
    QComboBox* field_box() const { return field_box_; }

    void select_part(const QString& name);
    void set_time(double seconds);
    void set_playing(bool playing);

    /// The commands run so far, as shell lines.
    QStringList history() const { return history_; }
    bool save_script(const QString& path) const;

    Viewport* viewport() const { return viewport_; }
    QTabWidget* workspaces() const { return workspaces_; }
    QTreeWidget* model_tree() const { return tree_; }
    QTableWidget* rig_table() const { return rig_table_; }
    QPlainTextEdit* explain_panel() const { return explain_; }
    QPlainTextEdit* log_console() const { return log_; }
    QPlainTextEdit* results() const { return results_; }
    QString project() const { return project_; }
    QSlider* timeline() const { return timeline_; }
    QSlider* explode_slider() const { return explode_; }
    QCheckBox* section_check() const { return section_; }
    QCheckBox* forces_check() const { return forces_check_; }
    QCheckBox* trails_check() const { return trails_check_; }

private slots:
    void on_timer();
    void on_open();
    void on_import();
    void on_demo();
    void on_timeline(int value);
    void on_view_changed();
    void on_tree_changed();
    void on_tree_selected();
    void on_run_analysis();
    void on_apply_role();
    void on_declare_mesh();

private:
    void build_ui();
    QWidget* build_rig_page();
    QWidget* build_animate_page();
    QWidget* build_analysis_page(const QString& title, const std::vector<std::pair<QString, QStringList>>& commands);
    bool load_project(const QString& project, const QString& step);
    /// Re-reads the rig from the project file, keeping the geometry, view, visibility and selection.
    bool reload_rig();
    bool install_scene(const QString& project, const QString& step, std::shared_ptr<Scene> scene);
    /// Runs `ohs <commands>` in order, then loads `project`, on a worker thread, and installs the scene when
    /// done. One at a time; `message` shows in the status bar meanwhile.
    void load_async(const QString& message, const std::vector<QStringList>& commands, const QString& project,
                    const QString& step);
    void attach_job_journal();
    void refresh_panels();
    QStringList with_project(QStringList args) const;
    void append_log(const QString& text);
    void on_job_changed(int index);
    void on_job_finished(int index);
    void show_result(const QStringList& args, const QString& out, int code);

    Viewport* viewport_{};
    QTabWidget* workspaces_{};
    QTreeWidget* tree_{};
    QTableWidget* rig_table_{};
    QPlainTextEdit* explain_{};
    QPlainTextEdit* log_{};
    QPlainTextEdit* results_{};
    QSlider* timeline_{};
    QSlider* explode_{};
    QSlider* section_offset_{};
    QCheckBox* section_{};
    QCheckBox* stepped_{};
    QCheckBox* forces_check_{};
    QCheckBox* trails_check_{};
    QDoubleSpinBox* trail_seconds_{};
    QDoubleSpinBox* speed_{};
    QPushButton* play_{};
    QLabel* status_{};
    QTimer* timer_{};
    QUndoStack* undo_{};
    QListWidget* jobs_{};
    PlotWidget* plot_{};
    QComboBox* role_box_{};
    QComboBox* mesh_box_{};
    QComboBox* field_box_{};
    QString field_vtu_, field_part_;
    JobQueue* queue_{};                               ///< every job, in submission order (the list shows these)

    std::shared_ptr<Scene> scene_;
    QString project_;
    QString step_;
    QStringList history_;
    double time_{0};
    bool rebuilding_{false};
    bool importing_{false};
    QThread* import_thread_{};  ///< the running load_async, if any; the destructor waits for it
};

}  // namespace ohs::gui
