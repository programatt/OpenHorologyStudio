// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>
#include <QString>
#include <QStringList>

#include "gui/scene.hpp"
#include "gui/viewport.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTabWidget;
class QTimer;
class QTreeWidget;

namespace ohs::gui {

/// The desktop shell. Every action is an `ohs` command run in this process (the same code the
/// command line and Python run), and the commands are kept as a history that can be saved as a
/// replayable shell script.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

    /// Loads a project and its geometry. Returns false and logs the reason on failure.
    bool open_project(const QString& project, const QString& step = {});
    /// Imports a STEP file into `project` (auto-rig) and opens it.
    bool import_step(const QString& step, const QString& project, double beat_rate_vph);
    /// Writes the procedural demonstration movement into `dir` and opens it.
    bool open_demo(const QString& dir);
    /// Runs `ohs <args>` in-process; logs the command and its output. Returns the exit code.
    int run_command(const QStringList& args, QString* out = nullptr);
    /// Changes a part's role through `ohs rig set` and reloads.
    bool set_role(const QString& part, const QString& role);

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

private:
    void build_ui();
    QWidget* build_rig_page();
    QWidget* build_animate_page();
    QWidget* build_analysis_page(const QString& title, const std::vector<std::pair<QString, QStringList>>& commands);
    void refresh_panels();
    QStringList with_project(QStringList args) const;
    void append_log(const QString& text);

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
    QDoubleSpinBox* speed_{};
    QPushButton* play_{};
    QLabel* status_{};
    QTimer* timer_{};

    std::shared_ptr<Scene> scene_;
    QString project_;
    QString step_;
    QStringList history_;
    double time_{0};
    bool rebuilding_{false};
};

}  // namespace ohs::gui
