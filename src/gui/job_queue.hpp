// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QSocketNotifier>
#include <QString>
#include <QStringList>

namespace ohs::gui {

/// A queue of `ohs` commands, each run in its own worker process (one at a time unless set_max_parallel allows
/// more), so a running job can be cancelled by killing the process and a crash in a job cannot take the window
/// down. With a journal file
/// the queue survives a restart of the application: every change is written to the journal, and when it is
/// loaded again the jobs that were still queued, or running when the application ended, are queued again.
class JobQueue : public QObject {
    Q_OBJECT
public:
    enum class State { Queued, Running, Done, Failed, Cancelled };

    struct Job {
        QString label;
        QStringList args;
        State state{State::Queued};
        int exit_code{0};
        int attempts{0};  ///< how many times it was started; more than 1 means it was resumed after a restart
        QString out;      ///< standard output of the last run
        QString err;      ///< standard error of the last run
    };

    /// `program` is the executable run for every job (the `ohs` command line tool).
    explicit JobQueue(QString program, QObject* parent = nullptr);
    ~JobQueue() override;

    const QString& program() const { return program_; }

    /// Binds the queue to a journal file. Replaces the jobs in memory with the journal's (stopping a
    /// running worker first, which stays "running" in its own journal and so resumes when that journal
    /// is loaded again). Returns the number of unfinished jobs found; call resume() to run them.
    int load(const QString& journal);
    /// Stops the worker and forgets the jobs without touching the journal.
    void detach();

    int submit(const QString& label, const QStringList& args);
    /// Cancels a queued job, or kills the worker of the running one. False for a finished job.
    bool cancel(int index);
    /// Starts the next queued job if none is running.
    void resume();

    /// Sends jobs to a resident `ohs serve` listening on this Unix socket instead of starting a process for each: no
    /// start-up, warm geometry, and the output arrives while the job runs. A job the daemon cannot take (nobody listening)
    /// is run in a process as before. Cancelling closes the connection, which ends the job in the daemon. Empty: processes only.
    void set_daemon(const QString& socket_path) { daemon_ = socket_path; }
    const QString& daemon() const { return daemon_; }

    /// How many jobs may run at once (default 1). Jobs still start in the order they were queued.
    void set_max_parallel(int n) { max_parallel_ = n < 1 ? 1 : n; }
    int max_parallel() const { return max_parallel_; }

    const std::vector<Job>& jobs() const { return jobs_; }
    int unfinished() const;
    /// Keeps the event loop running until every job has finished.
    void wait();

    static QString state_name(State state);

signals:
    void changed(int index);   ///< a job was added or moved to another state
    void finished(int index);  ///< a job ended on its own: done or failed (not cancelled)

private:
    /// One running job: its process (or its connection to the daemon), which job it is, and whether it was asked to stop.
    struct Worker {
        QProcess* process{};
        int index{-1};
        bool cancel_requested{false};
        int fd{-1};                       ///< the daemon connection, when the job runs there
        QSocketNotifier* notifier{};
        std::string received;             ///< bytes of frames not yet complete
    };
    void on_daemon_readable(int fd);
    void finish_daemon(int fd, int exit_code, bool crashed);
    void start_next();
    void on_process_finished(QProcess* process, int exit_code, QProcess::ExitStatus status);
    void on_process_error(QProcess* process, QProcess::ProcessError error);
    Worker* worker_of(QProcess* process);
    void stop_workers();
    void save();
    /// Saves at the next turn of the event loop: a burst of changes (a job submitted and started, one
    /// finished and the next started) is one write of the journal, not one each.
    void save_soon();

    QString program_;
    QString journal_;
    QString daemon_;
    std::vector<Job> jobs_;
    std::vector<Worker> workers_;
    int max_parallel_{1};
    bool save_pending_{false};
};

}  // namespace ohs::gui
