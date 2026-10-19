// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include <QObject>
#include <QPointer>
#include <QProcess>
#include <QString>
#include <QStringList>

namespace ohs::gui {

/// A queue of `ohs` commands run one at a time, each in its own worker process, so a running job can be
/// cancelled by killing the process and a crash in a job cannot take the window down. With a journal file
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

    const std::vector<Job>& jobs() const { return jobs_; }
    int unfinished() const;
    /// Keeps the event loop running until every job has finished.
    void wait();

    static QString state_name(State state);

signals:
    void changed(int index);   ///< a job was added or moved to another state
    void finished(int index);  ///< a job ended on its own: done or failed (not cancelled)

private:
    void start_next();
    void on_process_finished(int exit_code, QProcess::ExitStatus status);
    void on_process_error(QProcess::ProcessError error);
    void stop_worker();
    void save() const;

    QString program_;
    QString journal_;
    std::vector<Job> jobs_;
    QProcess* process_{};
    int running_{-1};
    bool cancel_requested_{false};
};

}  // namespace ohs::gui
