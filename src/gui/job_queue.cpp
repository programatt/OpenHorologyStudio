// SPDX-License-Identifier: Apache-2.0
#include "gui/job_queue.hpp"

#include <algorithm>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QThread>
#include <QTimer>

namespace ohs::gui {

namespace {

constexpr int kMaxJobs = 500;          // the journal keeps the newest finished jobs beyond this many
constexpr int kMaxOutputBytes = 1 << 20;  // per stream; a job's output is kept so it can be shown again

QString clip(QByteArray bytes) {
    if (bytes.size() > kMaxOutputBytes) bytes.truncate(kMaxOutputBytes);
    return QString::fromUtf8(bytes);
}

JobQueue::State state_from_name(const QString& name) {
    using S = JobQueue::State;
    for (S s : {S::Queued, S::Running, S::Done, S::Failed, S::Cancelled}) {
        if (JobQueue::state_name(s) == name) return s;
    }
    return S::Failed;
}

}  // namespace

JobQueue::JobQueue(QString program, QObject* parent) : QObject(parent), program_(std::move(program)) {}

JobQueue::~JobQueue() {
    if (save_pending_) save();
    stop_workers();
}

QString JobQueue::state_name(State state) {
    switch (state) {
        case State::Queued: return "queued";
        case State::Running: return "running";
        case State::Done: return "done";
        case State::Failed: return "failed";
        case State::Cancelled: return "cancelled";
    }
    return "failed";
}

void JobQueue::stop_workers() {
    // The jobs stay "running" in the journal on purpose: that is how a restart knows to run them again.
    for (Worker& w : workers_) {
        w.process->disconnect(this);
        w.process->kill();
        w.process->waitForFinished(3000);
        delete w.process;
    }
    workers_.clear();
}

JobQueue::Worker* JobQueue::worker_of(QProcess* process) {
    for (Worker& w : workers_) {
        if (w.process == process) return &w;
    }
    return nullptr;
}

void JobQueue::detach() {
    if (save_pending_) save();  // the old project's journal gets its last change
    stop_workers();
    jobs_.clear();
    journal_.clear();
}

int JobQueue::load(const QString& journal) {
    detach();
    journal_ = journal;
    int unfinished = 0;
    QFile f(journal);
    if (f.open(QIODevice::ReadOnly)) {
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        for (const QJsonValue& v : doc.array()) {
            const QJsonObject o = v.toObject();
            Job job;
            job.label = o["label"].toString();
            for (const QJsonValue& a : o["args"].toArray()) job.args << a.toString();
            job.state = state_from_name(o["state"].toString());
            job.exit_code = o["exit_code"].toInt();
            job.attempts = o["attempts"].toInt();
            job.out = o["out"].toString();
            job.err = o["err"].toString();
            if (job.args.isEmpty()) continue;
            if (job.state == State::Running) job.state = State::Queued;  // the application ended while it ran
            if (job.state == State::Queued) ++unfinished;
            jobs_.push_back(std::move(job));
        }
    }
    // Old finished jobs go first when the journal has grown long.
    while (static_cast<int>(jobs_.size()) > kMaxJobs) {
        const auto it = std::find_if(jobs_.begin(), jobs_.end(), [](const Job& j) { return j.state != State::Queued; });
        if (it == jobs_.end()) break;
        jobs_.erase(it);
    }
    save();
    for (std::size_t i = 0; i < jobs_.size(); ++i) emit changed(static_cast<int>(i));
    return unfinished;
}

void JobQueue::save_soon() {
    if (save_pending_) return;
    save_pending_ = true;
    QTimer::singleShot(0, this, [this] {
        if (save_pending_) save();
    });
}

void JobQueue::save() {
    save_pending_ = false;
    if (journal_.isEmpty()) return;
    QJsonArray array;
    for (const Job& j : jobs_) {
        QJsonObject o;
        o["label"] = j.label;
        o["args"] = QJsonArray::fromStringList(j.args);
        // A job that is running now is written as running: if we are killed, it is resumed.
        o["state"] = state_name(j.state);
        o["exit_code"] = j.exit_code;
        o["attempts"] = j.attempts;
        o["out"] = j.out;
        o["err"] = j.err;
        array.append(o);
    }
    QSaveFile f(journal_);
    if (!f.open(QIODevice::WriteOnly)) return;  // a read-only location only loses the restart feature
    f.write(QJsonDocument(array).toJson(QJsonDocument::Compact));
    f.commit();
}

int JobQueue::submit(const QString& label, const QStringList& args) {
    Job job;
    job.label = label;
    job.args = args;
    jobs_.push_back(std::move(job));
    const int index = static_cast<int>(jobs_.size()) - 1;
    save_soon();
    emit changed(index);
    start_next();
    return index;
}

int JobQueue::unfinished() const {
    return static_cast<int>(std::count_if(jobs_.begin(), jobs_.end(), [](const Job& j) {
        return j.state == State::Queued || j.state == State::Running;
    }));
}

void JobQueue::resume() { start_next(); }

bool JobQueue::cancel(int index) {
    if (index < 0 || index >= static_cast<int>(jobs_.size())) return false;
    Job& job = jobs_[static_cast<std::size_t>(index)];
    if (job.state == State::Queued) {
        job.state = State::Cancelled;
        save_soon();
        emit changed(index);
        return true;
    }
    if (job.state != State::Running) return false;
    for (Worker& w : workers_) {
        if (w.index != index || w.cancel_requested) continue;
        w.cancel_requested = true;
        w.process->terminate();
        // A worker that ignores the polite request is killed.
        QPointer<QProcess> p = w.process;
        QTimer::singleShot(1500, this, [p] {
            if (p && p->state() != QProcess::NotRunning) p->kill();
        });
        return true;
    }
    return false;
}

void JobQueue::start_next() {
    for (std::size_t i = 0; i < jobs_.size() && static_cast<int>(workers_.size()) < max_parallel_; ++i) {
        Job& job = jobs_[i];
        if (job.state != State::Queued) continue;
        job.state = State::Running;
        ++job.attempts;
        job.out.clear();
        job.err.clear();
        auto* process = new QProcess(this);
        workers_.push_back({process, static_cast<int>(i), false});
        connect(process, &QProcess::finished, this,
                [this, process](int code, QProcess::ExitStatus status) { on_process_finished(process, code, status); });
        connect(process, &QProcess::errorOccurred, this,
                [this, process](QProcess::ProcessError error) { on_process_error(process, error); });
        process->setProgram(program_);
        process->setArguments(job.args);
        save_soon();
        emit changed(static_cast<int>(i));
        process->start();
    }
}

void JobQueue::on_process_finished(QProcess* process, int exit_code, QProcess::ExitStatus status) {
    Worker* worker = worker_of(process);
    if (!worker) return;
    const int index = worker->index;
    const bool cancelled = worker->cancel_requested;
    Job& job = jobs_[static_cast<std::size_t>(index)];
    job.out = clip(process->readAllStandardOutput());
    job.err = clip(process->readAllStandardError());
    job.exit_code = status == QProcess::NormalExit ? exit_code : -1;
    if (status == QProcess::CrashExit && !cancelled) job.err += "\nthe worker process crashed";
    job.state = cancelled ? State::Cancelled : (job.exit_code == 0 ? State::Done : State::Failed);
    process->deleteLater();
    workers_.erase(workers_.begin() + (worker - workers_.data()));
    save_soon();
    emit changed(index);
    if (!cancelled) emit finished(index);
    start_next();
}

void JobQueue::on_process_error(QProcess* process, QProcess::ProcessError error) {
    if (error != QProcess::FailedToStart) return;
    Worker* worker = worker_of(process);
    if (!worker) return;
    const int index = worker->index;
    Job& job = jobs_[static_cast<std::size_t>(index)];
    job.state = State::Failed;
    job.exit_code = -1;
    job.err = "cannot start the worker program " + program_ + ": " + process->errorString();
    process->deleteLater();
    workers_.erase(workers_.begin() + (worker - workers_.data()));
    save_soon();
    emit changed(index);
    emit finished(index);
    start_next();
}

void JobQueue::wait() {
    while (unfinished() > 0) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        QThread::msleep(5);
    }
}

}  // namespace ohs::gui
