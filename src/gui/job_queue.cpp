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

JobQueue::~JobQueue() { stop_worker(); }

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

void JobQueue::stop_worker() {
    if (!process_) return;
    // The job stays "running" in the journal on purpose: that is how a restart knows to run it again.
    process_->disconnect(this);
    process_->kill();
    process_->waitForFinished(3000);
    delete process_;
    process_ = nullptr;
    running_ = -1;
    cancel_requested_ = false;
}

void JobQueue::detach() {
    stop_worker();
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

void JobQueue::save() const {
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
    save();
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
        save();
        emit changed(index);
        return true;
    }
    if (job.state == State::Running && index == running_ && process_ && !cancel_requested_) {
        cancel_requested_ = true;
        process_->terminate();
        // A worker that ignores the polite request is killed.
        QPointer<QProcess> p = process_;
        QTimer::singleShot(1500, this, [p] {
            if (p && p->state() != QProcess::NotRunning) p->kill();
        });
        return true;
    }
    return false;
}

void JobQueue::start_next() {
    if (running_ >= 0) return;
    for (std::size_t i = 0; i < jobs_.size(); ++i) {
        Job& job = jobs_[i];
        if (job.state != State::Queued) continue;
        job.state = State::Running;
        ++job.attempts;
        job.out.clear();
        job.err.clear();
        running_ = static_cast<int>(i);
        cancel_requested_ = false;
        process_ = new QProcess(this);
        connect(process_, &QProcess::finished, this, &JobQueue::on_process_finished);
        connect(process_, &QProcess::errorOccurred, this, &JobQueue::on_process_error);
        process_->setProgram(program_);
        process_->setArguments(job.args);
        save();
        emit changed(running_);
        process_->start();
        return;
    }
}

void JobQueue::on_process_finished(int exit_code, QProcess::ExitStatus status) {
    if (running_ < 0 || !process_) return;
    const int index = running_;
    Job& job = jobs_[static_cast<std::size_t>(index)];
    job.out = clip(process_->readAllStandardOutput());
    job.err = clip(process_->readAllStandardError());
    job.exit_code = status == QProcess::NormalExit ? exit_code : -1;
    if (status == QProcess::CrashExit && !cancel_requested_) job.err += "\nthe worker process crashed";
    job.state = cancel_requested_ ? State::Cancelled : (job.exit_code == 0 ? State::Done : State::Failed);
    const bool cancelled = cancel_requested_;
    process_->deleteLater();
    process_ = nullptr;
    running_ = -1;
    cancel_requested_ = false;
    save();
    emit changed(index);
    if (!cancelled) emit finished(index);
    start_next();
}

void JobQueue::on_process_error(QProcess::ProcessError error) {
    if (error != QProcess::FailedToStart || running_ < 0 || !process_) return;
    const int index = running_;
    Job& job = jobs_[static_cast<std::size_t>(index)];
    job.state = State::Failed;
    job.exit_code = -1;
    job.err = "cannot start the worker program " + program_ + ": " + process_->errorString();
    process_->deleteLater();
    process_ = nullptr;
    running_ = -1;
    save();
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
