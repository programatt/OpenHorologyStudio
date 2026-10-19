// SPDX-License-Identifier: Apache-2.0
#include "gui/video.hpp"

#include <algorithm>
#include <cmath>

#include <QFileInfo>
#include <QImage>
#include <QProcess>
#include <QStandardPaths>

namespace ohs::gui {

QString find_ffmpeg() {
    const QString env = qEnvironmentVariable("OHS_FFMPEG");
    if (!env.isEmpty()) return QFileInfo::exists(env) ? env : QString();
    return QStandardPaths::findExecutable("ffmpeg");
}

QString export_video(Viewport& viewport, const QString& path, const VideoOptions& o, const std::function<bool(int, int)>& progress) {
    Scene* scene = viewport.scene();
    if (!scene) return QObject::tr("Open a project first.");
    if (!scene->can_animate()) return QObject::tr("The movement has no train or no beat rate, so there is nothing to animate.");
    if (o.fps < 1 || o.video_seconds <= 0 || o.movement_seconds <= 0) return QObject::tr("The frame rate and both durations must be positive.");
    const QString ffmpeg = find_ffmpeg();
    if (ffmpeg.isEmpty()) return QObject::tr("ffmpeg was not found: install it, or set OHS_FFMPEG to its location.");

    const QSize size(std::max(2, o.size.width() & ~1), std::max(2, o.size.height() & ~1));
    const int frames = std::max(2, static_cast<int>(std::lround(o.video_seconds * o.fps)));
    QStringList args{"-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgra", "-video_size",
                     QString("%1x%2").arg(size.width()).arg(size.height()), "-framerate", QString::number(o.fps), "-i", "-", "-an"};
    if (path.endsWith(".mp4", Qt::CaseInsensitive)) args << "-c:v" << "libx264" << "-pix_fmt" << "yuv420p" << "-movflags" << "+faststart";
    args << path;

    QProcess encoder;
    encoder.start(ffmpeg, args);
    if (!encoder.waitForStarted(5000)) return QObject::tr("Cannot start ffmpeg: %1").arg(encoder.errorString());

    const double saved_time = scene->time();
    QString error;
    for (int i = 0; i < frames && error.isEmpty(); ++i) {
        scene->set_time(o.movement_seconds * i / (frames - 1));
        const QImage frame = viewport.render(size);  // ARGB32 is BGRA in memory on little-endian machines
        const qint64 bytes = frame.sizeInBytes();
        if (encoder.write(reinterpret_cast<const char*>(frame.constBits()), bytes) != bytes) error = QObject::tr("ffmpeg stopped accepting frames");
        while (error.isEmpty() && encoder.bytesToWrite() > 0) {
            if (!encoder.waitForBytesWritten(10000)) error = QObject::tr("ffmpeg stopped accepting frames");
        }
        if (error.isEmpty() && progress && !progress(i + 1, frames)) error = QObject::tr("Stopped.");
    }
    scene->set_time(saved_time);
    encoder.closeWriteChannel();
    if (!encoder.waitForFinished(120000)) {
        encoder.kill();
        return error.isEmpty() ? QObject::tr("ffmpeg did not finish") : error;
    }
    if (!error.isEmpty()) return error;
    if (encoder.exitStatus() != QProcess::NormalExit || encoder.exitCode() != 0) {
        return QObject::tr("ffmpeg failed: %1").arg(QString::fromUtf8(encoder.readAllStandardError()).trimmed());
    }
    return {};
}

}  // namespace ohs::gui
