// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>

#include <QSize>
#include <QString>

#include "gui/viewport.hpp"

namespace ohs::gui {

struct VideoOptions {
    double movement_seconds = 60.0;  ///< span of movement time the video shows, from the scene's start
    double video_seconds = 10.0;     ///< length of the video
    int fps = 30;
    QSize size{1280, 720};           ///< rounded down to even numbers, as H.264 needs
};

/// The ffmpeg program: $OHS_FFMPEG, else `ffmpeg` on PATH. Empty when there is none.
QString find_ffmpeg();

/// Renders the viewport at evenly spaced movement times and encodes the frames with ffmpeg (H.264 for .mp4;
/// ffmpeg picks the codec for other extensions, so .webm and .gif work too). The scene's time is put back
/// afterwards. `progress(frame, frames)` may return false to stop. Returns an empty string on success, else
/// the reason (no ffmpeg, an encoder error, a stop).
QString export_video(Viewport& viewport, const QString& path, const VideoOptions& options,
                     const std::function<bool(int, int)>& progress = {});

}  // namespace ohs::gui
