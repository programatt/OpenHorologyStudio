// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include <QImage>
#include <QString>
#include <QTimer>
#include <QWidget>

namespace ohs::gui {

/// One tick or tock of a virtual timegrapher run (the columns of `ohs timegrapher --csv`).
struct ScreenBeat {
    int index{};
    double time_s{};
    bool tock{false};
    double interval_ms{};
    double deviation_ms{};
    double lift_time_ms{};
};

struct ScreenReading {
    double rate_s_per_day{};
    double amplitude_deg{};
    double beat_error_ms{};
};

/// The screen of a generic timegrapher: a grey case with a blue LCD that shows the beat rate, the lift angle, the
/// position, the rate, amplitude and beat error in seven-segment digits, and the two traces of dots, ticks above
/// and tocks below, which the beats of the run draw one after the other. A straight, level trace is a movement
/// that keeps time; a slope is a rate, and a gap between the two traces is beat error.
class TimegrapherScreen : public QWidget {
    Q_OBJECT
public:
    explicit TimegrapherScreen(QWidget* parent = nullptr);

    /// The run to show. `position` is the orientation as `ohs` names it ("dial_up", "crown_left" ...).
    void set_run(std::vector<ScreenBeat> beats, ScreenReading reading, double beat_rate_vph, double lift_deg,
                 const QString& position);
    void clear();
    bool has_run() const { return !beats_.empty(); }

    /// Plays the run from the start; `speed` is the run's seconds per real second.
    void play(double speed = 4.0);
    void stop();
    bool playing() const { return timer_.isActive(); }
    /// The point of the run that is shown, in seconds from its first beat; beats up to there are drawn.
    void set_time(double seconds);
    double time() const { return time_; }
    int beats_shown() const;
    double duration() const { return beats_.empty() ? 0.0 : beats_.back().time_s - beats_.front().time_s; }

    /// The whole device drawn at `size`, with the beats up to `seconds` of the run.
    QImage render(const QSize& size, double seconds) const;

    /// The short code a timegrapher prints for an orientation: DU, DD, CU, CD, CL or CR.
    static QString position_code(const QString& position);

protected:
    void paintEvent(QPaintEvent*) override;
    void mouseDoubleClickEvent(QMouseEvent*) override { play(speed_); }  // run the beats again
    void mousePressEvent(QMouseEvent*) override;                          // the red start/stop button
    QSize sizeHint() const override { return {680, 420}; }
    QSize minimumSizeHint() const override { return {400, 247}; }

private:
    void draw(QPainter& painter, const QSize& size, double seconds) const;

    std::vector<ScreenBeat> beats_;
    ScreenReading reading_;
    double bph_{0.0}, lift_{52.0};
    QString position_;
    double time_{0.0}, speed_{4.0};
    QTimer timer_;
};

}  // namespace ohs::gui
