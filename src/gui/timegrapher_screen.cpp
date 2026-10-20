// SPDX-License-Identifier: Apache-2.0
#include "gui/timegrapher_screen.hpp"

#include <algorithm>
#include <cmath>

#include <QFontDatabase>
#include <QFontMetricsF>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QRadialGradient>

namespace ohs::gui {

namespace {

// The device is drawn on a canvas of 2000 x 1234 units, laid out after the generic "multifunction time grapher"
// with a character LCD; the widget scales the canvas to fit.
constexpr double kCanvasW = 2000.0, kCanvasH = 1234.0;

const QColor kPaper(236, 238, 244);      // the lit bar across the top of the LCD
const QColor kBarInk(28, 56, 226);       // and the blue of its characters
const QColor kLcdInk(255, 255, 255);     // white on the blue
const QPointF kStartButton(1510, 897);   // the red button
constexpr double kStartRadius = 85;

QRectF centred(const QPointF& c, double w, double h) { return {c.x() - w / 2, c.y() - h / 2, w, h}; }

void label(QPainter& p, const QRectF& area, const QString& text, int pixel, QFont::Weight weight, const QColor& colour,
           Qt::Alignment align = Qt::AlignCenter, int stretch = 100, bool italic = false) {
    QFont f = p.font();
    f.setPixelSize(pixel);
    f.setWeight(weight);
    f.setStretch(stretch);
    f.setItalic(italic);
    while (pixel > 12 && QFontMetricsF(f).horizontalAdvance(text) > area.width()) f.setPixelSize(--pixel);  // fit the width
    p.setFont(f);
    p.setPen(colour);
    p.drawText(area, align, text);
}

// A push button of the case: a ring sunk in the plastic and a domed cap.
void button(QPainter& p, const QPointF& c, double r, const QColor& cap, const QColor& cap_edge) {
    QRadialGradient ring(c + QPointF(-r * 0.2, -r * 0.3), r * 1.5);
    ring.setColorAt(0.0, QColor(214, 216, 222));
    ring.setColorAt(1.0, QColor(120, 123, 130));
    p.setPen(QPen(QColor(105, 108, 114), 3));
    p.setBrush(ring);
    p.drawEllipse(c, r * 1.38, r * 1.38);
    QRadialGradient dome(c + QPointF(-r * 0.3, -r * 0.35), r * 1.3);
    dome.setColorAt(0.0, cap.lighter(135));
    dome.setColorAt(1.0, cap);
    p.setPen(QPen(cap_edge, 3));
    p.setBrush(dome);
    p.drawEllipse(c, r, r);
}

void led(QPainter& p, const QPointF& c, double r, const QColor& on, bool lit) {
    if (lit) {
        QRadialGradient glow(c, r * 2.6);
        glow.setColorAt(0.0, QColor(on.red(), on.green(), on.blue(), 140));
        glow.setColorAt(1.0, QColor(on.red(), on.green(), on.blue(), 0));
        p.setPen(Qt::NoPen);
        p.setBrush(glow);
        p.drawEllipse(c, r * 2.6, r * 2.6);
    }
    QRadialGradient lens(c + QPointF(-r * 0.3, -r * 0.3), r * 1.4);
    lens.setColorAt(0.0, lit ? on.lighter(170) : on.darker(260).lighter(130));
    lens.setColorAt(1.0, lit ? on : on.darker(300));
    p.setPen(QPen(QColor(60, 62, 66), 2));
    p.setBrush(lens);
    p.drawEllipse(c, r, r);
}

QString signed_integer(double v) {
    const long n = std::lround(v);
    return QString("%1%2").arg(n < 0 ? '-' : '+').arg(std::labs(n));
}

}  // namespace

TimegrapherScreen::TimegrapherScreen(QWidget* parent) : QWidget(parent) {
    timer_.setInterval(33);
    connect(&timer_, &QTimer::timeout, this, [this] {
        time_ += 0.033 * speed_;
        if (time_ >= duration()) {
            time_ = duration();
            timer_.stop();
        }
        update();
    });
}

void TimegrapherScreen::set_run(std::vector<ScreenBeat> beats, ScreenReading reading, double beat_rate_vph, double lift_deg,
                                const QString& position) {
    timer_.stop();
    beats_ = std::move(beats);
    reading_ = reading;
    bph_ = beat_rate_vph;
    lift_ = lift_deg;
    position_ = position;
    time_ = 0.0;
    update();
}

void TimegrapherScreen::clear() {
    timer_.stop();
    beats_.clear();
    time_ = 0.0;
    update();
}

void TimegrapherScreen::play(double speed) {
    if (beats_.empty()) return;
    speed_ = speed;
    time_ = 0.0;
    timer_.start();
    update();
}

void TimegrapherScreen::stop() {
    timer_.stop();
    update();
}

void TimegrapherScreen::set_time(double seconds) {
    time_ = std::clamp(seconds, 0.0, duration());
    update();
}

int TimegrapherScreen::beats_shown() const {
    if (beats_.empty()) return 0;
    const double t0 = beats_.front().time_s;
    return static_cast<int>(std::count_if(beats_.begin(), beats_.end(), [&](const ScreenBeat& b) { return b.time_s - t0 <= time_ + 1e-9; }));
}

QString TimegrapherScreen::position_code(const QString& position) {
    if (position == "dial_up") return "DU";
    if (position == "dial_down") return "DD";
    if (position == "crown_up") return "CU";
    if (position == "crown_down") return "CD";
    if (position == "crown_left") return "CL";
    if (position == "crown_right") return "CR";
    return position.left(2).toUpper();
}

void TimegrapherScreen::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    draw(painter, size(), time_);
}

void TimegrapherScreen::mousePressEvent(QMouseEvent* e) {
    // The red start/stop button stops the beats, or runs them again.
    const double s = std::min(width() / kCanvasW, height() / kCanvasH);
    const QPointF at((e->position().x() - (width() - kCanvasW * s) / 2) / s, (e->position().y() - (height() - kCanvasH * s) / 2) / s);
    if (std::hypot(at.x() - kStartButton.x(), at.y() - kStartButton.y()) <= kStartRadius * 1.4 && has_run()) {
        if (playing()) stop();
        else play(speed_);
    }
}

QImage TimegrapherScreen::render(const QSize& size, double seconds) const {
    QImage image(size, QImage::Format_ARGB32);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    draw(painter, size, seconds);
    return image;
}

void TimegrapherScreen::draw(QPainter& p, const QSize& size, double seconds) const {
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);
    const double s = std::min(size.width() / kCanvasW, size.height() / kCanvasH);
    p.translate((size.width() - kCanvasW * s) / 2, (size.height() - kCanvasH * s) / 2);
    p.scale(s, s);

    const bool have = !beats_.empty();
    const double t0 = have ? beats_.front().time_s : 0.0;
    const int shown = have ? static_cast<int>(std::count_if(beats_.begin(), beats_.end(), [&](const ScreenBeat& b) { return b.time_s - t0 <= seconds + 1e-9; })) : 0;
    const bool measuring = shown >= 6;
    const bool running = have && timer_.isActive();

    // The case: a light grey front over a darker base.
    p.setPen(QPen(QColor(110, 110, 108), 3));
    p.setBrush(QColor(150, 150, 148));
    p.drawRoundedRect(QRectF(200, 1030, 1660, 170), 26, 26);
    QLinearGradient plastic(0, 30, 0, 1090);
    plastic.setColorAt(0.0, QColor(196, 196, 193));
    plastic.setColorAt(0.6, QColor(178, 178, 175));
    plastic.setColorAt(1.0, QColor(160, 160, 158));
    p.setPen(QPen(QColor(120, 120, 118), 3));
    p.setBrush(plastic);
    p.drawRoundedRect(QRectF(30, 30, 1955, 1060), 38, 38);
    p.setPen(QPen(QColor(255, 255, 255, 110), 3));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(QRectF(44, 44, 1927, 1032), 30, 30);

    // The names of the four columns of the display, the model and the name of the device.
    const QColor black(18, 18, 24), navy(14, 16, 62);
    label(p, centred({573, 205}, 300, 80), "RATE", 56, QFont::Normal, black, Qt::AlignCenter, 85);
    label(p, centred({878, 200}, 270, 80), "AMPLITUDE", 56, QFont::Normal, black, Qt::AlignCenter, 85);
    label(p, centred({1145, 200}, 250, 80), "BEAT ERROR", 56, QFont::Normal, black, Qt::AlignCenter, 85);
    label(p, centred({1440, 190}, 380, 80), "PARAMETERS", 56, QFont::Normal, black, Qt::AlignCenter, 85);
    label(p, centred({237, 530}, 260, 90), "No.1000", 66, QFont::Normal, navy, Qt::AlignCenter, 95);
    label(p, QRectF(268, 840, 1040, 130), "MULTIFUNCTION TIME GRAPHER", 84, QFont::Bold, navy, Qt::AlignLeft | Qt::AlignVCenter, 92);

    // The window of the display, sunk in the case.
    QLinearGradient well(0, 255, 0, 745);
    well.setColorAt(0.0, QColor(112, 114, 122));
    well.setColorAt(0.5, QColor(168, 169, 174));
    well.setColorAt(1.0, QColor(214, 214, 216));
    p.setPen(QPen(QColor(95, 97, 104), 3));
    p.setBrush(well);
    p.drawRoundedRect(QRectF(355, 255, 1255, 490), 44, 44);
    const QRectF lcd(405, 288, 1160, 412);
    p.setPen(QPen(QColor(24, 36, 150), 6));
    QRadialGradient glass(lcd.center(), lcd.width() * 0.62);
    glass.setColorAt(0.0, QColor(26, 58, 244));
    glass.setColorAt(0.75, QColor(30, 66, 255));
    glass.setColorAt(1.0, QColor(70, 120, 255));
    p.setBrush(glass);
    p.drawRoundedRect(lcd, 14, 14);
    p.save();
    p.setClipRect(lcd.adjusted(6, 6, -6, -6));

    // The top line of the display is a lit bar of characters: the rate, the amplitude, the beat error and a parameter.
    const QRectF bar(448, 312, 1070, 90);
    p.setPen(Qt::NoPen);
    p.setBrush(kPaper);
    p.drawRect(bar);
    p.setPen(QPen(QColor(150, 160, 200, 60), 1));
    for (double x = bar.left(); x < bar.right(); x += 6.5) p.drawLine(QPointF(x, bar.top()), QPointF(x, bar.bottom()));
    for (double y = bar.top(); y < bar.bottom(); y += 6.5) p.drawLine(QPointF(bar.left(), y), QPointF(bar.right(), y));
    QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    mono.setStyleHint(QFont::Monospace);
    const auto text_at = [&](double x, double y, double w, double h, const QString& t, int pixel, const QColor& ink, Qt::Alignment al = Qt::AlignLeft | Qt::AlignVCenter) {
        QFont f = mono;
        f.setPixelSize(pixel);
        f.setWeight(QFont::Bold);
        p.setFont(f);
        p.setPen(ink);
        p.drawText(QRectF(x, y, w, h), al, t);
    };
    if (have) {
        text_at(520, bar.top(), 270, bar.height(), measuring ? QString("%1s/d").arg(signed_integer(reading_.rate_s_per_day)) : QString("---s/d"), 74, kBarInk, Qt::AlignRight | Qt::AlignVCenter);
        text_at(815, bar.top(), 190, bar.height(), measuring ? QString("%1°").arg(std::lround(reading_.amplitude_deg)) : QString("---°"), 74, kBarInk);
        text_at(1030, bar.top(), 245, bar.height(), measuring ? QString("%1ms").arg(reading_.beat_error_ms, 0, 'f', 1) : QString("---ms"), 74, kBarInk);
        text_at(1290, bar.top(), 228, bar.height(), QString("%1°").arg(lift_, 0, 'f', 1), 74, kBarInk);
        // The line under it: the position, the beats counted so far, and the beat rate.
        text_at(620, 408, 120, 56, position_code(position_), 44, kLcdInk, Qt::AlignCenter);
        text_at(1060, 408, 200, 56, QString::number(shown), 44, kLcdInk, Qt::AlignCenter);
        text_at(1330, 408, 190, 56, QString::number(std::lround(bph_)), 44, kLcdInk, Qt::AlignCenter);
    }

    // The traces: two rows of dots across the display, ticks on the upper row and tocks on the lower. A level line
    // is a rate of nothing, a slope is a rate, and a step between the two rows is beat error.
    if (have) {
        constexpr int kPerLine = 70;                 // beats to a line, then it starts again
        constexpr double kDot = 5.0, kPitch = 7.0;   // the dots of the matrix
        const double x0 = 470, span = 1040, y_tick = 536, y_tock = 553, ms_px = 9.0;
        struct Dot { double x{}, y{}; int sweep{-1}; };
        std::vector<std::vector<Dot>> line(2, std::vector<Dot>(kPerLine));
        int seen[2] = {0, 0};
        double ref[2] = {0, 0};
        const double shift = std::clamp(reading_.beat_error_ms * 14.0, -60.0, 60.0);  // the tocks sit this far to the side
        for (int i = 0; i < shown; ++i) {
            const ScreenBeat& b = beats_[static_cast<std::size_t>(i)];
            const int r = b.tock ? 1 : 0;
            const int n = seen[r]++;
            const int k = n % kPerLine;
            if (k == 0) ref[r] = b.deviation_ms;
            const double dy = std::clamp((b.deviation_ms - ref[r]) * ms_px, -48.0, 48.0);
            const double x = x0 + span * k / kPerLine + (r ? shift : 0.0);
            line[static_cast<std::size_t>(r)][static_cast<std::size_t>(k)] = {x, (r ? y_tock : y_tick) + std::round(dy / kPitch) * kPitch, n / kPerLine};
        }
        p.setPen(Qt::NoPen);
        for (int r = 0; r < 2; ++r) {
            const int current = std::max(0, (seen[r] - 1) / kPerLine);
            for (const Dot& d : line[static_cast<std::size_t>(r)]) {
                if (d.sweep < 0) continue;
                p.setBrush(d.sweep == current ? kLcdInk : QColor(255, 255, 255, 110));
                for (double dx = 0; dx < span / kPerLine - 1; dx += kPitch) p.drawRect(QRectF(d.x + dx, d.y, kDot, kDot));
            }
        }
    }
    p.restore();
    // A little light on the glass.
    QLinearGradient sheen(lcd.topLeft(), lcd.topLeft() + QPointF(0, lcd.height() * 0.5));
    sheen.setColorAt(0.0, QColor(255, 255, 255, 18));
    sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    p.setPen(Qt::NoPen);
    p.setBrush(sheen);
    p.drawRoundedRect(QRectF(lcd.left(), lcd.top(), lcd.width(), lcd.height() * 0.5), 14, 14);

    // The buttons down the right, and the signal light between them.
    const QColor grey(176, 184, 202), grey_edge(110, 118, 140);
    button(p, {1800, 395}, 70, grey, grey_edge);
    button(p, {1785, 640}, 66, grey, grey_edge);
    label(p, centred({1913, 395}, 70, 90), QString::fromUtf8("↑"), 80, QFont::Bold, black);
    label(p, centred({1892, 640}, 70, 90), QString::fromUtf8("↓"), 80, QFont::Bold, black);
    label(p, centred({1672, 520}, 130, 70), "sig.", 56, QFont::Normal, QColor(190, 25, 35), Qt::AlignCenter, 100, true);
    led(p, {1744, 520}, 22, QColor(70, 220, 60), measuring);
    label(p, centred({1850, 515}, 190, 80), "value", 60, QFont::Normal, black, Qt::AlignCenter, 90);

    // Start/stop, and the menu button.
    label(p, centred({1495, 790}, 250, 80), "start/stop", 58, QFont::Normal, black, Qt::AlignCenter, 85);
    label(p, centred({1772, 780}, 250, 80), "menu/speaker", 58, QFont::Normal, black, Qt::AlignCenter, 85);
    led(p, {1393, 860}, 20, QColor(235, 30, 30), running);
    button(p, kStartButton, 62, QColor(220, 30, 28), QColor(120, 10, 10));
    button(p, {1780, 880}, 62, grey, grey_edge);
}

}  // namespace ohs::gui
