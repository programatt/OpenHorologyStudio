// SPDX-License-Identifier: Apache-2.0
#include "gui/theme.hpp"

#include <QApplication>
#include <QLabel>
#include <QPalette>
#include <QStyleFactory>

namespace ohs::gui {

namespace {

QPalette dark_palette() {
    const QColor window(36, 38, 43), base(27, 29, 33), alt(33, 35, 40), text(230, 227, 220), button(46, 49, 55);
    const QColor brass(201, 163, 76), muted(150, 148, 142), disabled(105, 104, 100);
    QPalette p;
    p.setColor(QPalette::Window, window);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, alt);
    p.setColor(QPalette::ToolTipBase, QColor(52, 55, 62));
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::PlaceholderText, muted);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, button);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::BrightText, QColor(255, 120, 100));
    p.setColor(QPalette::Light, QColor(64, 67, 74));
    p.setColor(QPalette::Midlight, QColor(54, 57, 63));
    p.setColor(QPalette::Mid, QColor(30, 32, 36));
    p.setColor(QPalette::Dark, QColor(20, 21, 24));
    p.setColor(QPalette::Shadow, QColor(10, 10, 12));
    p.setColor(QPalette::Highlight, brass);
    p.setColor(QPalette::HighlightedText, QColor(24, 22, 18));
    p.setColor(QPalette::Link, QColor(222, 189, 110));
    p.setColor(QPalette::LinkVisited, QColor(190, 160, 100));
    for (auto role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText})
        p.setColor(QPalette::Disabled, role, disabled);
    p.setColor(QPalette::Disabled, QPalette::Highlight, QColor(80, 78, 72));
    return p;
}

QPalette light_palette() {
    const QColor window(240, 238, 233), base(252, 251, 249), text(34, 32, 28), button(232, 229, 222);
    const QColor brass(168, 128, 44), muted(118, 114, 106), disabled(160, 156, 148);
    QPalette p;
    p.setColor(QPalette::Window, window);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, QColor(245, 243, 239));
    p.setColor(QPalette::ToolTipBase, QColor(255, 252, 240));
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::PlaceholderText, muted);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, button);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::BrightText, QColor(190, 40, 30));
    p.setColor(QPalette::Highlight, brass);
    p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    p.setColor(QPalette::Link, QColor(140, 100, 20));
    for (auto role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText})
        p.setColor(QPalette::Disabled, role, disabled);
    p.setColor(QPalette::Disabled, QPalette::Highlight, QColor(200, 196, 188));  // a disabled slider is not filled in brass
    return p;
}

}  // namespace

void apply_theme(QApplication& app) {
    // Decided from the desktop's palette before Fusion replaces it.
    const QByteArray forced = qgetenv("OHS_THEME");
    const bool dark = forced == "dark" || (forced != "light" && QApplication::palette().window().color().lightness() < 128);
    if (QStyle* fusion = QStyleFactory::create("Fusion")) QApplication::setStyle(fusion);
    QApplication::setPalette(dark ? dark_palette() : light_palette());
    app.setStyleSheet(QStringLiteral(
        "QToolBar { border: none; spacing: 2px; padding: 2px 4px; }"
        "QToolBar QToolButton { padding: 3px 6px; }"
        "QDockWidget::title { padding: 5px 8px; }"
        "QGroupBox { font-weight: 600; margin-top: 1.4ex; padding-top: 4px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 4px; }"
        "QGroupBox QWidget { font-weight: normal; }"
        "QTabWidget::pane { top: -1px; }"
        "QStatusBar QLabel { padding: 0 6px; }"));
}

bool dark_theme(const QWidget& widget) { return widget.palette().window().color().lightness() < 128; }

QLabel* make_hint(QLabel* label) {
    label->setWordWrap(true);
    QPalette p = label->palette();
    p.setColor(QPalette::WindowText, p.color(QPalette::PlaceholderText));
    label->setPalette(p);
    QFont f = label->font();
    f.setPointSizeF(f.pointSizeF() * 0.92);
    label->setFont(f);
    return label;
}

QLabel* make_heading(QLabel* label) {
    QFont f = label->font();
    f.setWeight(QFont::DemiBold);
    label->setFont(f);
    return label;
}

}  // namespace ohs::gui
