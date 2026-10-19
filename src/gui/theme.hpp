// SPDX-License-Identifier: Apache-2.0
#pragma once

class QApplication;
class QLabel;
class QWidget;

namespace ohs::gui {

/// The application's look: Qt's Fusion style (the same on every platform) with a graphite palette and a
/// brass accent, or a warm light palette when the desktop is light. OHS_THEME=dark or OHS_THEME=light
/// overrides the choice. Call once, after the QApplication is made and before the first window.
void apply_theme(QApplication& app);

/// True when the active palette is the dark one.
bool dark_theme(const QWidget& widget);

/// Styles a label as secondary text: the muted colour and a slightly smaller font. Returns the label.
QLabel* make_hint(QLabel* label);

/// Styles a label as a small section heading. Returns the label.
QLabel* make_heading(QLabel* label);

}  // namespace ohs::gui
