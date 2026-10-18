// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>

#include <CLI/CLI.hpp>

namespace ohs::cli {

/// Adds `lesson list|show|check` to `app` (see add_dynamics_commands).
void add_lesson_commands(CLI::App& app, const bool& json, std::function<int()>& chosen);

}  // namespace ohs::cli
