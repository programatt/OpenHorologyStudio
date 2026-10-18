// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>

#include <CLI/CLI.hpp>

namespace ohs::cli {

/// Adds gear, draw, tolerance, dfm, collisions and report to `app` (see add_dynamics_commands).
void add_manufacturing_commands(CLI::App& app, const bool& json, std::function<int()>& chosen);

}  // namespace ohs::cli
