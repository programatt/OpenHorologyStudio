// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>

#include <CLI/CLI.hpp>

namespace ohs::cli {

/// Adds the timekeeping commands (torque, simulate, timegrapher, timekeeping, whatif, verify,
/// bench, calibre) to `app`. When one of them is chosen, `chosen` is set to a callable that runs
/// it and returns the process exit code. `json` is the global --json flag.
void add_dynamics_commands(CLI::App& app, const bool& json, std::function<int()>& chosen);

}  // namespace ohs::cli
