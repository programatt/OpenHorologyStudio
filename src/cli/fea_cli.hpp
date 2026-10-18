// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>

#include <CLI/CLI.hpp>

namespace ohs::cli {

/// Adds the structural commands (materials, fea, thermal, verify physics) to `app`. As with the
/// dynamics commands, a chosen command is returned through `chosen`.
void add_fea_commands(CLI::App& app, const bool& json, std::function<int()>& chosen);

}  // namespace ohs::cli
