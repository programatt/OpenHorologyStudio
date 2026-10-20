// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <ostream>
#include <string>
#include <vector>

namespace ohs::cli {

/// The whole `ohs` program as a function: returns the process exit code (0 ok, 1 a check failed,
/// 2 bad input). Output goes to std::cout and std::cerr.
int run(int argc, char** argv);

struct Captured {
    int exit_code{};
    std::string out;   ///< what the command wrote to standard output
    std::string err;   ///< and to standard error
};

/// Runs `ohs <args>` in this process with std::cout and std::cerr captured. One run at a time
/// (it swaps the global stream buffers), so the Python module serialises calls.
Captured run_captured(const std::vector<std::string>& args);

/// The same, writing to `out` and `err` as the command goes (a daemon streams them to its client) and returning the exit code.
int run_streaming(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

/// What an application running `ohs import` in this process can watch and stop: called after each part's
/// features are extracted (from any thread), and a flag that makes the import stop between parts.
struct ImportHooks {
    std::function<void(std::size_t done, std::size_t total)> progress;
    const std::atomic<bool>* cancel{nullptr};
};
/// Sets the hooks the next imports use (an empty value clears them). Thread safe.
void set_import_hooks(ImportHooks hooks);
ImportHooks import_hooks();

}  // namespace ohs::cli
