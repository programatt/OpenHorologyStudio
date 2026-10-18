// SPDX-License-Identifier: Apache-2.0
#pragma once

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

}  // namespace ohs::cli
