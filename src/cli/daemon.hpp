// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

namespace ohs::cli {

/// `ohs serve`: listens on a Unix domain socket and runs each request (an `ohs` command line and a working directory) in
/// a forked child, so a crash in one command cannot take the daemon down and the child starts with the daemon's loaded
/// libraries and warm geometry. `warm` names projects whose geometry is read before the first request. Blocks until
/// SIGTERM or SIGINT. Throws ohs::Error when the socket cannot be made, or on Windows.
int serve(const std::string& socket_path, const std::vector<std::string>& warm);

}  // namespace ohs::cli
