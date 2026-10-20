// SPDX-License-Identifier: Apache-2.0
// ohs-client: runs an `ohs` command in the daemon (`ohs serve`) when one is listening, and otherwise hands over to `ohs`
// itself. It links no OpenCASCADE, VTK or MFEM, so it starts in about a millisecond where `ohs` loads 143 libraries
// (64 ms): for scripts of many small commands.
#include <unistd.h>

#include <climits>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "cli/daemon_protocol.hpp"

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    char cwd[PATH_MAX];
    const std::string where = getcwd(cwd, sizeof cwd) ? cwd : "";
    if (!std::getenv("OHS_NO_DAEMON")) {
        if (const auto reply = ohs::cli::daemon::run_remote(args, ohs::cli::daemon::default_socket_path(), where)) {
            std::fwrite(reply->out.data(), 1, reply->out.size(), stdout);
            std::fwrite(reply->err.data(), 1, reply->err.size(), stderr);
            return reply->exit_code;
        }
    }
    // No daemon: the real program, beside this one or on the path.
    char exe[PATH_MAX];
    const ssize_t length = readlink("/proc/self/exe", exe, sizeof exe - 1);  // where this program really is, if it was linked to
    std::string self = length > 0 ? std::string(exe, static_cast<std::size_t>(length)) : std::string(argv[0]);
    const auto slash = self.rfind('/');
    const std::string beside = slash == std::string::npos ? "" : self.substr(0, slash + 1) + "ohs";
    std::vector<char*> exec_args;
    std::string program = "ohs";
    exec_args.push_back(program.data());
    for (int i = 1; i < argc; ++i) exec_args.push_back(argv[i]);
    exec_args.push_back(nullptr);
    if (!beside.empty()) execv(beside.c_str(), exec_args.data());
    execvp("ohs", exec_args.data());
    std::perror("ohs-client: cannot run ohs");
    return 2;
}
