// SPDX-License-Identifier: Apache-2.0
#include "cli/daemon.hpp"

#include <csignal>
#include <iostream>

#include <spdlog/spdlog.h>

#include "cli/cli_run.hpp"
#include "cli/daemon_protocol.hpp"
#include "core/error.hpp"
#include "geometry/part_cache.hpp"
#include "model/rig_io.hpp"

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/wait.h>
#endif

namespace ohs::cli {

#ifdef _WIN32

int serve(const std::string&, const std::vector<std::string>&) {
    throw Error("ohs serve needs Unix domain sockets and fork: not available on Windows");
}

#else

namespace {

// One request, in a child forked after accept: it inherits what the daemon has loaded (the geometry kept in memory),
// runs the command like any `ohs` call with its output captured, and exits. A crash in the command (the OM10 tooth mesh
// once segfaulted) takes the child down, not the daemon: the client sees a broken conversation and runs the command itself.
void serve_one(int fd) {
    std::uint32_t argc = 0;
    std::vector<std::string> args;
    std::string cwd;
    if (!daemon::get_u32(fd, argc) || argc > 100000) _exit(2);
    for (std::uint32_t i = 0; i < argc; ++i) {
        args.emplace_back();
        if (!daemon::get_string(fd, args.back())) _exit(2);
    }
    if (!daemon::get_string(fd, cwd)) _exit(2);
    if (!cwd.empty() && ::chdir(cwd.c_str()) != 0) {
        const std::string message = "error: cannot enter " + cwd + "\n";
        daemon::put_u32(fd, 2);
        daemon::put_string(fd, "");
        daemon::put_string(fd, message);
        _exit(0);
    }
    const Captured result = run_captured(args);
    daemon::put_u32(fd, static_cast<std::uint32_t>(result.exit_code));
    daemon::put_string(fd, result.out);
    daemon::put_string(fd, result.err);
    ::close(fd);
    _exit(0);
}

}  // namespace

int serve(const std::string& socket_path, const std::vector<std::string>& warm) {
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) throw Error("the socket path is too long: " + socket_path);
    // Geometry of the projects named is read once, here, before any request: the children share it copy on write, so
    // a command on those projects reads neither the CAD file nor the cache. Reading makes no threads, so forking is safe.
    geometry::set_parts_memo(true);
    for (const std::string& project : warm) {
        const model::Movement m = model::load_rig(project);
        const std::filesystem::path step = std::filesystem::path(project).parent_path() / m.source_geometry;
        geometry::load_parts(step, project);
        spdlog::warn("warm: {} ({} parts)", project, m.components.size());
    }

    const int listener = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listener < 0) throw Error("cannot make a socket");
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    socket_path.copy(addr.sun_path, socket_path.size());
    // A socket left by a daemon that is gone; one that answers is somebody else's.
    if (daemon::run_remote({"--version"}, socket_path, "")) throw Error("a daemon is already listening on " + socket_path);
    ::unlink(socket_path.c_str());
    const mode_t old = ::umask(0177);  // only this user may talk to it
    const int bound = ::bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof addr);
    ::umask(old);
    if (bound != 0 || ::listen(listener, 16) != 0) {
        ::close(listener);
        throw Error("cannot listen on " + socket_path);
    }
    std::signal(SIGCHLD, SIG_IGN);  // finished requests are not waited for
    std::signal(SIGPIPE, SIG_IGN);  // a client that went away
    static volatile std::sig_atomic_t stop = 0;
    struct sigaction on_stop {};
    on_stop.sa_handler = [](int) { stop = 1; };
    sigaction(SIGTERM, &on_stop, nullptr);
    sigaction(SIGINT, &on_stop, nullptr);
    std::cout << "ohs serve: listening on " << socket_path << std::endl;

    while (!stop) {
        const int fd = ::accept(listener, nullptr, nullptr);
        if (fd < 0) continue;  // interrupted by a signal, or a failed accept
        const pid_t pid = ::fork();
        if (pid == 0) {
            ::close(listener);
            serve_one(fd);
        }
        ::close(fd);
    }
    ::close(listener);
    ::unlink(socket_path.c_str());
    return 0;
}

#endif

}  // namespace ohs::cli
