// SPDX-License-Identifier: Apache-2.0
#include "cli/daemon.hpp"

#include <poll.h>

#include <csignal>
#include <iostream>
#include <sstream>
#include <thread>

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

// A stream buffer that sends what is written to it as frames on the connection, a line at a time, so a client sees a
// long command's output while it runs.
class FrameBuf : public std::streambuf {
public:
    FrameBuf(int fd, char tag) : fd_(fd), tag_(tag) {}
    ~FrameBuf() override { flush(); }

protected:
    int_type overflow(int_type c) override {
        if (c != traits_type::eof()) {
            pending_.push_back(static_cast<char>(c));
            if (c == '\n' || pending_.size() > 4096) flush();
        }
        return c;
    }
    std::streamsize xsputn(const char* s, std::streamsize n) override {
        pending_.append(s, static_cast<std::size_t>(n));
        if (pending_.find('\n') != std::string::npos || pending_.size() > 4096) flush();
        return n;
    }
    int sync() override {
        flush();
        return 0;
    }

private:
    void flush() {
        if (pending_.empty()) return;
        daemon::put_frame(fd_, tag_, pending_);
        pending_.clear();
    }
    int fd_;
    char tag_;
    std::string pending_;
};

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
        daemon::put_frame(fd, 'e', message);
        daemon::put_frame(fd, 'x', "2");
        _exit(0);
    }
    // A client that goes away (the window cancelling a job, a script killed) is the end of this command: the child watches
    // the connection, which the client never writes to again, and kills itself when it closes.
    std::thread([fd] {
        pollfd p{fd, POLLIN, 0};
        for (;;) {
            if (::poll(&p, 1, -1) < 0) continue;
            char byte;
            if (::read(fd, &byte, 1) <= 0) ::kill(::getpid(), SIGKILL);
        }
    }).detach();
    // Progress of an import, as it happens.
    ImportHooks hooks;
    hooks.progress = [fd](std::size_t done, std::size_t total) {
        daemon::put_frame(fd, 'p', "part " + std::to_string(done) + " of " + std::to_string(total) + "\n");
    };
    set_import_hooks(hooks);
    int code = 2;
    {
        FrameBuf out(fd, 'o'), err(fd, 'e');
        std::ostream out_stream(&out), err_stream(&err);
        code = run_streaming(args, out_stream, err_stream);
        out_stream.flush();
        err_stream.flush();
    }
    daemon::put_frame(fd, 'x', std::to_string(code));
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
