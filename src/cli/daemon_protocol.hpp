// SPDX-License-Identifier: Apache-2.0
// The wire format between `ohs-client` (or `ohs` itself) and `ohs serve`, over a Unix domain socket. Header only and free of
// OpenCASCADE, so the thin client links nothing heavy. Lengths are 32-bit, native byte order (both ends are on one machine).
//
//   request : u32 argc, argc x (u32 length, bytes), u32 length, working directory
//   reply   : frames, each u8 tag, u32 length, bytes: 'o' standard output, 'e' standard error, 'p' a progress line
//             ("part 37 of 166"), and last 'x' with the exit code in decimal. They arrive while the command runs. A reply that
//             ends without 'x' means the command's process died (a crash, or it was killed); closing the connection
//             before then cancels the command: the daemon's child watches it and exits.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace ohs::cli::daemon {

struct Reply {
    int exit_code{};
    std::string out, err;
    bool crashed{false};  ///< the daemon's process for the command ended without an exit code
};

/// Called for each frame as it arrives: the tag ('o', 'e' or 'p') and the text.
using FrameSink = std::function<void(char tag, const std::string& text)>;

/// $OHS_SOCKET, else $XDG_RUNTIME_DIR/ohs.sock, else /tmp/ohs-<uid>.sock.
inline std::string default_socket_path() {
    if (const char* s = std::getenv("OHS_SOCKET"); s && *s) return s;
#ifndef _WIN32
    if (const char* d = std::getenv("XDG_RUNTIME_DIR"); d && *d) return std::string(d) + "/ohs.sock";
    return "/tmp/ohs-" + std::to_string(static_cast<unsigned long>(getuid())) + ".sock";
#else
    return {};
#endif
}

#ifndef _WIN32

inline bool write_all(int fd, const void* data, std::size_t n) {
    const char* p = static_cast<const char*>(data);
    while (n > 0) {
        const ssize_t w = ::write(fd, p, n);
        if (w <= 0) return false;
        p += w;
        n -= static_cast<std::size_t>(w);
    }
    return true;
}

inline bool read_all(int fd, void* data, std::size_t n) {
    char* p = static_cast<char*>(data);
    while (n > 0) {
        const ssize_t r = ::read(fd, p, n);
        if (r <= 0) return false;
        p += r;
        n -= static_cast<std::size_t>(r);
    }
    return true;
}

inline bool put_u32(int fd, std::uint32_t v) { return write_all(fd, &v, sizeof v); }
inline bool put_string(int fd, const std::string& s) { return put_u32(fd, static_cast<std::uint32_t>(s.size())) && write_all(fd, s.data(), s.size()); }
inline bool get_u32(int fd, std::uint32_t& v) { return read_all(fd, &v, sizeof v); }
inline bool get_string(int fd, std::string& s) {
    std::uint32_t n = 0;
    if (!get_u32(fd, n) || n > (1u << 30)) return false;
    s.resize(n);
    return n == 0 || read_all(fd, s.data(), n);
}
inline bool put_frame(int fd, char tag, const std::string& text) { return write_all(fd, &tag, 1) && put_string(fd, text); }

/// Opens a connection to the daemon and sends the request; -1 when nobody is listening.
inline int connect_and_send(const std::vector<std::string>& args, const std::string& path, const std::string& cwd) {
    if (path.empty() || path.size() >= sizeof(sockaddr_un::sun_path)) return -1;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    path.copy(addr.sun_path, path.size());
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
        ::close(fd);
        return -1;
    }
    bool ok = put_u32(fd, static_cast<std::uint32_t>(args.size()));
    for (const std::string& a : args) ok = ok && put_string(fd, a);
    ok = ok && put_string(fd, cwd);
    if (!ok) {
        ::close(fd);
        return -1;
    }
    return fd;
}

/// Connects to the daemon and runs `args` there (the command line after `ohs`), handing each frame to `sink` as it
/// arrives. nullopt when nobody is listening; a Reply with `crashed` when the command's process died.
inline std::optional<Reply> run_remote(const std::vector<std::string>& args, const std::string& path, const std::string& cwd,
                                       const FrameSink& sink = {}) {
    const int fd = connect_and_send(args, path, cwd);
    if (fd < 0) return std::nullopt;
    Reply reply;
    reply.crashed = true;
    for (;;) {
        char tag = 0;
        std::string text;
        if (!read_all(fd, &tag, 1)) break;
        if (!get_string(fd, text)) break;
        if (tag == 'x') {  // the exit code, in decimal
            reply.exit_code = std::atoi(text.c_str());
            reply.crashed = false;
            break;
        }
        if (sink) sink(tag, text);
        if (tag == 'o') reply.out += text;
        else if (tag == 'e') reply.err += text;
    }
    ::close(fd);
    return reply;
}

#else

inline std::optional<Reply> run_remote(const std::vector<std::string>&, const std::string&, const std::string&, const FrameSink& = {}) { return std::nullopt; }

#endif

}  // namespace ohs::cli::daemon
