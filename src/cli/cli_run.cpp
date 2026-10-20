// SPDX-License-Identifier: Apache-2.0
#include "cli/cli_run.hpp"

#include <iostream>
#include <mutex>
#include <sstream>

namespace ohs::cli {

namespace {
std::mutex hooks_mutex;
ImportHooks hooks;
}  // namespace

void set_import_hooks(ImportHooks value) {
    std::lock_guard<std::mutex> lock(hooks_mutex);
    hooks = std::move(value);
}

ImportHooks import_hooks() {
    std::lock_guard<std::mutex> lock(hooks_mutex);
    return hooks;
}

int run_streaming(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    static std::mutex one_at_a_time;
    std::lock_guard<std::mutex> lock(one_at_a_time);

    std::vector<std::string> storage;
    storage.reserve(args.size() + 1);
    storage.push_back("ohs");
    for (const std::string& a : args) storage.push_back(a);
    std::vector<char*> argv;
    for (std::string& s : storage) argv.push_back(s.data());
    argv.push_back(nullptr);

    std::streambuf* old_out = std::cout.rdbuf(out.rdbuf());
    std::streambuf* old_err = std::cerr.rdbuf(err.rdbuf());
    int code = 2;
    try {
        code = run(static_cast<int>(storage.size()), argv.data());
    } catch (const std::exception& e) {
        err << "internal error: " << e.what() << '\n';
        code = 2;
    }
    std::cout.rdbuf(old_out);
    std::cerr.rdbuf(old_err);
    return code;
}

Captured run_captured(const std::vector<std::string>& args) {
    std::ostringstream out, err;
    const int code = run_streaming(args, out, err);
    return {code, out.str(), err.str()};
}

}  // namespace ohs::cli
