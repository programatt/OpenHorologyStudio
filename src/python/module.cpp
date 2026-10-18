// SPDX-License-Identifier: Apache-2.0
// The `ohs._native` extension: runs the command line in-process. The Python package builds its
// typed API on top of this one function, so Python, the CLI and any other front end share one
// implementation.
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "cli/cli_run.hpp"

namespace nb = nanobind;

NB_MODULE(_native, m) {
    m.doc() = "Open Horology Studio native core: run ohs commands in-process.";
    m.def(
        "run",
        [](const std::vector<std::string>& args) {
            const auto r = ohs::cli::run_captured(args);
            return std::make_tuple(r.exit_code, r.out, r.err);
        },
        nb::arg("args"), nb::call_guard<nb::gil_scoped_release>(),
        "Run `ohs <args>` and return (exit_code, stdout, stderr).");
    m.def("version", [] { return std::string(OHS_VERSION); }, "The OHS release, e.g. '1.0.0'.");
}
