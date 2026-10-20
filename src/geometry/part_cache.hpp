// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "geometry/step_io.hpp"

namespace ohs::geometry {

/// The cache directory of a project: `<project>.ohscache` beside the project file. It holds what is slow to
/// get from the CAD file (the parts as OCCT's binary BRep, the display mesh) in forms that load in a
/// fraction of the time. A missing, stale or unreadable cache is never an error: callers fall back to the CAD file.
std::filesystem::path cache_dir(const std::filesystem::path& project);

/// Identifies the state of a CAD file cheaply: its size and modification time and a hash of its first and
/// last megabyte. A cache is valid only while this string is unchanged. Empty when the file cannot be read.
std::string cad_fingerprint(const std::filesystem::path& cad_file);

/// Reads the parts of `cad_file`. With a non-empty `project` the parts come from the project's cache when it
/// was made from this file as it is now (same size, time and content hash), and otherwise from the file itself,
/// rewriting the cache when `update_cache` is true. Pass false for a file the project was not imported from (an
/// explicit --step file): it may use a cache that matches it but never replaces the project's cache.
/// With an empty `project` the file is read and no cache is touched.
std::vector<ImportedPart> load_parts(const std::filesystem::path& cad_file, const std::filesystem::path& project,
                                     bool update_cache = true);

/// True when `cad_file` is the CAD file `project` was imported from (`source_geometry` beside the project), so
/// that the project's cache may be written from it.
bool is_project_source(const std::filesystem::path& cad_file, const std::filesystem::path& project,
                       const std::string& source_geometry);

/// Writes the parts cache for `project` from parts already in memory (the import has them). Failures are
/// ignored: the cache is an optimisation.
void write_parts_cache(const std::filesystem::path& project, const std::filesystem::path& cad_file,
                       const std::vector<ImportedPart>& parts);

/// Writes `data` to `path` through a temporary file and a rename, so a reader sees the old or the new file,
/// never half of one. Returns false on failure.
bool write_file_atomic(const std::filesystem::path& path, const std::string& data);

}  // namespace ohs::geometry
