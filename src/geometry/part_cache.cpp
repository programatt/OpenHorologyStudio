// SPDX-License-Identifier: Apache-2.0
#include "geometry/part_cache.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <system_error>

#include <BRep_Builder.hxx>
#include <BinTools.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>

#include <spdlog/spdlog.h>

#include "geometry/cad_io.hpp"

namespace fs = std::filesystem;

namespace ohs::geometry {

namespace {

constexpr int kSchema = 1;
constexpr std::uintmax_t kHashSpan = 1u << 20;

std::uint64_t fnv(std::uint64_t h, const char* data, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

fs::path parts_path(const fs::path& project) { return cache_dir(project) / "parts.brep"; }
fs::path manifest_path(const fs::path& project) { return cache_dir(project) / "manifest.txt"; }

struct Manifest {
    std::string fingerprint;
    std::uintmax_t brep_size{};
    std::vector<std::string> names;
};

std::string format_manifest(const Manifest& m) {
    std::ostringstream out;
    out << "ohs-cache " << kSchema << '\n' << "occt " << OCC_VERSION_COMPLETE << '\n' << "fingerprint " << m.fingerprint << '\n'
        << "brep_size " << m.brep_size << '\n' << "names " << m.names.size() << '\n';
    for (const std::string& n : m.names) out << n << '\n';
    return out.str();
}

bool read_manifest(const fs::path& path, Manifest& m) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    auto field = [&](const char* key, std::string& value) {
        if (!std::getline(in, line)) return false;
        const std::string prefix = std::string(key) + ' ';
        if (line.rfind(prefix, 0) != 0) return false;
        value = line.substr(prefix.size());
        return true;
    };
    std::string schema, occt, size, count;
    if (!field("ohs-cache", schema) || schema != std::to_string(kSchema)) return false;
    if (!field("occt", occt) || occt != OCC_VERSION_COMPLETE) return false;
    if (!field("fingerprint", m.fingerprint) || !field("brep_size", size) || !field("names", count)) return false;
    try {
        m.brep_size = std::stoull(size);
        const std::size_t n = std::stoull(count);
        m.names.clear();
        for (std::size_t i = 0; i < n; ++i) {
            if (!std::getline(in, line)) return false;
            m.names.push_back(line);
        }
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

bool read_cache(const fs::path& cad_file, const fs::path& project, std::vector<ImportedPart>& parts) {
    Manifest m;
    if (!read_manifest(manifest_path(project), m)) return false;
    const std::string now = cad_fingerprint(cad_file);
    if (now.empty() || now != m.fingerprint) return false;
    std::error_code ec;
    if (fs::file_size(parts_path(project), ec) != m.brep_size || ec) return false;
    try {
        TopoDS_Shape compound;
        if (!BinTools::Read(compound, parts_path(project).string().c_str()) || compound.IsNull()) return false;
        std::vector<ImportedPart> out;
        for (TopoDS_Iterator it(compound); it.More(); it.Next()) out.push_back({{}, it.Value()});
        if (out.size() != m.names.size()) return false;
        for (std::size_t i = 0; i < out.size(); ++i) out[i].name = m.names[i];
        parts = std::move(out);
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

}  // namespace

fs::path cache_dir(const fs::path& project) {
    // A job the desktop app queues runs on a snapshot of the project (`name.job<N>.ext`); it shares the project's cache.
    static const std::regex snapshot(R"((.*)\.job[0-9]+(\.[^.]*)?)");
    std::string name = project.filename().string();
    std::smatch parts;
    if (std::regex_match(name, parts, snapshot)) name = parts[1].str() + parts[2].str();
    fs::path dir = project.parent_path() / name;
    dir += ".ohscache";
    return dir;
}

std::string cad_fingerprint(const fs::path& cad_file) {
    std::error_code ec;
    const std::uintmax_t size = fs::file_size(cad_file, ec);
    if (ec) return {};
    const auto mtime = fs::last_write_time(cad_file, ec);
    if (ec) return {};
    std::ifstream in(cad_file, std::ios::binary);
    if (!in) return {};
    std::uint64_t h = 14695981039346656037ULL;
    std::vector<char> buffer(kHashSpan);
    in.read(buffer.data(), static_cast<std::streamsize>(std::min<std::uintmax_t>(size, kHashSpan)));
    h = fnv(h, buffer.data(), static_cast<std::size_t>(in.gcount()));
    if (size > kHashSpan) {
        in.clear();
        in.seekg(static_cast<std::streamoff>(size - std::min<std::uintmax_t>(size - kHashSpan, kHashSpan)));
        in.read(buffer.data(), static_cast<std::streamsize>(kHashSpan));
        h = fnv(h, buffer.data(), static_cast<std::size_t>(in.gcount()));
    }
    std::ostringstream out;
    out << size << ' ' << mtime.time_since_epoch().count() << ' ' << std::hex << h;
    return out.str();
}

bool write_file_atomic(const fs::path& path, const std::string& data) {
    fs::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) return false;
        out.write(data.data(), static_cast<std::streamsize>(data.size()));
        if (!out) return false;
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

void write_parts_cache(const fs::path& project, const fs::path& cad_file, const std::vector<ImportedPart>& parts) {
    try {
        Manifest m;
        m.fingerprint = cad_fingerprint(cad_file);
        if (m.fingerprint.empty() || parts.empty()) return;
        std::error_code ec;
        fs::create_directories(cache_dir(project), ec);
        if (ec) return;

        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        for (const ImportedPart& p : parts) {
            builder.Add(compound, p.shape);
            m.names.push_back(p.name);
        }
        fs::path tmp = parts_path(project);
        tmp += ".tmp";
        if (!BinTools::Write(compound, tmp.string().c_str())) return;
        m.brep_size = fs::file_size(tmp, ec);
        if (ec) return;
        fs::rename(tmp, parts_path(project), ec);
        if (ec) return;
        write_file_atomic(manifest_path(project), format_manifest(m));
    } catch (const std::exception& e) {
        spdlog::debug("could not write the geometry cache: {}", e.what());
    } catch (const Standard_Failure& e) {
        spdlog::debug("could not write the geometry cache: {}", e.GetMessageString());
    }
}

bool is_project_source(const fs::path& cad_file, const fs::path& project, const std::string& source_geometry) {
    std::error_code ec;
    return fs::equivalent(cad_file, project.parent_path() / source_geometry, ec) && !ec;
}

std::vector<ImportedPart> load_parts(const fs::path& cad_file, const fs::path& project, bool update_cache) {
    if (project.empty()) return import_cad(cad_file);
    std::vector<ImportedPart> parts;
    if (read_cache(cad_file, project, parts)) {
        spdlog::debug("read {} parts from {}", parts.size(), cache_dir(project).string());
        return parts;
    }
    parts = import_cad(cad_file);
    if (update_cache) write_parts_cache(project, cad_file, parts);
    return parts;
}

}  // namespace ohs::geometry
