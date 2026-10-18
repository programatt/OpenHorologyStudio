// SPDX-License-Identifier: Apache-2.0
#include "plugins/plugin_loader.hpp"

#include <algorithm>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include "core/error.hpp"
#include "ohs/plugin.h"

namespace ohs::plugins {

namespace {

void* open_library(const std::string& path, std::string& why) {
#if defined(_WIN32)
    void* h = reinterpret_cast<void*>(LoadLibraryA(path.c_str()));
    if (!h) why = "LoadLibrary failed with code " + std::to_string(GetLastError());
    return h;
#else
    void* h = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        const char* e = dlerror();
        why = e ? e : "dlopen failed";
    }
    return h;
#endif
}

void* find_symbol(void* handle, const char* name) {
#if defined(_WIN32)
    return reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(handle), name));
#else
    return dlsym(handle, name);
#endif
}

void close_library(void* handle) {
#if defined(_WIN32)
    FreeLibrary(reinterpret_cast<HMODULE>(handle));
#else
    dlclose(handle);
#endif
}

double signed_area(const mesh::Ring& r) {
    double a = 0;
    for (std::size_t i = 0; i < r.size(); ++i) a += r[i].x * r[(i + 1) % r.size()].y - r[(i + 1) % r.size()].x * r[i].y;
    return 0.5 * a;
}

}  // namespace

std::shared_ptr<DfmPlugin> DfmPlugin::load(const std::string& path) {
    std::string why;
    void* handle = open_library(path, why);
    if (!handle) throw Error("cannot load plugin " + path + ": " + why);
    using Entry = const ohs_dfm_rule_set* (*)();
    const auto entry = reinterpret_cast<Entry>(find_symbol(handle, "ohs_plugin_dfm_rule_set"));
    if (!entry) {
        close_library(handle);
        throw Error("plugin " + path + " does not export ohs_plugin_dfm_rule_set: it is not an OHS DFM plugin");
    }
    const ohs_dfm_rule_set* set = entry();
    if (!set || set->struct_size < sizeof(ohs_dfm_rule_set) || !set->check || !set->name) {
        close_library(handle);
        throw Error("plugin " + path + " returned an incomplete rule set");
    }
    if (set->abi_major != OHS_PLUGIN_ABI_MAJOR) {
        const std::string theirs = std::to_string(set->abi_major) + "." + std::to_string(set->abi_minor);
        close_library(handle);
        throw Error("plugin " + path + " was built for plugin ABI " + theirs + ", but this OHS speaks " + std::to_string(OHS_PLUGIN_ABI_MAJOR) + "." + std::to_string(OHS_PLUGIN_ABI_MINOR));
    }
    if (set->abi_minor > OHS_PLUGIN_ABI_MINOR) {
        const std::string theirs = std::to_string(set->abi_major) + "." + std::to_string(set->abi_minor);
        close_library(handle);
        throw Error("plugin " + path + " needs plugin ABI " + theirs + ", newer than this OHS's " + std::to_string(OHS_PLUGIN_ABI_MAJOR) + "." + std::to_string(OHS_PLUGIN_ABI_MINOR) + "; update OHS");
    }
    std::shared_ptr<DfmPlugin> p(new DfmPlugin());
    p->handle_ = handle;
    p->set_ = set;
    p->name_ = set->name;
    p->description_ = set->description ? set->description : "";
    p->version_ = set->version ? set->version : "";
    p->path_ = path;
    return p;
}

DfmPlugin::~DfmPlugin() {
    if (handle_) close_library(handle_);
}

std::vector<manufacturing::Finding> DfmPlugin::check(const manufacturing::PartGeometry& part) const {
    const auto* set = static_cast<const ohs_dfm_rule_set*>(set_);
    std::vector<int32_t> counts;
    std::vector<double> xy;
    for (std::size_t k = 0; k < part.region.rings.size(); ++k) {
        mesh::Ring r = part.region.rings[k];
        if ((signed_area(r) > 0) != (k == 0)) std::reverse(r.begin(), r.end());  // outer counter-clockwise, holes clockwise
        counts.push_back(static_cast<int32_t>(r.size()));
        for (const auto& p : r) {
            xy.push_back(p.x);
            xy.push_back(p.y);
        }
    }
    const std::string role = std::string(model::to_string(part.role));
    ohs_part c{};
    c.struct_size = sizeof(ohs_part);
    c.name = part.name.c_str();
    c.role = role.c_str();
    c.thickness_mm = part.thickness;
    c.teeth = part.teeth;
    c.module_mm = part.module;
    c.pressure_angle_deg = part.pressure_angle_deg;
    c.ring_count = static_cast<int32_t>(counts.size());
    c.ring_point_counts = counts.data();
    c.xy = xy.data();

    auto error_finding = [&](const std::string& message) {
        manufacturing::Finding f;
        f.process = name_;
        f.rule = "plugin";
        f.part = part.name;
        f.severity = manufacturing::Severity::Error;
        f.message = message;
        f.fix = "report this to the plugin's author";
        return std::vector<manufacturing::Finding>{f};
    };

    std::vector<ohs_finding> buffer(32);
    int32_t n = 0;
    for (int attempt = 0; attempt < 2; ++attempt) {
        for (auto& f : buffer) f = ohs_finding{};
        n = set->check(&c, buffer.data(), static_cast<int32_t>(buffer.size()), set->user);
        if (n < 0) return error_finding("the plugin '" + name_ + "' reported an internal error (" + std::to_string(n) + ")");
        if (static_cast<std::size_t>(n) <= buffer.size()) break;
        buffer.assign(static_cast<std::size_t>(n), ohs_finding{});
    }
    std::vector<manufacturing::Finding> out;
    for (int32_t i = 0; i < n && static_cast<std::size_t>(i) < buffer.size(); ++i) {
        const ohs_finding& f = buffer[static_cast<std::size_t>(i)];
        manufacturing::Finding g;
        g.process = name_;
        g.rule = f.rule ? f.rule : "plugin";
        g.part = part.name;
        g.severity = f.severity >= OHS_SEVERITY_ERROR ? manufacturing::Severity::Error : f.severity == OHS_SEVERITY_WARNING ? manufacturing::Severity::Warning : manufacturing::Severity::Info;
        g.message = f.message ? f.message : "";
        g.fix = f.fix ? f.fix : "";
        g.value = f.value;
        g.limit = f.limit;
        g.has_location = f.has_location != 0;
        g.location = {f.x, f.y};
        out.push_back(std::move(g));
    }
    return out;
}

}  // namespace ohs::plugins
