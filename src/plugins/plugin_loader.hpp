// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "manufacturing/dfm.hpp"

namespace ohs::plugins {

/// A DFM rule set loaded from a shared library that implements include/ohs/plugin.h.
class DfmPlugin {
public:
    /// Loads the library and validates its ABI version. Throws ohs::Error with the reason when the
    /// file cannot be loaded, lacks the entry point, or was written against an incompatible ABI.
    static std::shared_ptr<DfmPlugin> load(const std::string& path);
    ~DfmPlugin();
    DfmPlugin(const DfmPlugin&) = delete;
    DfmPlugin& operator=(const DfmPlugin&) = delete;

    const std::string& name() const { return name_; }
    const std::string& description() const { return description_; }
    const std::string& version() const { return version_; }
    const std::string& path() const { return path_; }

    /// Runs the plugin's rules on a part; findings carry the plugin's name as their process.
    /// A plugin error is reported as an Error finding rather than thrown.
    std::vector<manufacturing::Finding> check(const manufacturing::PartGeometry& part) const;

private:
    DfmPlugin() = default;
    void* handle_{nullptr};
    const void* set_{nullptr};  // const ohs_dfm_rule_set*
    std::string name_, description_, version_, path_;
};

}  // namespace ohs::plugins
