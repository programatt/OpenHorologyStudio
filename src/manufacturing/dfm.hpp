// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "mesh/mesh2d.hpp"
#include "model/model.hpp"

namespace ohs::manufacturing {

enum class Severity { Info, Warning, Error };

const char* to_string(Severity s);

/// What the rules need to know about one part: its section outline (outer ring plus holes), its
/// thickness and, for wheels, the tooth data.
struct PartGeometry {
    std::string name;
    model::Role role{model::Role::Unknown};
    mesh::Region region;
    double thickness{};
    int teeth{};
    double module{};
    double pressure_angle_deg{20};
    double rolling_radius{};  ///< in modules; > 0 marks a cycloidal profile
};

struct Finding {
    std::string process, rule, part;
    Severity severity{Severity::Info};   ///< Info means the check passed
    std::string message;
    std::string fix;                     ///< what to change, empty when nothing is wrong
    double value{};                      ///< the measured quantity
    double limit{};                      ///< the limit it was compared with
    bool has_location{false};
    mesh::P2 location{};
};

/// One manufacturing process: which rules to run and their limits.
struct ProcessRules {
    std::string name, description;
    std::vector<std::string> rules;
    std::map<std::string, double> limits;
    std::map<std::string, std::vector<double>> lists;

    double limit(const std::string& key, double fallback) const;
    const std::vector<double>& list(const std::string& key) const;
};

/// `[process.<name>]` tables; throws ohs::Error on malformed input or unknown rule names.
std::vector<ProcessRules> rule_sets_from_toml(const std::string& text);
std::vector<ProcessRules> load_rule_sets(const std::string& path);
const std::vector<ProcessRules>& builtin_rule_sets();
const ProcessRules& find_process(std::string_view name);
const ProcessRules& find_process(std::string_view name, const std::vector<ProcessRules>& extra);

/// Runs the process's rules on the part. Every rule reports at least one finding, Info when it passes.
std::vector<Finding> run_dfm(const ProcessRules& process, const PartGeometry& part);

/// The smallest concave (internal) corner radius of the outline, 0 for a sharp corner, and where
/// it is. Returns a huge number when there is no concave feature.
double min_internal_radius(const mesh::Region& region, mesh::P2* where = nullptr);

/// The thinnest wall: the shortest distance across material from one boundary point to another.
double min_wall_thickness(const mesh::Region& region, mesh::P2* where = nullptr);

}  // namespace ohs::manufacturing
