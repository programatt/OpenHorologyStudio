// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "geometry/features.hpp"
#include "manufacturing/dfm.hpp"
#include "manufacturing/tolerance.hpp"
#include "model/model.hpp"
#include "report/document.hpp"

namespace ohs::reports {

struct ReportOptions {
    std::string step_file;            ///< CAD geometry for DFM, drawings and collisions; empty skips those sections
    geometry::Axis up = geometry::Axis::Z;
    bool timekeeping = true;
    bool tolerance = true;
    bool dfm = true;
    bool structural = true;
    bool collisions = true;

    std::vector<std::string> processes = {"mill", "wheel-cutting"};
    std::vector<manufacturing::ProcessRules> extra_rules;
    std::vector<std::string> plugins;   ///< DFM plugin libraries whose rules run next to the process rules
    manufacturing::ToleranceConfig tolerance_config;
    std::optional<dynamics::Calibre> calibre;
    double beat_rate_vph = 0;         ///< overrides the project's beat rate

    // Acceptance criteria for the pass/fail marks
    double target_power_reserve_h = 40;
    double min_safety_factor = 2.0;
    double max_failing_fraction = 0.02;
};

/// Runs the analyses the options ask for and writes them up. Each section is a pass, warning or
/// fail against the criteria; sections that need geometry are skipped with a note when no STEP
/// file is given. Throws ohs::Error only for unusable input (no rig); problems inside one section
/// are reported in that section.
report::Document build_report(const model::Movement& movement, const ReportOptions& options);

}  // namespace ohs::reports
