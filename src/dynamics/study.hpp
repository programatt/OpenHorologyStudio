// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dynamics/calibre.hpp"
#include "dynamics/detailed.hpp"
#include "dynamics/timekeeping.hpp"
#include "dynamics/torque_chain.hpp"

namespace ohs::dynamics {

struct Change {
    enum class Mode { Set, Scale, Add };
    std::string parameter;
    Mode mode{Mode::Scale};
    double value{1.0};
};

/// A what-if: the baseline design and one modified copy of it.
struct StudyDefinition {
    std::string name = "what-if";
    std::vector<Change> changes;
    bool regulate = true;  ///< set the baseline's hairspring so it keeps perfect time dial up, half wound
    bool verify = false;   ///< also run the detailed model on both designs
};

/// TOML:
///   [study]  name = "..."  regulate = true  verify = false
///   [[change]]  parameter = "hairspring_length_mm"  scale = 0.98   # or set = ... / add = ...
StudyDefinition study_from_toml(const std::string& text);
StudyDefinition load_study(const std::string& path);

/// Applies one change; throws ohs::Error for an unknown parameter.
void apply(Calibre& calibre, const Change& change);

struct MetricRow {
    std::string name;
    std::string unit;
    double baseline{};
    double modified{};
    double delta() const { return modified - baseline; }
};

struct StudyResult {
    StudyDefinition definition;
    Calibre baseline;
    Calibre modified;
    Timekeeping baseline_result;
    Timekeeping modified_result;
    std::vector<MetricRow> metrics;
    std::vector<std::string> causes;  ///< one explanation per change, with the numbers of this design
    std::optional<Agreement> baseline_check;  ///< detailed model against level 2, dial up, half wound
    std::optional<Agreement> modified_check;
};

StudyResult run_study(const Calibre& baseline, const TorqueChain& chain, const StudyDefinition& definition);

std::string to_text(const StudyResult& result);

}  // namespace ohs::dynamics
