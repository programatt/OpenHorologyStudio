// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "fea/fea.hpp"

namespace ohs::fea {

// CalculiX is GPL-2: it is only ever run as a separate program. This adapter writes its input
// deck and reads its result file; nothing of it is linked into OHS.

/// Input deck for a linear static analysis (C3D4 or C3D10 elements, one material, fixed
/// displacements as *BOUNDARY, `load` as *CLOAD point forces).
std::string write_calculix_input(const Model& model, const Vec& load, const std::string& title = "ohs");

/// Displacements from the nodal DISP block of a CalculiX .frd text result (3 per node).
Vec parse_calculix_displacements(const std::string& frd_text, std::size_t node_count);

/// Path of the `ccx` executable found on PATH or in $OHS_CCX, if any.
std::optional<std::filesystem::path> find_calculix();

struct CrossCheck {
    bool available{false};
    double max_displacement_diff{};   ///< relative to the largest displacement
    double ohs_max_displacement{};
    double ccx_max_displacement{};
    std::string note;
};

/// Runs the model in CalculiX in `work_dir` and compares displacements with OHS's solution.
CrossCheck run_calculix_crosscheck(const Model& model, const Vec& load, const StaticResult& ohs_result,
                                   const std::filesystem::path& work_dir);

}  // namespace ohs::fea
