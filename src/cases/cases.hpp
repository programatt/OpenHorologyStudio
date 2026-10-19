// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "fea/analysis.hpp"
#include "fea/fea.hpp"
#include "gear/gear.hpp"

namespace ohs::cases {

// ---- tooth bending ----------------------------------------------------------------------------------

struct ToothCaseInput {
    gear::GearSpec spec;
    double face_width{1.0};          ///< mm
    double tangential_load_n{1.0};   ///< at the tooth tip, perpendicular to the tooth axis
    fea::Material material;
    double size_factor{0.14};        ///< element size at the root in modules
    int teeth_in_sector{3};
};

struct ToothCaseResult {
    double lewis_mpa{};
    double fe_max_principal_mpa{};
    double fe_max_von_mises_mpa{};
    double concentration{};     ///< FE peak over the Lewis stress
    double safety_factor{};     ///< material strength over the FE peak; 0 when the strength is unknown
    double tip_deflection_mm{};
    std::size_t nodes{};
    std::size_t dofs{};
    fea::Model model;
    fea::StaticResult solution;
};

/// A sector of the wheel is clamped at its rim and loaded at the tip of the middle tooth; the
/// FE root stress is reported next to the Lewis hand calculation.
ToothCaseResult run_tooth_case(const ToothCaseInput& input);

// ---- a part under shock, and its vibration modes -------------------------------------------------------

/// How a part is turned into a tetrahedral mesh.
enum class Mesher {
    Section,  ///< the mid-height section meshed in 2D and extruded: fast, exact for prismatic parts
    Netgen,   ///< Netgen meshes the B-rep, so steps, pockets and bosses through the thickness are kept
};

struct PartCaseInput {
    TopoDS_Shape shape;            ///< with the dial plane normal to Z
    double element_size{0.2};
    int layers{2};                 ///< Section mesher only
    Mesher mesher{Mesher::Section};
    fea::Material material;
    double shock_g{5000};          ///< acceleration along -Z (end shake) in g, for the static case
    bool in_plane_shock{false};    ///< apply it along -X instead
    int modes{0};                  ///< >0 also computes this many natural frequencies
    std::size_t max_dofs{250000};  ///< refuse meshes larger than this (a direct solver needs memory for the fill)
    /// Only inner outlines that look like a pivot or screw hole are clamped: round (circularity
    /// 4 pi A / P^2 at least this) and no wider than max_clamp_diameter. Windows, slots and spoke
    /// openings stay free, as they are in the real part.
    double max_clamp_diameter{4.0};
    double min_clamp_circularity{0.85};
};

struct PartCaseResult {
    std::size_t nodes{};
    std::size_t dofs{};
    int clamped_holes{};  ///< round holes clamped
    int free_openings{};  ///< other inner outlines (windows, slots), left free
    double mass_g{};
    double max_displacement_mm{};
    double max_von_mises_mpa{};
    double safety_factor{};
    std::vector<double> frequencies_hz;
    fea::Model model;
    fea::StaticResult solution;
};

/// Meshes the part with quadratic tetrahedra (by default from its mid-height section, extruded; or with
/// Netgen), clamps every round hole (pivots and screws) found in the mid-height section and applies the
/// shock as a body force.
PartCaseResult run_part_case(const PartCaseInput& input);

// ---- the physics verification suite ----------------------------------------------------------------------

struct Check {
    std::string name;
    std::string category;   ///< "analytical", "convergence" or "benchmark"
    double expected{};
    double actual{};
    double tolerance{};     ///< relative
    bool passed{};
    std::string note;
};

/// Re-runs the solvers against known answers. The fast set takes a few seconds; `full` adds the
/// finer meshes, convergence rates, contact, press fit and tooth-root benchmarks.
std::vector<Check> run_physics_checks(bool full);

}  // namespace ohs::cases
