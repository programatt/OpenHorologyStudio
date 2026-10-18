// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ohs::fea {

/// A material as the data sheets give it. The solvers work in mm, N, MPa, tonne and s, so
/// `youngs_mpa()` and `density_t_mm3()` convert.
struct Material {
    std::string name;
    std::string description;
    double youngs_gpa = 210;
    double poisson = 0.3;
    double density = 7850;        ///< kg/m3
    double cte = 11.5e-6;         ///< linear thermal expansion, 1/K
    double strength_mpa = 0;      ///< yield or fracture strength, 0 when unknown
    double thermoelastic = 0;     ///< (1/E) dE/dT, 1/K
    std::pair<double, double> temp_range_c{-20, 80};
    std::string source;

    double youngs_mpa() const { return youngs_gpa * 1e3; }
    double density_t_mm3() const { return density * 1e-12; }
    double shear_mpa() const { return youngs_mpa() / (2 * (1 + poisson)); }
};

/// The catalogue shipped in data/materials/materials.toml.
const std::vector<Material>& builtin_materials();

/// Parses `[[material]]` tables; throws ohs::Error on malformed input.
std::vector<Material> materials_from_toml(const std::string& text);
std::vector<Material> load_materials(const std::string& path);

/// Looks a material up by name in `extra` first (project overrides), then the built-in list.
const Material& find_material(std::string_view name);
const Material& find_material(std::string_view name, const std::vector<Material>& extra);

}  // namespace ohs::fea
