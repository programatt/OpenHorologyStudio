// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "model/model.hpp"

namespace ohs::rigging {

/// Maps part names matching a case-insensitive regex to a role (e.g. an OM10 preset).
struct RolePreset {
    std::string pattern;
    model::Role role{model::Role::Unknown};
};

struct RigOptions {
    double coaxial_tolerance = 0.05;  ///< mm, max centre offset for parts of one mobile
    // Mesh plausibility, in modules, with the module taken from the centre distance
    // (m = 2C / (z1 + z2)): both gears must have a sensible addendum and dedendum, and the tip of
    // each must clear the root of the other by roughly a quarter module.
    double min_addendum = 0.75;
    double max_addendum = 1.5;
    double min_dedendum = 0.9;
    double max_dedendum = 1.8;
    double min_clearance = -0.1;
    double max_clearance = 0.6;
    double ideal_clearance = 0.25;
    /// A flank fits an involute when its rms deviation is below this many modules.
    double involute_rms_modules = 0.02;
    /// A flank that is not an involute is cycloidal when its cycloid fit is within this many modules
    /// and the fitted pitch circle is within `cycloid_pitch_tolerance` of the one the module gives.
    double cycloid_rms_modules = 0.02;
    double cycloid_pitch_tolerance = 0.04;
    int min_teeth = 6;                ///< toothed parts need at least this many detected teeth
    double beat_rate_vph = 0.0;
    std::vector<RolePreset> presets;
};

struct Classification {
    model::Role role{model::Role::Unknown};
    double confidence{};
    std::string source;
};

Classification classify(const model::PartFeatures& part, const RigOptions& options);

/// Classifies parts, groups mobiles by coaxiality and infers gear meshes.
model::Movement auto_rig(std::string movement_name, std::vector<model::PartFeatures> parts,
                         const RigOptions& options);

/// Recomputes mobiles, meshes and gear fits from the components' roles and features.
void infer_structure(model::Movement& movement, const RigOptions& options);

/// Overrides a component's role (source "user", confidence 1) and re-infers the structure.
/// Throws ohs::Error if there is no component called `name`.
void set_role(model::Movement& movement, const std::string& name, model::Role role, const RigOptions& options);

/// Takes the mesh between two parts out of the going train (`winding` true: it is the winding work, or just not a link
/// from the barrel to the escape wheel) or puts it back. Throws when the two parts are not meshed.
/// Takes away a mesh that was declared by hand (`add_manual_mesh`) and fits the gears again without it, as a wrong
/// declared mesh also spoils the module of the gears it joins. A mesh the geometry found would come back at once, so
/// that one is refused: leave it out of the train with set_mesh_winding instead. Throws when the parts are not meshed.
void remove_manual_mesh(model::Movement& movement, const std::string& a, const std::string& b, const RigOptions& options);

void set_mesh_winding(model::Movement& movement, const std::string& a, const std::string& b, bool winding);

/// Declares that two toothed parts mesh although the geometry does not show it (a smooth barrel
/// drum, a simplified model). The module follows from the centre distance and the tooth counts.
/// The mesh is kept whenever the structure is re-inferred. Throws ohs::Error if either part is
/// missing, is not on a rotating mobile, has no teeth, or both are on the same mobile.
void add_manual_mesh(model::Movement& movement, const std::string& a, const std::string& b, const RigOptions& options);

/// What re-running the auto-rig changed.
struct RoleChange {
    std::string part;
    model::Role from{model::Role::Unknown};
    model::Role to{model::Role::Unknown};
    double confidence{};
};

struct RerigReport {
    std::vector<RoleChange> role_changes;
    int mobiles_before{}, mobiles_after{};
    int meshes_before{}, meshes_after{};
};

/// Runs the classification and the structure inference again on a rig that already exists, from the
/// features stored in it. Parts whose role you set (source "user", or a preset) keep it, and so do the
/// meshes you declared; every other role is decided afresh, so parts that were unknown or are newly
/// recognised join their mobiles and meshes without a re-import. With `keep_user` false everything is
/// decided afresh, including your roles and declared meshes.
RerigReport re_rig(model::Movement& movement, const RigOptions& options, bool keep_user = true);

/// A pair of toothed parts on different mobiles that probably mesh although the strict test did not
/// accept them (tip circles a little off, a tooth count not detected exactly). Offered for confirmation:
/// the confidence is at most 0.5.
struct MeshSuggestion {
    int a{};
    int b{};
    double centre_distance{};
    double module{};
    double confidence{};
};

/// The pairs worth declaring as meshes, most likely first. Only pairs in which at least one part
/// meshes with nothing yet are offered, since those are what is left to rig.
std::vector<MeshSuggestion> suggest_meshes(const model::Movement& movement, const RigOptions& options);

/// How the gear chain runs from the barrel to the escape wheel, over mobile indices.
struct TrainChain {
    int barrel{-1};   ///< mobile holding the barrel, or -1
    int escape{-1};   ///< mobile holding the escape wheel, or -1
    bool connected{false};
    std::vector<int> path;          ///< barrel ... escape when connected
    std::vector<int> from_barrel;   ///< otherwise, the longest chain of meshes that starts at the barrel
    std::vector<int> from_escape;   ///< otherwise, the longest chain that ends at the escape wheel
    std::vector<char> fed;          ///< per mobile: 1 when meshes join it to the barrel (including the barrel)
    std::vector<int> unreached;     ///< geared mobiles joined to neither end
};

TrainChain train_chain(const model::Movement& movement);

/// "M3 (centre wheel)": the mobile's name and the name of its first toothed part.
std::string describe_mobile(const model::Movement& movement, int mobile);

/// The chain as one line of text, with "->" between mobiles; says where it breaks.
std::string describe_chain(const model::Movement& movement, const TrainChain& chain);

/// One step of walking the train from the barrel towards the escape wheel.
struct WalkStep {
    /// Mesh: a mesh of the train, `driver` driving `driven`. Gap: the chain stops at `driver`, which meshes with
    /// nothing further. Missing: there is no barrel or no escape wheel at all (`needs_role` says which).
    enum class Kind { Mesh, Gap, Missing } kind{Kind::Mesh};
    int mesh{-1};                    ///< Mesh: index into Movement::meshes
    int driver{-1};                  ///< component; -1 for Missing
    int driven{-1};                  ///< component; -1 for Gap and Missing
    model::Role needs_role{model::Role::Unknown};  ///< Missing: the role to give a part
    std::string text;                ///< one sentence saying what the step is
    std::vector<int> suggestions;    ///< Gap: indices into the suggestions given, those that would join the gap
};

struct TrainWalk {
    std::vector<WalkStep> steps;
    bool complete{false};            ///< the steps reach the escape wheel
};

/// The steps from the barrel along the chain of meshes, in order, ending at the escape wheel (complete) or at
/// the step that is missing. `suggestions` are those of suggest_meshes.
TrainWalk walk_train(const model::Movement& movement, const TrainChain& chain,
                     const std::vector<MeshSuggestion>& suggestions);

/// A way to close the going train: `driver` (a part of another mobile) drives `input`, the toothed part at the loose
/// end of the train that nothing drives. A driver whose teeth were not found has them worked out from the centre
/// distance and the module of `input`, and they are stored with the mesh.
struct SolveCandidate {
    int driver{-1};
    int input{-1};
    int teeth{};            ///< the driver's tooth count, found or estimated
    bool estimated{false};  ///< `teeth` was worked out, not found in the geometry
    double centre{};        ///< centre distance, mm
    double module{};        ///< the module that centre distance and the two tooth counts give
    double residual{};      ///< how far the pair's module and the driver's tip radius are from the gears' own: 0 is exact
    double driver_rate{};   ///< turns per hour of the driver's mobile if it is the driver
};

/// A wheel of the train that turns at the rate of a hand.
struct HandWheel {
    std::string hand;         ///< "seconds", "minute" or "hour"
    int mobile{-1};
    std::string description;  ///< describe_mobile
    double rate{};            ///< turns per hour
};

struct TrainSolution {
    bool connected{false};     ///< the train already runs from the barrel to the escape wheel
    double beat_rate_vph{};
    int input{-1};             ///< the gear nothing drives, or -1
    std::vector<SolveCandidate> candidates;  ///< best first; the barrel first when it fits
    std::vector<HandWheel> hands;            ///< found from the escape wheel and the beat rate along the meshes of the train
    std::vector<std::string> notes;          ///< what could not be worked out, and why
};

/// Works from the escape wheel back along the meshes already found, with the beat rate, to say how fast every wheel
/// of the train turns (so which turn the seconds, minute and hour hands) and, if the train is broken, which part
/// could drive the gear at its loose end. Changes nothing.
TrainSolution solve_train(const model::Movement& movement, double beat_rate_vph, const RigOptions& options);

/// Declares the mesh of a candidate (and stores the estimated tooth count of its driver).
void apply_solution(model::Movement& movement, const SolveCandidate& candidate, const RigOptions& options);

enum class Severity { Info, Warning, Error };

struct Issue {
    Severity severity;
    std::string message;
    std::vector<std::string> parts;  ///< the parts the issue is about (for a viewer to select); may be empty
};

/// Sanity-checks a rig and lists everything a user should confirm or correct.
std::vector<Issue> check_rig(const model::Movement& movement, double low_confidence = 0.6);

}  // namespace ohs::rigging
