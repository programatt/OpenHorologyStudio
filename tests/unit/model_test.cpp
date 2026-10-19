// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "model/rig_io.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;

TEST_CASE("role names round-trip") {
    for (auto role : {model::Role::Unknown, model::Role::Plate, model::Role::Barrel, model::Role::EscapeWheel,
                      model::Role::PalletFork, model::Role::Screw, model::Role::Rotor, model::Role::DateRing,
                      model::Role::Stem, model::Role::Spring, model::Role::Lever, model::Role::Bearing}) {
        REQUIRE(model::role_from_string(model::to_string(role)) == role);
    }
    // Every role from the first to the last has its own name, and none of the four new ones moves with a mobile.
    for (int r = static_cast<int>(model::Role::Unknown); r <= static_cast<int>(model::kLastRole); ++r) {
        const auto role = static_cast<model::Role>(r);
        CHECK((model::to_string(role) != "unknown" || role == model::Role::Unknown));
    }
    for (auto role : {model::Role::Stem, model::Role::Spring, model::Role::Lever, model::Role::Bearing}) {
        CHECK_FALSE(model::is_rotating_role(role));
        CHECK_FALSE(model::is_toothed_role(role));
    }
    REQUIRE_FALSE(model::role_from_string("sprocket").has_value());
}

TEST_CASE("rig TOML round-trips a full movement") {
    rigging::RigOptions opt;
    opt.beat_rate_vph = 25200;
    const auto original = rigging::auto_rig("test", test::synthetic_train(), opt);
    REQUIRE_FALSE(original.meshes.empty());

    const auto copy = model::from_toml(model::to_toml(original));
    REQUIRE(copy.name == "test");
    REQUIRE(copy.beat_rate_vph == 25200);
    REQUIRE(copy.components.size() == original.components.size());
    REQUIRE(copy.mobiles.size() == original.mobiles.size());
    REQUIRE(copy.meshes.size() == original.meshes.size());
    for (std::size_t i = 0; i < original.components.size(); ++i) {
        const auto& a = original.components[i];
        const auto& b = copy.components[i];
        REQUIRE(a.features.name == b.features.name);
        REQUIRE(a.role == b.role);
        REQUIRE(a.features.teeth == b.features.teeth);
        REQUIRE(a.features.outer_radius == b.features.outer_radius);
        REQUIRE(a.features.fingerprint == b.features.fingerprint);
    }
    // A second serialisation is byte-identical: the file is a stable contract.
    REQUIRE(model::to_toml(copy) == model::to_toml(original));
}

TEST_CASE("malformed rig files raise ohs::Error") {
    using Catch::Matchers::ContainsSubstring;
    REQUIRE_THROWS_AS(model::from_toml("this is = not [ toml"), Error);
    REQUIRE_THROWS_WITH(model::from_toml("schema_version = 99"), ContainsSubstring("schema_version"));
    REQUIRE_THROWS_WITH(model::from_toml("schema_version = 1\n[[component]]\nname='a'\nrole='sprocket'\n"),
                        ContainsSubstring("unknown role"));
    REQUIRE_THROWS_AS(model::load_rig("/nonexistent/file.hsproj"), Error);
}
