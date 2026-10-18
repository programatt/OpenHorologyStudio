// SPDX-License-Identifier: Apache-2.0
#include <sys/wait.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace {

struct Result {
    int exit_code;
    std::string output;  ///< stdout
    std::string error;   ///< stderr (sanitizer reports also land here)
};

std::string slurp(const fs::path& path) {
    std::ifstream in(path);
    std::stringstream buf;
    buf << in.rdbuf();
    return buf.str();
}

Result run(const std::string& args, const fs::path& dir) {
    const fs::path out = dir / "out.txt";
    const fs::path err = dir / "err.txt";
    const std::string cmd = std::string("'") + OHS_CLI_PATH + "' " + args + " > '" + out.string() + "' 2> '" +
                            err.string() + "'";
    const int status = std::system(cmd.c_str());
    return {WEXITSTATUS(status), slurp(out), slurp(err)};
}

}  // namespace

TEST_CASE("ohs CLI end to end") {
    const fs::path dir = fs::temp_directory_path() / "ohs_cli_test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const std::string step = (dir / "m.step").string();
    const std::string proj = (dir / "m.hsproj").string();

    SECTION("generate, import, check, kinematics") {
        REQUIRE(run("dev gen-movement -o " + step, dir).exit_code == 0);
        REQUIRE(fs::exists(step));

        const auto imp = run("--json import " + step + " -o " + proj + " --beat-rate 25200", dir);
        REQUIRE(imp.exit_code == 0);
        const auto summary = nlohmann::json::parse(imp.output);
        CHECK(summary["meshes"] == 4);
        CHECK(summary["mobiles"] == 5);

        const auto check = run("--json rig check " + proj, dir);
        CHECK(check.exit_code == 0);
        CHECK(nlohmann::json::parse(check.output)["errors"] == 0);

        const auto kin = run("--json kinematics " + proj + " --expect fourth_wheel=1", dir);
        // mobile names are M<n>, so a wrong name must be an error (exit 2), not a silent pass.
        CHECK(kin.exit_code == 2);

        const auto text = run("kinematics " + proj, dir);
        CHECK(text.exit_code == 0);
        CHECK(text.output.find("escape wheel 15 teeth") != std::string::npos);

        // Find the mobile holding fourth_wheel from the JSON and verify 1 rpm via --expect.
        const auto j = nlohmann::json::parse(run("--json kinematics " + proj, dir).output);
        std::string fourth;
        for (const auto& m : j["mobiles"]) {
            if (std::abs(m["rpm"].get<double>() - 1.0) < 1e-6) fourth = m["name"];
        }
        REQUIRE_FALSE(fourth.empty());
        CHECK(run("kinematics " + proj + " --expect " + fourth + "=1", dir).exit_code == 0);
        CHECK(run("kinematics " + proj + " --expect " + fourth + "=2", dir).exit_code == 1);

        // M1: gear fit in `rig check`, Explain, animation export, manual rig edits.
        CHECK(check.output.find("\"involute\"") == std::string::npos);  // JSON summary stays terse
        const auto text_check = run("rig check " + proj, dir);
        CHECK(text_check.output.find("m=0.15 involute 20") != std::string::npos);

        const auto walk = run("explain " + proj + " --train", dir);
        CHECK(walk.exit_code == 0);
        CHECK(walk.output.find("84:6 = x14") != std::string::npos);
        const auto part = run("--json explain " + proj + " fourth_wheel", dir);
        CHECK(part.exit_code == 0);
        CHECK(nlohmann::json::parse(part.output)["part"]["role"] == "wheel");
        CHECK(run("explain " + proj + " nope", dir).exit_code == 2);

        const std::string glb = (dir / "m.glb").string();
        const auto anim = run("--json animate " + proj + " --step " + step + " -o " + glb + " --duration 2", dir);
        CHECK(anim.exit_code == 0);
        CHECK(nlohmann::json::parse(anim.output)["animated_nodes"] == 14);
        CHECK(fs::file_size(glb) > 100000);

        CHECK(run("rig set " + proj + " fourth_wheel --role screw", dir).exit_code == 0);
        CHECK(nlohmann::json::parse(run("--json rig check " + proj, dir).output)["components"].size() == 15);
        CHECK(run("kinematics " + proj, dir).output.find("warning") != std::string::npos);
        CHECK(run("rig set " + proj + " fourth_wheel --role sprocket", dir).exit_code == 2);
    }

    SECTION("error handling and exit codes") {
        CHECK(run("import /nonexistent.step -o " + proj, dir).exit_code == 2);
        CHECK(run("rig check /nonexistent.hsproj", dir).exit_code == 2);
        CHECK(run("bogus-command", dir).exit_code == 2);
        CHECK(run("--version", dir).exit_code == 0);

        std::ofstream(dir / "bad.step") << "not a step file";
        const auto bad = run("import " + (dir / "bad.step").string() + " -o " + proj, dir);
        CHECK(bad.exit_code == 2);
        CHECK(bad.error.find("error:") != std::string::npos);
    }
    fs::remove_all(dir);
}
