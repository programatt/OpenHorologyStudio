// SPDX-License-Identifier: Apache-2.0
#include <sys/wait.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using Catch::Approx;

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

TEST_CASE("ohs timekeeping commands end to end") {
    // Generating the STEP movement takes seconds, so every section shares one project.
    static const fs::path dir = [] {
        const fs::path d = fs::temp_directory_path() / "ohs_cli_m2_test";
        fs::remove_all(d);
        fs::create_directories(d);
        return d;
    }();
    const std::string step = (dir / "m.step").string();
    const std::string proj = (dir / "m.hsproj").string();
    if (!fs::exists(proj)) {
        REQUIRE(run("dev gen-movement -o " + step, dir).exit_code == 0);
        REQUIRE(run("import " + step + " -o " + proj + " --beat-rate 25200", dir).exit_code == 0);
    }

    SECTION("torque chain") {
        const auto r = run("--json torque " + proj, dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["speed_ratio"].get<double>() == Approx(6720));
        CHECK(j["meshes"] == 4);
        CHECK(j["stages"].size() == 5);
        CHECK(j["power_reserve_full_wind_h"].get<double>() == Approx(50.0));
        CHECK(j["escape_torque_nm"].get<double>() == Approx(1.1e-3 * std::pow(0.9, 4) / 6720));
        CHECK(run("torque " + proj, dir).output.find("N·mm") != std::string::npos);
    }

    SECTION("simulate: short run, detailed run, run-down, events") {
        const std::string events = (dir / "events.csv").string();
        const auto r = run("--json simulate " + proj + " --duration 60s --events " + events, dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["result"]["amplitude_deg"].get<double>() > 240);
        CHECK(j["result"]["amplitude_deg"].get<double>() < 300);
        CHECK(std::abs(j["result"]["rate_s_per_day"].get<double>()) < 5);
        CHECK(slurp(events).find("impulse_start") != std::string::npos);

        const auto d = run("--json simulate " + proj + " --level detailed --duration 25s", dir);
        REQUIRE(d.exit_code == 0);
        CHECK(std::abs(nlohmann::json::parse(d.output)["result"]["amplitude_deg"].get<double>() -
                       j["result"]["amplitude_deg"].get<double>()) < 15);

        const std::string out = (dir / "run.json").string();
        const auto rd = run("--json simulate " + proj + " --duration 48h -o " + out, dir);
        REQUIRE(rd.exit_code == 0);
        const auto k = nlohmann::json::parse(slurp(out));
        CHECK(k["wind_down"].size() > 10);
        CHECK(k["wind_down"].front()["amplitude_deg"].get<double>() > k["wind_down"].back()["amplitude_deg"].get<double>());

        CHECK(run("simulate " + proj + " --duration soon", dir).exit_code == 2);
        CHECK(run("simulate " + proj + " --position sideways", dir).exit_code == 2);
        CHECK(run("simulate " + proj + " --level quantum", dir).exit_code == 2);
    }

    SECTION("virtual timegrapher and timekeeping summary") {
        const auto t = run("--json timegrapher " + proj + " --duration 30s --jitter-us 30 --seed 3", dir);
        REQUIRE(t.exit_code == 0);
        const auto j = nlohmann::json::parse(t.output);
        CHECK(j["amplitude_deg"].get<double>() > 240);
        CHECK(j["beats"].get<int>() > 150);
        const auto again = run("--json timegrapher " + proj + " --duration 30s --jitter-us 30 --seed 3", dir);
        CHECK(again.output == t.output);
        CHECK(run("timegrapher " + proj + " --duration 20s", dir).output.find("tick") != std::string::npos);

        const auto k = run("--json timekeeping " + proj, dir);
        REQUIRE(k.exit_code == 0);
        const auto kj = nlohmann::json::parse(k.output);
        CHECK(kj["power_reserve_h"].get<double>() > 40);
        CHECK(kj["positions"].size() == 6);
        CHECK(kj["energy_per_beat_j"]["efficiency"].get<double>() > 0.5);
    }

    SECTION("what-if study") {
        std::ofstream(dir / "study.toml") << "[study]\nname = 'mod'\n[[change]]\nparameter = 'hairspring_length_mm'\nscale = 1.002\n";
        const auto r = run("--json whatif " + proj + " " + (dir / "study.toml").string(), dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["causes"].size() == 1);
        bool found = false;
        for (const auto& m : j["metrics"]) {
            if (m["name"] == "rate, dial up, half wind") {
                found = true;
                CHECK(m["change"].get<double>() == Approx((std::sqrt(1 / 1.002) - 1) * 86400).epsilon(0.03));
            }
        }
        CHECK(found);
        std::ofstream(dir / "bad.toml") << "[study]\nname = 'nothing'\n";
        CHECK(run("whatif " + proj + " " + (dir / "bad.toml").string(), dir).exit_code == 2);
    }

    SECTION("gate: detailed model agrees with the reduced model, timegrapher trend is reproduced") {
        const auto v = run("--json verify levels " + proj, dir);
        REQUIRE(v.exit_code == 0);
        const auto j = nlohmann::json::parse(v.output);
        CHECK(j["ok"] == true);
        CHECK(std::abs(j["amplitude_diff_percent"].get<double>()) < 2.0);
        CHECK(run("verify levels " + proj + " --amplitude-tolerance 0.0001 --rate-tolerance 0.0001", dir).exit_code == 1);

        // Bench: a movement with more friction than the reference. Calibrating on its readings
        // brings the simulated amplitude trend in line with the measured one.
        std::ofstream(dir / "bench.csv") << "hours,position,amplitude_deg,rate_s_per_day\n"
                                           "0,dial_up,262,2.0\n24,dial_up,240,1.5\n48,dial_up,214,0.5\n";
        const std::string cal = (dir / "calibre.toml").string();
        const auto before = run("--json bench compare " + proj + " " + (dir / "bench.csv").string(), dir);
        REQUIRE(before.exit_code == 0);
        const auto c = run("--json bench calibrate " + proj + " " + (dir / "bench.csv").string() + " -o " + cal, dir);
        REQUIRE(c.exit_code == 0);
        const auto cj = nlohmann::json::parse(c.output);
        CHECK(cj["rms_amplitude_deg"].get<double>() < nlohmann::json::parse(before.output)["rms_amplitude_deg"].get<double>());
        CHECK(cj["rms_amplitude_deg"].get<double>() < 4.0);
        CHECK(cj["amplitude_slope_simulated_deg_per_h"].get<double>() < 0);
        CHECK(fs::exists(cal));
        const auto torque = run("--json torque " + proj + " --calibre " + cal, dir);
        CHECK(torque.exit_code == 0);
        CHECK(run("bench compare " + proj + " " + (dir / "missing.csv").string(), dir).exit_code == 2);
    }

    SECTION("calibre init") {
        const std::string file = (dir / "c.toml").string();
        REQUIRE(run("calibre init -o " + file, dir).exit_code == 0);
        CHECK(slurp(file).find("balance_inertia_mgcm2") != std::string::npos);
        CHECK(run("torque " + proj + " --calibre " + file, dir).exit_code == 0);
        std::ofstream(dir / "broken.toml") << "[calibre]\nbogus = 1\n";
        CHECK(run("torque " + proj + " --calibre " + (dir / "broken.toml").string(), dir).exit_code == 2);
    }
}
