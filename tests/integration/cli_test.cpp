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

#include "dynamics/chrono_adapter.hpp"

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
        CHECK(run("verify levels " + proj + " --engine bullet", dir).exit_code == 2);
        if (ohs::dynamics::chrono_available()) {
            const auto c = run("--json verify levels " + proj + " --engine chrono", dir);
            REQUIRE(c.exit_code == 0);
            const auto cj = nlohmann::json::parse(c.output);
            CHECK(cj["engine"] == "chrono");
            CHECK(std::abs(cj["amplitude_diff_percent"].get<double>()) < 2.0);
        } else {
            CHECK(run("verify levels " + proj + " --engine chrono", dir).exit_code == 2);
        }

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

TEST_CASE("ohs structural commands end to end") {
    static const fs::path dir = [] {
        const fs::path d = fs::temp_directory_path() / "ohs_cli_m3_test";
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

    SECTION("materials and thermal coefficient") {
        const auto m = run("--json materials", dir);
        REQUIRE(m.exit_code == 0);
        const auto j = nlohmann::json::parse(m.output);
        CHECK(j.size() >= 8);
        CHECK(run("materials", dir).output.find("brass_leaded") != std::string::npos);

        const auto t = run("--json thermal --hairspring steel_hardened --balance steel_hardened", dir);
        REQUIRE(t.exit_code == 0);
        CHECK(nlohmann::json::parse(t.output)["rate_s_per_day_per_k"].get<double>() < -8.0);
        const auto good = run("--json thermal", dir);
        CHECK(std::abs(nlohmann::json::parse(good.output)["rate_s_per_day_per_k"].get<double>()) < 1.0);
        CHECK(run("thermal --hairspring unobtainium", dir).exit_code == 2);
    }

    SECTION("gate: the physics verification suite passes and can fail loudly") {
        const auto v = run("--json verify physics", dir);
        REQUIRE(v.exit_code == 0);
        const auto j = nlohmann::json::parse(v.output);
        CHECK(j["ok"] == true);
        CHECK(j["checks"].size() >= 4);
#ifndef OHS_SANITIZED
        // The full set is minutes under a sanitizer; it runs in the normal build.
        const auto full = run("--json verify physics --full", dir);
        REQUIRE(full.exit_code == 0);
        CHECK(nlohmann::json::parse(full.output)["checks"].size() >= 9);
#endif
    }

    SECTION("tooth bending from the torque chain") {
        const auto r = run("--json fea " + proj + " --part third_wheel --vtu " + (dir / "tooth.vtu").string(), dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["teeth"] == 75);
        CHECK(j["module"].get<double>() == Approx(0.15));
        CHECK(j["load_source"].get<std::string>().find("torque of mobile") != std::string::npos);
        // The load is the mobile's torque over the pitch radius: 1.1 N·mm * 0.9^2 / 8 / 8 / 7.5 ... checked loosely.
        CHECK(j["load_n"].get<double>() > 0);
        CHECK(j["concentration_over_lewis"].get<double>() > 1.0);
        CHECK(j["concentration_over_lewis"].get<double>() < 2.4);
        CHECK(slurp(dir / "tooth.vtu").find("von_mises") != std::string::npos);
        // A given load overrides the chain and scales the stress linearly.
        const auto a = nlohmann::json::parse(run("--json fea " + proj + " --part third_wheel --load-n 0.1", dir).output);
        const auto b = nlohmann::json::parse(run("--json fea " + proj + " --part third_wheel --load-n 0.2", dir).output);
        CHECK(b["fe_max_principal_mpa"].get<double>() == Approx(2 * a["fe_max_principal_mpa"].get<double>()).epsilon(1e-6));
        CHECK(run("fea " + proj + " --part third_wheel", dir).output.find("Lewis hand calculation") != std::string::npos);
    }

    SECTION("shock on a part and errors") {
        const auto r = run("--json fea " + proj + " --part escape_wheel --case shock --size 0.3 --g 1000", dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["max_von_mises_mpa"].get<double>() > 0);
        CHECK(j["safety_factor"].get<double>() > 10);
        const auto j2 = nlohmann::json::parse(run("--json fea " + proj + " --part escape_wheel --case shock --size 0.3 --g 2000", dir).output);
        CHECK(j2["max_von_mises_mpa"].get<double>() == Approx(2 * j["max_von_mises_mpa"].get<double>()).epsilon(1e-6));

        CHECK(run("fea " + proj + " --part nope", dir).exit_code == 2);
        CHECK(run("fea " + proj + " --part main_plate", dir).exit_code == 2);                    // no teeth
        CHECK(run("fea " + proj + " --part main_plate --case shock", dir).exit_code == 2);       // too many dofs / no holes
        CHECK(run("fea " + proj + " --part third_wheel --case buckling", dir).exit_code == 2);
        CHECK(run("fea " + proj + " --part third_wheel --material unobtainium", dir).exit_code == 2);
    }
}

TEST_CASE("ohs manufacturing commands end to end") {
    // A single generated wheel keeps the CAD work small: gear -> STEP -> import -> project.
    static const fs::path dir = [] {
        const fs::path d = fs::temp_directory_path() / "ohs_cli_m4_test";
        fs::remove_all(d);
        fs::create_directories(d);
        return d;
    }();
    const std::string step = (dir / "w.step").string();
    const std::string proj = (dir / "w.hsproj").string();
    if (!fs::exists(proj)) {
        REQUIRE(run("--json gear --teeth 20 --module 0.3 --bore 0.8 --thickness 0.4 --name wheel20 --step-out " + step, dir).exit_code == 0);
        REQUIRE(run("import " + step + " -o " + proj + " --beat-rate 25200", dir).exit_code == 0);
    }

    SECTION("gear generator writes STEP, DXF, SVG and PDF that agree with each other") {
        const std::string base = (dir / "g").string();
        const auto r = run("--json gear --teeth 24 --module 0.25 --bore 0.6 --thickness 0.3 --material brass --name wheel24 --dxf " + base + ".dxf --svg " + base + ".svg --pdf " + base + ".pdf",
                           dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["tip_diameter"].get<double>() == Approx(2 * (0.5 * 0.25 * 24 + 0.25)));
        CHECK(j["pitch_diameter"].get<double>() == Approx(6.0));
        CHECK(j["outputs"].size() == 3);
        const std::string dxf = slurp(base + ".dxf");
        CHECK(dxf.find("POLYLINE") != std::string::npos);
        CHECK(dxf.find("tip D 6.500") != std::string::npos);
        CHECK(slurp(base + ".svg").find("pitch D 6.000") != std::string::npos);
        const std::string pdf = slurp(base + ".pdf");
        CHECK(pdf.rfind("%PDF-1.4", 0) == 0);
        CHECK(pdf.find("scale ") != std::string::npos);
        // The cutting file has no annotation.
        REQUIRE(run("gear --teeth 24 --module 0.25 --profile-only --dxf " + base + "-cut.dxf", dir).exit_code == 0);
        CHECK(slurp(base + "-cut.dxf").find("TEXT") == std::string::npos);
        CHECK(run("gear --teeth 24 --module 0.25", dir).exit_code == 2);                       // no output asked for
        CHECK(run("gear --teeth 3 --module 0.25 --dxf " + base + "-x.dxf", dir).exit_code == 2);  // not a gear
    }

    SECTION("part drawings from the project") {
        const std::string svg = (dir / "d.svg").string();
        const auto r = run("--json draw " + proj + " --part wheel20 -o " + svg, dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["width"].get<double>() == Approx(2 * (0.5 * 0.3 * 20 + 0.3)).epsilon(0.01));
        REQUIRE(j["holes"].size() == 1);
        CHECK(j["holes"][0]["diameter"].get<double>() == Approx(1.6).epsilon(0.02));
        CHECK(slurp(svg).find("<svg") != std::string::npos);
        REQUIRE(run("draw " + proj + " --part wheel20 -o " + (dir / "d.dxf").string(), dir).exit_code == 0);
        REQUIRE(run("draw " + proj + " --part wheel20 -o " + (dir / "d.pdf").string(), dir).exit_code == 0);
        CHECK(slurp(dir / "d.pdf").rfind("%PDF-1.4", 0) == 0);
        CHECK(run("draw " + proj + " --part wheel20 -o " + (dir / "d.png").string(), dir).exit_code == 2);
        CHECK(run("draw " + proj + " --part nope -o " + svg, dir).exit_code == 2);
        CHECK(run("draw " + proj + " --part wheel20", dir).exit_code == 2);
    }

    SECTION("dfm: process list, findings, exit codes, user rules") {
        const auto list = run("--json dfm --list", dir);
        REQUIRE(list.exit_code == 0);
        CHECK(nlohmann::json::parse(list.output).size() >= 7);

        const auto ok = run("--json dfm " + proj + " --process wheel-cutting", dir);
        REQUIRE(ok.exit_code == 0);
        const auto j = nlohmann::json::parse(ok.output);
        CHECK(j["errors"] == 0);
        CHECK(j["findings"].size() >= 3);
        // Milling does not apply to a toothed wheel.
        const auto mill = nlohmann::json::parse(run("--json dfm " + proj + " --process mill", dir).output);
        CHECK(mill["findings"][0]["rule"] == "applicability");

        // Your own rules: only a 0.5 module cutter, so a 0.3 module wheel cannot be cut.
        std::ofstream(dir / "rules.toml") << "[process.mine]\nrules = ['module_cutter']\ncutter_modules = [0.5]\nmodule_tolerance = 0.02\n";
        const auto bad = run("dfm " + proj + " --process mine --rules " + (dir / "rules.toml").string(), dir);
        CHECK(bad.exit_code == 1);
        CHECK(bad.output.find("no cutter within") != std::string::npos);
        CHECK(run("dfm " + proj + " --process laser", dir).exit_code == 2);
        CHECK(run("dfm", dir).exit_code == 2);

        // Plugins: the example rule set runs next to the built-in one; an incompatible ABI is refused.
        const auto with_plugin = run("--json dfm " + proj + " --process wheel-cutting --plugin " + std::string(OHS_EXAMPLE_PLUGIN), dir);
        REQUIRE(with_plugin.exit_code == 0);
        bool plugin_finding = false;
        const auto plugin_json = nlohmann::json::parse(with_plugin.output);
        for (const auto& f : plugin_json["findings"]) {
            if (f["process"] == "min-hole-example") {
                plugin_finding = true;
                CHECK(f["severity"] == "ok");  // the 1.6 mm bore is above the smallest drill
                CHECK(f["part"] == "wheel20");
            }
        }
        CHECK(plugin_finding);
        const auto refused = run("dfm " + proj + " --plugin " + std::string(OHS_ABI_V2_PLUGIN), dir);
        CHECK(refused.exit_code == 2);
        CHECK(refused.error.find("built for plugin ABI 2.0") != std::string::npos);
        CHECK(run("dfm " + proj + " --plugin " + std::string(OHS_NO_ENTRY_PLUGIN), dir).exit_code == 2);
        CHECK(run("dfm " + proj + " --plugin /nonexistent.so", dir).exit_code == 2);
        // A plugin that reports an error makes the command exit 1.
        const auto many = run("--json dfm " + proj + " --process wheel-cutting --plugin " + std::string(OHS_MANY_PLUGIN), dir);
        CHECK(many.exit_code == 1);
        CHECK(nlohmann::json::parse(many.output)["errors"].get<int>() > 10);
    }

    SECTION("tolerance and collisions need a train: clear errors, not crashes") {
        CHECK(run("tolerance " + proj, dir).exit_code == 2);   // no gear meshes
        CHECK(run("collisions " + proj, dir).exit_code == 2);  // no barrel
    }

    SECTION("report: html and pdf, failing rig gives exit 1") {
        const std::string prefix = (dir / "rep").string();
        const auto r = run("--json report " + proj + " --no-fea --no-timekeeping --samples 500 -o " + prefix, dir);
        CHECK(r.exit_code == 1);  // a lone wheel has no barrel, so the rig check fails
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["overall"] == "FAIL");
        CHECK(j["outputs"].size() == 2);
        const std::string html = slurp(prefix + ".html");
        CHECK(html.find("Design for manufacturing") != std::string::npos);
        CHECK(html.find("module 0.3000 has") == std::string::npos);
        CHECK(slurp(prefix + ".pdf").rfind("%PDF-1.4", 0) == 0);
        CHECK(run("report " + proj + " --format docx", dir).exit_code == 2);
    }
}

#ifndef OHS_SANITIZED
TEST_CASE("ohs manufacturing commands on the generated movement") {
    static const fs::path dir = [] {
        const fs::path d = fs::temp_directory_path() / "ohs_cli_m4b_test";
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

    SECTION("tolerance: seeded, reproducible, with an exit code against a threshold") {
        const auto a = run("--json tolerance " + proj + " --samples 3000", dir);
        REQUIRE(a.exit_code == 1);  // about 12 % of builds have an end-shake or binding problem with the default table
        const auto j = nlohmann::json::parse(a.output);
        CHECK(j["meshes"].size() == 4);
        CHECK(j["mobiles"].size() == 5);
        CHECK(j["failing_fraction"].get<double>() > 0.05);
        CHECK(run("--json tolerance " + proj + " --samples 3000", dir).output == a.output);
        CHECK(run("tolerance " + proj + " --samples 3000 --fail-above 0.5", dir).exit_code == 0);
        std::ofstream(dir / "tight.toml") << "[tolerance]\nposition_tol = 0.002\ntooth_thickness_tol = 0.002\narbor_length_tol = 0.004\nbridge_gap_tol = 0.004\n"
                                             "plate_flatness_tol = 0.004\npivot_diameter_tol = 0.001\nhole_diameter_tol = 0.001\n";
        CHECK(run("tolerance " + proj + " --samples 3000 --tolerances " + (dir / "tight.toml").string(), dir).exit_code == 0);
    }

    SECTION("collisions: the generated train meshes without overlap") {
        const auto r = run("--json collisions " + proj, dir);
        REQUIRE(r.exit_code == 0);
        const auto j = nlohmann::json::parse(r.output);
        CHECK(j["overlaps"] == 0);
        CHECK(j["pairs_checked"].get<int>() >= 4);
    }

    SECTION("report with every section") {
        const std::string prefix = (dir / "rep").string();
        const auto r = run("--json report " + proj + " --samples 2000 -o " + prefix, dir);
        REQUIRE((r.exit_code == 0 || r.exit_code == 1));
        const std::string html = slurp(prefix + ".html");
        for (const char* needle : {"Tolerance stack-up", "Design for manufacturing", "Motion and collisions", "Structural checks", "Tooth bending at full wind", "Solver verification"}) {
            CHECK(html.find(needle) != std::string::npos);
        }
        CHECK(slurp(prefix + ".pdf").rfind("%PDF-1.4", 0) == 0);
    }
}
#endif

TEST_CASE("ohs lesson commands end to end: every lesson can be done with the CLI alone") {
    static const fs::path dir = [] {
        const fs::path d = fs::temp_directory_path() / "ohs_cli_lessons_test";
        fs::remove_all(d);
        fs::create_directories(d);
        return d;
    }();

    SECTION("list and show, with and without JSON") {
        const auto list = run("--json lesson list", dir);
        REQUIRE(list.exit_code == 0);
        const auto j = nlohmann::json::parse(list.output);
        REQUIRE(j.size() == 7);
        CHECK(j[0]["id"] == "train");
        CHECK(j[6]["id"] == "skeletonise");
        const auto text = run("lesson list", dir);
        CHECK(text.output.find("hairspring") != std::string::npos);

        const auto show = run("lesson show train", dir);
        REQUIRE(show.exit_code == 0);
        CHECK(show.output.find("Lesson: Read the gear train  (demo movement)") != std::string::npos);
        CHECK(show.output.find("840.0") != std::string::npos);
        CHECK(show.output.find("Exercise") != std::string::npos);
        const auto js = nlohmann::json::parse(run("--json lesson show hairspring", dir).output);
        CHECK(js["kind"] == "study");
        CHECK(js["steps"].size() == 3);
        CHECK(run("lesson show astrology", dir).exit_code == 2);
        CHECK(run("lesson", dir).exit_code == 2);
    }

    SECTION("gate: the reference solution of every lesson passes its check") {
        struct Lesson {
            const char* id;
            const char* flag;
        };
        for (const Lesson l : {Lesson{"train", "--answer"}, Lesson{"timegrapher", "--answer"}, Lesson{"hairspring", "--study"}, Lesson{"mainspring", "--study"},
                               Lesson{"escapement", "--study"}, Lesson{"jewelling", "--answer"}, Lesson{"skeletonise", "--cutout"}}) {
            INFO(l.id);
            const auto sol = run(std::string("--json lesson check ") + l.id + " --solution", dir);
            REQUIRE(sol.exit_code == 0);
            const std::string solution = nlohmann::json::parse(sol.output)["solution"];
            std::string argument = solution;
            if (std::string(l.flag) == "--study") {
                const fs::path study = dir / (std::string(l.id) + ".toml");
                std::ofstream(study) << solution;
                argument = study.string();
            }
            const auto r = run(std::string("--json lesson check ") + l.id + " " + l.flag + " " + argument, dir);
            CHECK(r.exit_code == 0);
            const auto jr = nlohmann::json::parse(r.output);
            CHECK(jr["passed"] == true);
            CHECK_FALSE(jr["feedback"].empty());
        }
    }

    SECTION("wrong work exits 1 with advice; missing input exits 2") {
        const auto wrong = run("lesson check train --answer 12", dir);
        CHECK(wrong.exit_code == 1);
        CHECK(wrong.output.find("not yet") != std::string::npos);
        CHECK(wrong.output.find("Not passed yet") != std::string::npos);
        CHECK(run("lesson check train", dir).exit_code == 2);
        CHECK(run("lesson check hairspring", dir).exit_code == 2);
        std::ofstream(dir / "bad.toml") << "[study]\nname='x'\n[[change]]\nparameter='barrel_torque_nmm'\nscale=1.1\n";
        const auto off_topic = run("--json lesson check hairspring --study " + (dir / "bad.toml").string(), dir);
        CHECK(off_topic.exit_code == 1);
        CHECK(off_topic.output.find("is not part of this exercise") != std::string::npos);
        CHECK(run("lesson check skeletonise --cutout 20x9", dir).exit_code == 1);
        CHECK(run("lesson check skeletonise --cutout big", dir).exit_code == 2);
    }
}
