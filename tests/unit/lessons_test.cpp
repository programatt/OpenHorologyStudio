// SPDX-License-Identifier: Apache-2.0
#include <algorithm>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "lessons/lessons.hpp"

using namespace ohs;
using namespace ohs::lessons;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

const Context& context() {
    static const Context c = make_context(demo_movement());
    return c;
}

bool any_feedback(const ExerciseResult& r, const std::string& needle) {
    return std::any_of(r.feedback.begin(), r.feedback.end(), [&](const std::string& f) { return f.find(needle) != std::string::npos; });
}

ExerciseResult study(const std::string& id, const std::string& toml) {
    ExerciseInput in;
    in.study_toml = toml;
    return check_exercise(id, context(), in);
}

std::string scale(const std::string& parameter, double v) {
    return "[study]\nname='x'\n[[change]]\nparameter='" + parameter + "'\nscale=" + std::to_string(v) + "\n";
}

}  // namespace

TEST_CASE("the demo movement is the generated train: 25,200 vph, 6720 to the escape wheel") {
    const model::Movement m = demo_movement();
    CHECK(m.components.size() == 15);
    CHECK(m.mobiles.size() == 5);
    CHECK(m.meshes.size() == 4);
    const Context& c = context();
    CHECK(c.beat_rate_vph == Approx(25200));
    CHECK(c.chain.speed_ratio == Approx(6720));
    CHECK(c.chain.meshes == 4);
    CHECK(c.calibre.escape_teeth == 15);
    CHECK(c.train.mobiles[static_cast<std::size_t>(c.train.escape_mobile)].revs_per_hour == Approx(840));
}

TEST_CASE("make_context reports a rig without a train") {
    model::Movement m = demo_movement();
    m.meshes.clear();
    CHECK_THROWS_AS(make_context(m), Error);
    model::Movement no_rate = demo_movement();
    no_rate.beat_rate_vph = 0;
    CHECK_THROWS_AS(make_context(no_rate), Error);
    CHECK_NOTHROW(make_context(no_rate, "", 25200));
}

TEST_CASE("lessons are listed in order and every one has a narrative with this movement's numbers") {
    const auto all = list_lessons();
    REQUIRE(all.size() == 7);
    const std::vector<std::string> ids = {"train", "timegrapher", "hairspring", "mainspring", "escapement", "jewelling", "skeletonise"};
    for (std::size_t i = 0; i < ids.size(); ++i) {
        CHECK(all[i].id == ids[i]);
        CHECK_FALSE(all[i].title.empty());
        CHECK_FALSE(all[i].summary.empty());
    }
    CHECK_THROWS_AS(run_lesson("astrology", context()), Error);
    CHECK_THROWS_AS(check_exercise("astrology", context(), {}), Error);
    CHECK_THROWS_AS(reference_solution("astrology", context()), Error);

    for (const char* id : {"train", "timegrapher", "hairspring", "mainspring", "escapement", "jewelling"}) {
        const LessonRun run = run_lesson(id, context());
        INFO(id);
        int with_numbers = 0;
        CHECK(run.steps.size() == 3);
        CHECK_FALSE(run.exercise.empty());
        for (const Step& s : run.steps) {
            CHECK_FALSE(s.title.empty());
            CHECK(s.text.size() > 80);
            if (!s.numbers.empty()) ++with_numbers;
        }
        CHECK(with_numbers >= 2);  // at least two steps show this movement's own figures
    }
}

TEST_CASE("the train lesson shows the beat arithmetic with real figures") {
    const LessonRun run = run_lesson("train", context());
    auto value = [&](const std::string& name) {
        for (const Step& s : run.steps) {
            for (const auto& [k, v] : s.numbers) {
                if (k == name) return v;
            }
        }
        return std::string();
    };
    CHECK(value("escape wheel turns per hour") == "840.0");
    CHECK(value("beats per escape wheel turn") == "30");
    CHECK_THAT(value("beat rate"), ContainsSubstring("25200 vph (3.50 Hz)"));
    CHECK_THAT(value("fourth_wheel -> escape_pinion (84:6)"), ContainsSubstring("x14.000"));
    CHECK_THAT(run.exercise, ContainsSubstring("turns in an hour"));
}

TEST_CASE("the hairspring lesson's what-if figures follow the square-root rule") {
    const LessonRun run = run_lesson("hairspring", context());
    bool found = false;
    for (const Step& s : run.steps) {
        for (const auto& [k, v] : s.numbers) {
            if (k == "0.1 % shorter spring") {
                found = true;
                // "+43.2 s/day (the square-root rule predicts +43.2)": simulated and predicted agree.
                const double simulated = std::stod(v.substr(0, v.find(' ')));
                const auto at = v.find("predicts ");
                REQUIRE(at != std::string::npos);
                const double predicted = std::stod(v.substr(at + 9));
                CHECK(simulated == Approx(predicted).epsilon(0.03));
                CHECK(simulated > 30);
            }
        }
    }
    CHECK(found);
}

TEST_CASE("gate: every lesson's reference solution passes its exercise") {
    for (const char* id : {"train", "timegrapher", "hairspring", "mainspring", "escapement", "jewelling", "skeletonise"}) {
        INFO("lesson " << id);
        const std::string solution = reference_solution(id, context());
        ExerciseInput in;
        const std::string kind = [&] {
            for (const auto& l : list_lessons()) {
                if (l.id == id) return l.kind;
            }
            return std::string();
        }();
        if (kind == "answer") in.answer = std::stod(solution);
        else if (kind == "study") in.study_toml = solution;
        else in.cutout = solution;
        const ExerciseResult r = check_exercise(id, context(), in);
        for (const auto& f : r.feedback) INFO(f);
        CHECK(r.passed);
        CHECK_FALSE(r.feedback.empty());
        CHECK_FALSE(r.numbers.empty());
    }
}

TEST_CASE("wrong work fails with advice about what to change") {
    {
        ExerciseInput in;
        in.answer = 1.0;
        const auto r = check_exercise("train", context(), in);
        CHECK_FALSE(r.passed);
        CHECK(any_feedback(r, "not yet"));
        in.answer = 840.0 * 1.004;  // within 0.5 %
        CHECK(check_exercise("train", context(), in).passed);
    }
    {
        ExerciseInput in;
        in.answer = 5.0;
        CHECK_FALSE(check_exercise("timegrapher", context(), in).passed);
    }
    // Hairspring: the wrong direction, then a parameter outside the exercise.
    auto r = study("hairspring", scale("hairspring_length_mm", 1.01));
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "not +10"));
    r = study("hairspring", scale("barrel_torque_nmm", 1.1));
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "is not part of this exercise"));
    // Mainspring: a weaker spring loses amplitude; one that is far too strong knocks.
    r = study("mainspring", scale("barrel_torque_nmm", 0.9));
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "gained only"));
    r = study("mainspring", scale("barrel_torque_nmm", 1.6));
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "above 315"));
    // Escapement: a change that does not help.
    r = study("escapement", scale("drop_angle_deg", 1.05));
    CHECK_FALSE(r.passed);
    r = study("escapement", scale("mesh_efficiency", 1.05));
    CHECK(any_feedback(r, "is not part of this exercise"));
    // Jewelling: too tight, too loose, not a jewel seat.
    ExerciseInput j;
    j.answer = 1.95;
    r = check_exercise("jewelling", context(), j);
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "ream the seat"));
    j.answer = 2.001;
    CHECK_FALSE(check_exercise("jewelling", context(), j).passed);
    j.answer = 0.7;
    r = check_exercise("jewelling", context(), j);
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "not close enough"));
    j.answer = 1.990;
    CHECK(check_exercise("jewelling", context(), j).passed);
    j.answer = 1.980;  // 20 um: the edge of the window
    CHECK(check_exercise("jewelling", context(), j).passed);
}

TEST_CASE("exercises report bad input as errors, not verdicts") {
    CHECK_THROWS_AS(check_exercise("train", context(), {}), Error);
    CHECK_THROWS_AS(check_exercise("jewelling", context(), {}), Error);
    CHECK_THROWS_AS(check_exercise("hairspring", context(), {}), Error);
    ExerciseInput bad_toml;
    bad_toml.study_toml = "nothing = 1";
    CHECK_THROWS_AS(check_exercise("hairspring", context(), bad_toml), Error);
    ExerciseInput bad_cut;
    bad_cut.cutout = "big";
    CHECK_THROWS_AS(check_exercise("skeletonise", context(), bad_cut), Error);
    bad_cut.cutout = "0x5";
    CHECK_THROWS_AS(check_exercise("skeletonise", context(), bad_cut), Error);
}

TEST_CASE("skeletonise: windows too big or too small are told why") {
    ExerciseInput in;
    in.cutout = "13x8";
    auto r = check_exercise("skeletonise", context(), in);
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "would reach the pivot holes"));
    in.cutout = "2x1";
    r = check_exercise("skeletonise", context(), in);
    CHECK_FALSE(r.passed);
    CHECK(any_feedback(r, "make the window larger"));
    // The mass saved by a window is its area over the plate's (240 - 2 pi 0.36 mm2): 4 x 2 is 3.4 %.
    bool mass_listed = false;
    for (const auto& [k, v] : r.numbers) mass_listed |= k == "mass" && v.find("% less") != std::string::npos;
    CHECK(mass_listed);
}
