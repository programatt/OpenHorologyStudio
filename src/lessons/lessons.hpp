// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dynamics/calibre.hpp"
#include "dynamics/torque_chain.hpp"
#include "kinematics/kinematics.hpp"
#include "model/model.hpp"

namespace ohs::lessons {

/// Everything a lesson needs to talk about the user's own movement.
struct Context {
    model::Movement movement;
    kinematics::TrainAnalysis train;
    dynamics::TorqueChain chain;
    dynamics::Calibre calibre;
    double beat_rate_vph{};
};

/// Builds the context from a rig (and optional calibre file). Throws ohs::Error when the rig has
/// no train from a barrel to an escape wheel.
Context make_context(const model::Movement& movement, const std::string& calibre_file = "", double beat_rate_vph = 0.0);

/// A small going train and escapement from plain numbers, for learning without any CAD file:
/// 25,200 vph, the same train as the generated test movement.
model::Movement demo_movement();

struct Lesson {
    std::string id;
    std::string title;
    std::string summary;
    std::string kind;      ///< what the exercise asks for: "answer", "study" or "cutout"
};

struct Step {
    std::string title;
    std::string text;
    std::vector<std::pair<std::string, std::string>> numbers;  ///< the user's movement's figures for this step
    std::string try_it;                                        ///< a command to run next, may be empty
};

struct LessonRun {
    Lesson lesson;
    std::vector<Step> steps;
    std::string exercise;   ///< what to do, and how to check it
};

std::vector<Lesson> list_lessons();

/// The lesson's narrative with this movement's numbers plugged in. Throws ohs::Error for an unknown id.
LessonRun run_lesson(const std::string& id, const Context& context);

struct ExerciseInput {
    std::optional<double> answer;
    std::string study_toml;    ///< for "study" lessons: the what-if file the learner wrote
    std::string cutout;        ///< for "cutout" lessons: "WIDTHxHEIGHT" in mm
};

struct ExerciseResult {
    bool passed{false};
    std::vector<std::string> feedback;                          ///< what was checked, and what to change when it failed
    std::vector<std::pair<std::string, std::string>> numbers;
};

/// Checks the learner's work against the lesson's goal and says why it passes or not.
ExerciseResult check_exercise(const std::string& id, const Context& context, const ExerciseInput& input);

/// A working answer: a study file, a number or a cutout, as text to paste or save.
std::string reference_solution(const std::string& id, const Context& context);

}  // namespace ohs::lessons
