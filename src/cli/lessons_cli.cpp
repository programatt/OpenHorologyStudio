// SPDX-License-Identifier: Apache-2.0
#include "cli/lessons_cli.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

#include <nlohmann/json.hpp>

#include "core/error.hpp"
#include "lessons/lessons.hpp"
#include "model/rig_io.hpp"

using nlohmann::json;

namespace ohs::cli {

namespace {

struct LessonOptions {
    std::string id, project, calibre_file, study_file, cutout;
    double beat_rate = 0, answer = 0;
    bool has_answer = false, solution = false;
};

lessons::Context load_context(const LessonOptions& o) {
    const model::Movement m = o.project.empty() ? lessons::demo_movement() : model::load_rig(o.project);
    return lessons::make_context(m, o.calibre_file, o.beat_rate);
}

std::string read_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << f.rdbuf();
    return buf.str();
}

// Wraps text at about 92 columns with an indent.
std::string wrap(const std::string& text, const std::string& indent) {
    std::istringstream in(text);
    std::string word, line, out;
    while (in >> word) {
        if (!line.empty() && indent.size() + line.size() + 1 + word.size() > 92) {
            out += indent + line + "\n";
            line = word;
        } else {
            line += (line.empty() ? "" : " ") + word;
        }
    }
    if (!line.empty()) out += indent + line + "\n";
    return out;
}

int cmd_list(const LessonOptions&, bool as_json) {
    const auto all = lessons::list_lessons();
    if (as_json) {
        json j = json::array();
        for (const auto& l : all) j.push_back({{"id", l.id}, {"title", l.title}, {"summary", l.summary}, {"kind", l.kind}});
        std::cout << j.dump(2) << '\n';
        return 0;
    }
    std::cout << "Guided lessons. Each one runs real analyses on your movement (or a built-in demo when you give no project):\n\n";
    int n = 1;
    for (const auto& l : all) {
        std::cout << std::right << std::setw(2) << n++ << ". " << std::left << std::setw(13) << l.id << l.title << '\n' << wrap(l.summary, "                ");
    }
    std::cout << "\nohs lesson show <id> [--project P]   reads the lesson with your numbers\n"
                 "ohs lesson check <id> ...            checks your exercise\n";
    return 0;
}

int cmd_show(const LessonOptions& o, bool as_json) {
    const lessons::Context c = load_context(o);
    const lessons::LessonRun run = lessons::run_lesson(o.id, c);
    if (as_json) {
        json j = {{"id", run.lesson.id}, {"title", run.lesson.title}, {"kind", run.lesson.kind}, {"exercise", run.exercise}, {"steps", json::array()}};
        for (const auto& s : run.steps) {
            json numbers = json::array();
            for (const auto& [k, v] : s.numbers) numbers.push_back({{"name", k}, {"value", v}});
            j["steps"].push_back({{"title", s.title}, {"text", s.text}, {"numbers", numbers}, {"try_it", s.try_it}});
        }
        std::cout << j.dump(2) << '\n';
        return 0;
    }
    std::cout << "Lesson: " << run.lesson.title << (o.project.empty() ? "  (demo movement)" : "") << "\n\n";
    int n = 1;
    for (const auto& s : run.steps) {
        std::cout << n++ << ". " << s.title << "\n" << wrap(s.text, "   ");
        for (const auto& [k, v] : s.numbers) std::cout << "     " << std::left << std::setw(34) << k << v << '\n';
        if (!s.try_it.empty()) std::cout << "   Try: " << s.try_it << '\n';
        std::cout << '\n';
    }
    std::cout << "Exercise\n" << wrap(run.exercise, "   ");
    return 0;
}

int cmd_check(const LessonOptions& o, bool as_json) {
    const lessons::Context c = load_context(o);
    if (o.solution) {
        const std::string s = lessons::reference_solution(o.id, c);
        if (as_json) std::cout << json{{"id", o.id}, {"solution", s}}.dump(2) << '\n';
        else std::cout << s << (s.back() == '\n' ? "" : "\n");
        return 0;
    }
    lessons::ExerciseInput in;
    if (o.has_answer) in.answer = o.answer;
    if (!o.study_file.empty()) in.study_toml = read_file(o.study_file);
    in.cutout = o.cutout;
    const lessons::ExerciseResult r = lessons::check_exercise(o.id, c, in);
    if (as_json) {
        json numbers = json::array();
        for (const auto& [k, v] : r.numbers) numbers.push_back({{"name", k}, {"value", v}});
        std::cout << json{{"id", o.id}, {"passed", r.passed}, {"feedback", r.feedback}, {"numbers", numbers}}.dump(2) << '\n';
    } else {
        for (const auto& [k, v] : r.numbers) std::cout << std::left << std::setw(30) << k << v << '\n';
        for (const auto& f : r.feedback) std::cout << wrap(f, "");
        std::cout << (r.passed ? "Lesson exercise passed.\n" : "Not passed yet: try again after changing the work as suggested.\n");
    }
    return r.passed ? 0 : 1;
}

}  // namespace

void add_lesson_commands(CLI::App& app, const bool& json_flag, std::function<int()>& chosen) {
    auto o = std::make_shared<LessonOptions>();
    auto pick = [&chosen, &json_flag, o](int (*fn)(const LessonOptions&, bool)) {
        return [&chosen, &json_flag, o, fn] { chosen = [&json_flag, o, fn] { return fn(*o, json_flag); }; };
    };
    auto* lesson = app.add_subcommand("lesson", "Guided lessons on a movement (OM10 or the built-in demo)");
    lesson->require_subcommand(1);
    auto* list = lesson->add_subcommand("list", "The lessons");
    list->callback(pick(cmd_list));

    auto common = [o](CLI::App* cmd) {
        cmd->add_option("id", o->id, "Lesson id (ohs lesson list)")->required();
        cmd->add_option("--project", o->project, "Project file (default: a built-in demo movement)")->check(CLI::ExistingFile);
        cmd->add_option("--calibre", o->calibre_file, "Calibre parameters TOML")->check(CLI::ExistingFile);
        cmd->add_option("--beat-rate", o->beat_rate, "Beat rate in vph (default: from project)");
    };
    auto* show = lesson->add_subcommand("show", "Read a lesson with your movement's numbers");
    common(show);
    show->callback(pick(cmd_show));
    auto* check = lesson->add_subcommand("check", "Check your exercise");
    common(check);
    check->add_option("--answer", o->answer, "Your numeric answer")->each([o](const std::string&) { o->has_answer = true; });
    check->add_option("--study", o->study_file, "Your what-if study (TOML)")->check(CLI::ExistingFile);
    check->add_option("--cutout", o->cutout, "Window WIDTHxHEIGHT in mm");
    check->add_flag("--solution", o->solution, "Print a working answer instead of checking yours");
    check->callback(pick(cmd_check));
}

}  // namespace ohs::cli
