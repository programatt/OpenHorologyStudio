// SPDX-License-Identifier: Apache-2.0
#include "lessons/lessons.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <set>
#include <sstream>

#include "cases/cases.hpp"
#include "core/error.hpp"
#include "dynamics/oscillator.hpp"
#include "dynamics/study.hpp"
#include "dynamics/timegrapher.hpp"
#include "dynamics/timekeeping.hpp"
#include "explain/explain.hpp"
#include "manufacturing/dfm.hpp"
#include "rigging/rigging.hpp"

namespace ohs::lessons {

namespace {

constexpr double kPi = std::numbers::pi;

std::string num(double v, int prec = 2) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

std::string sgn(double v, int prec = 1) {
    std::ostringstream os;
    os << std::showpos << std::fixed << std::setprecision(prec) << v;
    return os.str();
}


const std::vector<Lesson>& catalogue() {
    static const std::vector<Lesson> v = {
        {"train", "Read the gear train", "Follow the power from the barrel to the escape wheel and do the beat arithmetic yourself.", "answer"},
        {"timegrapher", "Read a timegrapher", "Rate, amplitude and beat error, and how orientation and wind change them.", "answer"},
        {"hairspring", "Hairspring and balance", "What sets the frequency, and how to regulate by changing the hairspring.", "study"},
        {"mainspring", "Choosing a mainspring", "Barrel torque against amplitude and power reserve.", "study"},
        {"escapement", "Escape wheel and lever geometry", "Drop, lift and unlocking: where the energy of each beat goes.", "study"},
        {"jewelling", "Jewelling", "Press-fit interference of a jewel in a plate and what it does to the plate.", "answer"},
        {"skeletonise", "Skeletonising a bridge", "Remove metal from a bridge without losing its strength under shock.", "cutout"},
    };
    return v;
}

const Lesson& find_lesson(const std::string& id) {
    for (const Lesson& l : catalogue()) {
        if (l.id == id) return l;
    }
    std::string known;
    for (const Lesson& l : catalogue()) known += (known.empty() ? "" : ", ") + l.id;
    throw Error("unknown lesson '" + id + "' (" + known + ")");
}

double escape_rph(const Context& c) {
    if (c.train.escape_mobile < 0) return 0.0;
    return c.train.mobiles[static_cast<std::size_t>(c.train.escape_mobile)].revs_per_hour;
}

// ---- the demonstration bridge for the skeletonising lesson ---------------------------------------------------------

constexpr double kBridgeLength = 24, kBridgeWidth = 10, kBridgeThickness = 0.8, kBoreRadius = 0.6;
constexpr double kShockG = 5000;

TopoDS_Shape demo_bridge(double cut_w, double cut_h) {
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(kBridgeLength, kBridgeWidth, kBridgeThickness).Shape();
    for (double x : {3.0, kBridgeLength - 3.0}) {
        const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, kBridgeWidth / 2, -0.1), gp_Dir(0, 0, 1)), kBoreRadius, kBridgeThickness + 0.2).Shape();
        plate = BRepAlgoAPI_Cut(plate, bore).Shape();
    }
    if (cut_w > 0 && cut_h > 0) {
        const TopoDS_Shape window = BRepPrimAPI_MakeBox(gp_Pnt((kBridgeLength - cut_w) / 2, (kBridgeWidth - cut_h) / 2, -0.1), cut_w, cut_h, kBridgeThickness + 0.2).Shape();
        plate = BRepAlgoAPI_Cut(plate, window).Shape();
    }
    return plate;
}

cases::PartCaseResult bridge_case(double cut_w, double cut_h) {
    cases::PartCaseInput in;
    in.shape = demo_bridge(cut_w, cut_h);
    in.element_size = 0.5;
    in.layers = 1;
    in.material = fea::find_material("brass_leaded");
    in.shock_g = kShockG;
    return cases::run_part_case(in);
}

bool parse_cutout(const std::string& text, double& w, double& h) {
    const auto x = text.find_first_of("xX");
    if (x == std::string::npos) return false;
    try {
        w = std::stod(text.substr(0, x));
        h = std::stod(text.substr(x + 1));
    } catch (const std::exception&) {
        return false;
    }
    return w > 0 && h > 0;
}

// ---- helpers for the study exercises -------------------------------------------------------------------------------------

struct StudyOutcome {
    dynamics::StudyResult result;
    std::vector<std::string> problems;
};

const dynamics::MetricRow& metric(const dynamics::StudyResult& r, const std::string& name) {
    for (const auto& m : r.metrics) {
        if (m.name == name) return m;
    }
    throw Error("internal: no metric " + name);
}

/// Parses and runs the learner's study, after checking it only touches `allowed` parameters.
StudyOutcome run_learner_study(const Context& c, const std::string& toml, const std::set<std::string>& allowed, const std::string& allowed_text) {
    StudyOutcome out;
    if (toml.empty()) throw Error("give your study file with --study (see the lesson for what to put in it)");
    const dynamics::StudyDefinition def = dynamics::study_from_toml(toml);
    for (const auto& ch : def.changes) {
        if (!allowed.count(ch.parameter)) out.problems.push_back("'" + ch.parameter + "' is not part of this exercise: change only " + allowed_text);
    }
    out.result = dynamics::run_study(c.calibre, c.chain, def);
    return out;
}

std::string study_text(const std::vector<std::pair<std::string, double>>& scales, const std::string& name) {
    std::ostringstream os;
    os << std::setprecision(8) << "[study]\nname = \"" << name << "\"\n";
    for (const auto& [p, v] : scales) os << "\n[[change]]\nparameter = \"" << p << "\"\nscale = " << v << "\n";
    return os.str();
}

// Lame pressure of a jewel pressed into a plate, plane stress: ruby jewel of radius r in a brass ring of outer radius ro.
double jewel_pressure(double interference, double r, double ro, const fea::Material& plate, const fea::Material& jewel) {
    const double hub = (1.0 / plate.youngs_mpa()) * ((ro * ro + r * r) / (ro * ro - r * r) + plate.poisson);
    const double shaft = (1.0 / jewel.youngs_mpa()) * (1.0 - jewel.poisson);
    return interference / (r * (hub + shaft));
}

}  // namespace

// ---- context ---------------------------------------------------------------------------------------------------------------

Context make_context(const model::Movement& m, const std::string& calibre_file, double beat_rate_vph) {
    Context c;
    c.movement = m;
    c.beat_rate_vph = beat_rate_vph > 0 ? beat_rate_vph : m.beat_rate_vph;
    if (c.beat_rate_vph <= 0) throw Error("the project has no beat rate; pass --beat-rate (e.g. 25200)");
    c.train = kinematics::analyze_train(m, c.beat_rate_vph);
    c.calibre = calibre_file.empty() ? dynamics::default_calibre(c.beat_rate_vph) : dynamics::load_calibre(calibre_file);
    c.calibre.beat_rate_vph = c.beat_rate_vph;
    c.chain = dynamics::build_chain(m, c.train, c.calibre);
    dynamics::adopt_train(c.calibre, c.chain, c.beat_rate_vph);
    return c;
}

model::Movement demo_movement() {
    constexpr double m = 0.15, deg = kPi / 180.0;
    auto gear = [&](std::string name, int teeth, double x, double y, double z0, double z1) {
        model::PartFeatures f;
        f.name = std::move(name);
        f.teeth = teeth;
        f.tooth_confidence = 1.0;
        f.outer_radius = (teeth + 2) * m / 2.0;
        f.root_radius = (teeth - 2.5) * m / 2.0;
        f.flank_base_radius = teeth * m / 2.0 * std::cos(20.0 * deg);
        f.flank_rms = 1e-4;
        f.bore_radius = 0.1;
        f.center = {x, y};
        f.z_min = z0;
        f.z_max = z1;
        f.extent_x = f.extent_y = 2 * f.outer_radius;
        f.volume = 1.0;
        return f;
    };
    auto arbor = [&](std::string name, double x, double y, double z0, double z1) {
        model::PartFeatures f;
        f.name = std::move(name);
        f.outer_radius = f.extent_x = f.extent_y = 0.1;
        f.center = {x, y};
        f.z_min = z0;
        f.z_max = z1;
        f.volume = 0.1;
        return f;
    };
    std::vector<model::PartFeatures> p;
    double x = 0, y = 0;
    p.push_back(gear("barrel", 80, x, y, 0.0, 1.0));
    p.push_back(arbor("barrel_arbor", x, y, -0.2, 1.2));
    x += m * 45;
    p.push_back(gear("center_pinion", 10, x, y, 0.0, 1.0));
    p.push_back(gear("center_wheel", 80, x, y, 1.2, 2.0));
    p.push_back(arbor("center_arbor", x, y, -0.2, 2.2));
    x += m * 45 * std::cos(120 * deg);
    y += m * 45 * std::sin(120 * deg);
    p.push_back(gear("third_pinion", 10, x, y, 1.2, 2.0));
    p.push_back(gear("third_wheel", 75, x, y, 2.2, 3.0));
    p.push_back(arbor("third_arbor", x, y, 1.0, 3.2));
    x += m * 42.5 * std::cos(20 * deg);
    y += m * 42.5 * std::sin(20 * deg);
    p.push_back(gear("fourth_pinion", 10, x, y, 2.2, 3.0));
    p.push_back(gear("fourth_wheel", 84, x, y, 3.2, 4.0));
    p.push_back(arbor("fourth_arbor", x, y, 2.0, 4.2));
    x += m * 45 * std::cos(60 * deg);
    y += m * 45 * std::sin(60 * deg);
    p.push_back(gear("escape_pinion", 6, x, y, 3.2, 4.0));
    p.push_back(gear("escape_wheel", 15, x, y, 4.2, 5.0));
    p.push_back(arbor("escape_arbor", x, y, 3.0, 5.2));
    model::PartFeatures plate;
    plate.name = "main_plate";
    plate.extent_x = plate.extent_y = 40;
    plate.z_min = -0.8;
    plate.z_max = -0.3;
    plate.volume = 100;
    p.push_back(plate);
    rigging::RigOptions opt;
    opt.beat_rate_vph = 25200;
    return rigging::auto_rig("demo movement", std::move(p), opt);
}

std::vector<Lesson> list_lessons() { return catalogue(); }

// ---- narratives --------------------------------------------------------------------------------------------------------------------

LessonRun run_lesson(const std::string& id, const Context& c) {
    LessonRun run;
    run.lesson = find_lesson(id);
    const dynamics::Calibre& cal = c.calibre;
    const double barrel_period = c.chain.barrel_period_hours;

    if (id == "train") {
        const auto walk = explain::walk_train(c.movement, c.train);
        Step s1{"Follow the power", "The mainspring's torque turns the barrel. Every gear mesh hands the power to the next mobile: the speed is multiplied by the ratio of teeth and the torque falls by the same factor, "
                "less a little friction in the mesh. Reading the train is multiplying these ratios.", {}, "ohs explain PROJECT --train"};
        s1.numbers.push_back({"barrel turns once per", num(barrel_period, 2) + " h"});
        for (const auto& st : walk.steps) {
            s1.numbers.push_back({st.driver + " -> " + st.driven + " (" + std::to_string(st.driver_teeth) + ":" + std::to_string(st.driven_teeth) + ")", "x" + num(st.step_ratio, 3) + " step, x" + num(std::abs(st.cumulative_ratio), 2) + " from the barrel"});
        }
        Step s2{"The beat arithmetic", "The escape wheel releases one tooth every two beats, so one turn of an N-tooth escape wheel is 2N beats. The balance sets the beat rate; the train must turn the escape wheel at exactly the speed that makes the teeth "
                "pass at that rate. That fixes the whole train: beats per hour / (2N) is the escape wheel's turns per hour.", {}, "ohs kinematics PROJECT"};
        s2.numbers = {{"beat rate", num(c.beat_rate_vph, 0) + " vph (" + num(c.beat_rate_vph / 7200.0, 2) + " Hz)"},
                      {"escape wheel teeth", std::to_string(c.train.escape_teeth)},
                      {"beats per escape wheel turn", std::to_string(2 * c.train.escape_teeth)},
                      {"escape wheel turns per hour", num(escape_rph(c), 1)}};
        Step s3{"Where the torque goes", "Because speed goes up by the whole train ratio, the torque left at the escape wheel is a tiny fraction of the barrel's: the ratio and the mesh losses (here " + num(cal.mesh_efficiency * 100, 0) +
                " % per mesh) shrink it. That small torque times the escape wheel's speed is all the power the balance ever gets.", {}, "ohs torque PROJECT"};
        s3.numbers = {{"barrel torque, fully wound", num(cal.barrel_torque_nmm, 3) + " N·mm"},
                      {"speed-up to the escape wheel", "x" + num(c.chain.speed_ratio, 0)},
                      {"meshes on the way", std::to_string(c.chain.meshes)},
                      {"torque at the escape wheel", num(dynamics::escape_torque_nm(c.chain, cal, 1.0) * 1e3, 6) + " N·mm"},
                      {"power at the escape wheel", num(dynamics::escape_power_w(c.chain, cal, 1.0) * 1e6, 3) + " µW"}};
        run.steps = {s1, s2, s3};
        run.exercise = "Work out by hand how many times the escape wheel turns in an hour, then check it with `ohs lesson check train --answer <your number>`. (Beat rate divided by two beats per tooth, divided by the tooth count.)";
    } else if (id == "timegrapher") {
        const auto full = dynamics::wind_point(cal, c.chain, 1.0, dynamics::Position::DialUp);
        dynamics::RunConditions rc;
        rc.wheel_torque_nm = full.wheel_torque_nm;
        rc.duration_s = 140;
        dynamics::EventLog log;
        dynamics::simulate_oscillator(cal, rc, log.sink());
        const auto tr = dynamics::virtual_timegrapher(log.events, cal, 120, 140);
        Step s1{"What it hears", "A timegrapher listens to the ticks, never to the balance itself. From their spacing it reads the rate; from how long the lever pushes (the lift time) and the period it works out the amplitude with "
                "A = lift / (2 sin(pi t / T)); from the unequal spacing of tick and tock it reads the beat error.", {}, "ohs timegrapher PROJECT --duration 30s"};
        s1.numbers = {{"rate", sgn(tr.reading.rate_s_per_day) + " s/day"}, {"amplitude", num(tr.reading.amplitude_deg, 1) + " deg"}, {"beat error", num(tr.reading.beat_error_ms, 2) + " ms"},
                      {"lift angle assumed", num(cal.lift_angle_deg, 0) + " deg"}};
        const auto pos = dynamics::positional_error(cal, c.chain, 0.5);
        Step s2{"Orientation", "Lying flat the balance's weight rests on the pivot ends; hanging, it rests on the pivot sides, which rub more, so the amplitude drops. A slightly unpoised balance also feels gravity differently in each hanging position and changes the rate.", {}, "ohs simulate PROJECT --position crown_up --wind 0.5"};
        for (const auto& p : pos.points) s2.numbers.push_back({dynamics::to_string(p.position), num(p.amplitude_deg, 1) + " deg, " + sgn(p.rate_s_per_day) + " s/day"});
        s2.numbers.push_back({"positional spread", num(pos.spread_s_per_day, 1) + " s/day"});
        const auto low = dynamics::wind_point(cal, c.chain, 0.1, dynamics::Position::DialUp);
        const auto mid = dynamics::wind_point(cal, c.chain, 0.5, dynamics::Position::DialUp);
        Step s3{"As the spring runs down", "The torque at the escape wheel falls as the mainspring unwinds, so the balance gets less energy per beat and the amplitude falls. Rate changing with amplitude is the isochronism of the movement.", {}, "ohs timekeeping PROJECT"};
        s3.numbers = {{"amplitude fully wound", num(full.amplitude_deg, 1) + " deg"}, {"amplitude half wound", num(mid.amplitude_deg, 1) + " deg"}, {"amplitude 10 % wound", num(low.amplitude_deg, 1) + " deg"},
                      {"rate change, full to 10 %", sgn(low.rate_s_per_day - full.rate_s_per_day, 2) + " s/day"}};
        run.steps = {s1, s2, s3};
        run.exercise = "What rate does the movement show crown up, half wound? Find it with `ohs simulate PROJECT --position crown_up --wind 0.5` and check it with `ohs lesson check timegrapher --answer <s/day>` (within 1 s/day).";
    } else if (id == "hairspring") {
        const double k = cal.hairspring_stiffness();
        Step s1{"What sets the beat", "The balance and hairspring are a torsion pendulum: frequency f = sqrt(k / I) / 2pi, with k the stiffness of the spring (k = E b h^3 / 12 L for a flat spiral) and I the inertia of the balance. The escapement only lets it run; "
                "it hardly changes the frequency.", {}, "ohs calibre init -o calibre.toml"};
        s1.numbers = {{"balance inertia", num(cal.balance_inertia_mgcm2, 2) + " mg·cm²"}, {"hairspring section", num(cal.hairspring_width_mm, 3) + " x " + num(cal.hairspring_thickness_mm, 3) + " mm"},
                      {"hairspring length", num(cal.hairspring_length_mm, 1) + " mm"}, {"stiffness k", num(k * 1e6, 4) + " µN·m/rad"},
                      {"frequency", num(cal.natural_frequency_hz(), 4) + " Hz = " + num(cal.natural_frequency_hz() * 7200, 0) + " vph"}};
        Step s2{"Regulating", "Because k is proportional to 1/L, a shorter spring is stiffer and the watch runs faster; the frequency goes as the square root, so 1 % of length is 0.5 % of frequency, a bit over 400 s/day. Real regulators move a pin on the spring to do this by tiny amounts.", {}, ""};
        {
            const auto def = dynamics::study_from_toml(study_text({{"hairspring_length_mm", 0.999}}, "0.1 % shorter"));
            const auto r = dynamics::run_study(cal, c.chain, def);
            s2.numbers = {{"0.1 % shorter spring", sgn(metric(r, "rate, dial up, half wind").delta(), 1) + " s/day (the square-root rule predicts " + sgn((std::sqrt(1 / 0.999) - 1) * 86400, 1) + ")"}};
        }
        Step s3{"Mass and the balance", "Heavier or larger balances have more inertia and slow the watch the same way a longer spring does. Changing inertia by 1 % changes the frequency by 0.5 %, in the opposite direction to the spring's stiffness.", {}, "ohs whatif PROJECT study.toml"};
        {
            const auto def = dynamics::study_from_toml(study_text({{"balance_inertia_mgcm2", 1.001}}, "0.1 % heavier balance"));
            const auto r = dynamics::run_study(cal, c.chain, def);
            s3.numbers = {{"0.1 % more inertia", sgn(metric(r, "rate, dial up, half wind").delta(), 1) + " s/day"}};
        }
        run.steps = {s1, s2, s3};
        run.exercise = "Write a what-if study (a TOML file with one [[change]] of hairspring_length_mm) that makes the dial-up rate 10 s/day faster than the regulated baseline. Check it with `ohs lesson check hairspring --study study.toml`. It passes when the rate shift is 10 s/day within 1.5 and nothing else moved much.";
    } else if (id == "mainspring") {
        const auto full = dynamics::wind_point(cal, c.chain, 1.0);
        const auto low = dynamics::wind_point(cal, c.chain, 0.1);
        const double reserve = dynamics::power_reserve_hours(cal, c.chain);
        Step s1{"Torque and amplitude", "The balance's amplitude settles where the energy each beat delivers equals what friction and air take away. A stronger mainspring delivers more, so the amplitude grows, but not in proportion: losses grow with amplitude too. Above about 315 degrees the roller jewel can knock the fork's guard pin.", {}, "ohs simulate PROJECT --wind 1"};
        s1.numbers = {{"barrel torque", num(cal.barrel_torque_nmm, 3) + " N·mm"}, {"amplitude, fully wound", num(full.amplitude_deg, 1) + " deg"}, {"amplitude, 10 % wound", num(low.amplitude_deg, 1) + " deg"}};
        Step s2{"Power reserve", "The reserve is how long the watch runs above the minimum amplitude (" + num(cal.min_running_amplitude_deg, 0) + " deg here). It is limited by the turns the barrel offers (" + num(cal.barrel_turns, 2) + ") times the barrel's period, and by how soon the torque curve falls too low to keep the amplitude up.", {}, "ohs timekeeping PROJECT"};
        s2.numbers = {{"barrel turns", num(cal.barrel_turns, 2)}, {"barrel period", num(barrel_period, 2) + " h"}, {"spring time if never limited", num(dynamics::full_wind_hours(cal, c.chain), 1) + " h"},
                      {"power reserve", num(reserve, 1) + " h"}};
        Step s3{"The trade-off", "Raising the torque buys amplitude and costs wear and the risk of knocking. Lowering it extends nothing here (the reserve is set by turns) but gives margin against dirt and cold oil. A good choice keeps the amplitude between about 270 and 310 degrees across the wind.", {}, ""};
        run.steps = {s1, s2, s3};
        run.exercise = "Raise the fully wound dial-up amplitude by at least 8 degrees without going above 315 degrees or losing more than 2 hours of power reserve, changing only barrel_torque_nmm and/or barrel_turns. Check it with `ohs lesson check mainspring --study study.toml`.";
    } else if (id == "escapement") {
        const auto g = dynamics::escapement_geometry(cal, dynamics::escape_torque_nm(c.chain, cal, 1.0));
        const auto full = dynamics::wind_point(cal, c.chain, 1.0);
        const auto budget = dynamics::energy_budget(cal, full.wheel_torque_nm, full.amplitude_deg, dynamics::Position::DialUp);
        Step s1{"Where one beat's energy goes", "Each beat the escape wheel does work torque x (pi / teeth). The lever passes most of it to the balance through the impulse; some is lost to the drop, to friction on the pallets, to unlocking, and to the kinetic energy of the wheel, which is picked up and then thrown away at every beat. What is left keeps the balance swinging against pivot friction and air.", {}, "ohs timekeeping PROJECT"};
        s1.numbers = {{"wheel work per beat", num(budget.wheel_work_j * 1e9, 2) + " nJ"}, {"delivered to the balance", num(budget.delivered_j * 1e9, 2) + " nJ"}, {"unlocking", num(budget.lost_unlock_j * 1e9, 2) + " nJ"},
                      {"wheel pick-up and drop", num(budget.lost_wheel_hit_j * 1e9, 2) + " nJ"}, {"pivots", num(budget.lost_friction_j * 1e9, 2) + " nJ"}, {"air", num(budget.lost_air_j * 1e9, 2) + " nJ"},
                      {"wheel to balance", num(budget.efficiency * 100, 0) + " %"}};
        Step s2{"Geometry", "Of each wheel step of " + num(g.wheel_step * 180 / kPi, 1) + " degrees, " + num(cal.drop_angle_deg, 1) + " is lost as drop (the wheel falls free onto the next pallet), the rest drives the impulse over " + num(cal.lift_angle_deg, 0) +
                " degrees of balance motion. Unlocking costs " + num(cal.unlock_angle_deg, 0) + " degrees of balance motion against friction that grows with the wheel's torque.", {}, ""};
        s2.numbers = {{"lift angle", num(cal.lift_angle_deg, 1) + " deg"}, {"drop", num(cal.drop_angle_deg, 2) + " deg"}, {"unlocking", num(cal.unlock_angle_deg, 1) + " deg"},
                      {"impulse efficiency", num(cal.impulse_efficiency * 100, 0) + " %"}, {"unlocking friction", num(cal.unlock_friction_ratio * 100, 0) + " % of the impulse torque"}};
        Step s3{"Lead and isochronism", "An impulse that starts a little before the dead point (lead) pushes the balance ahead and makes the rate depend on amplitude; the model's lead is " + num(cal.impulse_lead_deg, 1) +
                " degrees. Cutting losses gains amplitude, but a change that also changes the rate-versus-amplitude slope trades one problem for another.", {}, ""};
        const auto points = dynamics::amplitude_vs_wind(cal, c.chain, 4);
        s3.numbers = {{"isochronism slope", num(dynamics::isochronism_slope(points), 4) + " s/day per deg"}};
        run.steps = {s1, s2, s3};
        run.exercise = "Gain at least 4 degrees of fully wound dial-up amplitude by changing only escapement parameters (drop_angle_deg, impulse_efficiency, unlock_friction_ratio, unlock_angle_deg, lift_angle_deg, impulse_lead_deg, escape_wheel_inertia_mgcm2), "
                       "without letting the isochronism slope grow beyond 0.05 s/day per degree. Check it with `ohs lesson check escapement --study study.toml`.";
    } else if (id == "jewelling") {
        const fea::Material& plate = fea::find_material("brass_leaded");
        const fea::Material& ruby = fea::find_material("ruby_synthetic");
        const double d_jewel = 2.0, ro = 4.0;
        Step s1{"The fit", "A jewel is held by an interference fit: the hole in the plate is a few micrometres smaller than the jewel, and pressing it in stretches the plate around it. Too little and the jewel falls out, too much and it cracks or distorts the bearing.", {}, "ohs dfm PROJECT --process jewelling"};
        s1.numbers = {{"jewel outer diameter", num(d_jewel, 3) + " mm"}, {"acceptable interference", "4 to 20 µm"}};
        Step s2{"What the plate feels", "For a thick ring of brass (outer radius " + num(ro, 1) + " mm) around a ruby, Lame's equations give the contact pressure from the interference. The ring is stretched in hoop direction by about the pressure times (R^2 + r^2)/(R^2 - r^2).", {}, ""};
        for (double um : {4.0, 10.0, 20.0}) {
            const double p = jewel_pressure(um * 1e-3, d_jewel / 2, ro, plate, ruby);
            const double hoop = p * (ro * ro + d_jewel * d_jewel / 4) / (ro * ro - d_jewel * d_jewel / 4);
            s2.numbers.push_back({num(um, 0) + " µm interference", "contact pressure " + num(p, 0) + " MPa, plate hoop stress " + num(hoop, 0) + " MPa (brass yields near " + num(plate.strength_mpa, 0) + " MPa)"});
        }
        Step s3{"Making the hole", "You reach the diameter by reaming. A hole smaller than the jewel by the interference you want is the answer; the DFM rule for jewelling checks that every jewel seat in your plates is inside the window.", {}, ""};
        run.steps = {s1, s2, s3};
        run.exercise = "What hole diameter (in mm) gives a 2.000 mm jewel an interference of 10 µm? Check it with `ohs lesson check jewelling --answer <mm>`. Any hole that leaves 4 to 20 µm passes, and the check reports the plate's stress for your choice.";
    } else if (id == "skeletonise") {
        const auto base = bridge_case(0, 0);
        const double area_full = kBridgeLength * kBridgeWidth - 2 * kPi * kBoreRadius * kBoreRadius;
        Step s1{"The solid bridge", "A bridge holds the pivots of the mobiles; under a shock it is loaded by its own inertia. With both pivot holes clamped, the demonstration bridge (24 x 10 x 0.8 mm brass) is checked against a " + num(kShockG, 0) +
                " g end shake, the way `ohs fea --case shock` checks a real one.", {}, "ohs fea PROJECT --part BRIDGE --case shock --g 5000"};
        s1.numbers = {{"mass", num(base.mass_g, 3) + " g"}, {"peak von Mises stress", num(base.max_von_mises_mpa, 1) + " MPa"}, {"largest deflection", num(base.max_displacement_mm * 1000, 2) + " µm"},
                      {"safety factor against yield", num(base.safety_factor, 1)}, {"first-pass mesh", std::to_string(base.dofs) + " dofs"}};
        Step s2{"Why skeletonising works", "Away from the clamped pivots the metal carries little: a bridge is stiff because of its depth and edges, not its middle. Cutting a window removes mass with little loss of stiffness, until the frame around the window gets slender enough to take the shock load itself.", {}, ""};
        s2.numbers = {{"plate area", num(area_full, 1) + " mm²"}, {"window that fits", "up to about 12 x 6 mm between the pivot holes"}};
        Step s3{"Try a window", "A rectangular window is centred between the pivot holes. Give its width and height and the lesson re-runs the same shock analysis. Mass should fall by at least 20 % while the safety factor stays at 2 or more.", {}, "ohs lesson check skeletonise --cutout 10x5"};
        run.steps = {s1, s2, s3};
        run.exercise = "Choose a window `WIDTHxHEIGHT` (mm) that cuts the bridge's mass by at least 20 % and keeps the safety factor at 2 or more under 5000 g. Check it with `ohs lesson check skeletonise --cutout WIDTHxHEIGHT`. The window must stay inside the plate and clear of the pivot holes.";
    }
    return run;
}

// ---- exercises -----------------------------------------------------------------------------------------------------------------------

ExerciseResult check_exercise(const std::string& id, const Context& c, const ExerciseInput& in) {
    find_lesson(id);
    ExerciseResult r;
    auto pass = [&](bool ok, const std::string& good, const std::string& bad) {
        r.feedback.push_back(std::string(ok ? "ok: " : "not yet: ") + (ok ? good : bad));
        return ok;
    };

    if (id == "train") {
        if (!in.answer) throw Error("give your answer with --answer");
        const double truth = escape_rph(c);
        r.numbers = {{"your answer", num(*in.answer, 2)}, {"the train gives", num(truth, 2)}};
        r.passed = pass(std::abs(*in.answer - truth) <= 0.005 * truth, "the escape wheel turns " + num(truth, 1) + " times an hour: " + num(c.beat_rate_vph, 0) + " beats / (2 x " + std::to_string(c.train.escape_teeth) + " teeth).",
                        "beats per hour divided by two beats per tooth and by the tooth count; compare with `ohs kinematics PROJECT`.");
    } else if (id == "timegrapher") {
        if (!in.answer) throw Error("give your answer with --answer");
        const auto p = dynamics::wind_point(c.calibre, c.chain, 0.5, dynamics::Position::CrownUp);
        r.numbers = {{"your answer", sgn(*in.answer)}, {"the simulation gives", sgn(p.rate_s_per_day)}};
        r.passed = pass(std::abs(*in.answer - p.rate_s_per_day) <= 1.0, "crown up, half wound the rate is " + sgn(p.rate_s_per_day) + " s/day.",
                        "run `ohs simulate PROJECT --position crown_up --wind 0.5` and read the rate; it must be within 1 s/day of your answer.");
    } else if (id == "hairspring") {
        const StudyOutcome o = run_learner_study(c, in.study_toml, {"hairspring_length_mm", "hairspring_width_mm", "hairspring_thickness_mm", "hairspring_youngs_gpa", "balance_inertia_mgcm2"},
                                                 "the hairspring and the balance inertia");
        const double shift = metric(o.result, "rate, dial up, half wind").delta();
        const double damp = metric(o.result, "amplitude, full wind").delta();
        r.numbers = {{"rate shift", sgn(shift, 2) + " s/day"}, {"amplitude change", sgn(damp, 2) + " deg"}};
        for (const auto& p : o.problems) r.feedback.push_back("not yet: " + p);
        bool ok = o.problems.empty();
        ok &= pass(std::abs(shift - 10.0) <= 1.5, "the dial-up rate moved " + sgn(shift, 1) + " s/day.",
                   "the rate moved " + sgn(shift, 1) + " s/day, not +10: a shorter spring runs faster, by about 432 s/day per percent of length (half the percentage, times 86400).");
        ok &= pass(std::abs(damp) < 3.0, "the amplitude hardly changed.", "the amplitude moved " + sgn(damp, 1) + " deg: a regulation should not change the energy balance.");
        r.passed = ok;
    } else if (id == "mainspring") {
        const StudyOutcome o = run_learner_study(c, in.study_toml, {"barrel_torque_nmm", "barrel_turns"}, "barrel_torque_nmm and barrel_turns");
        const auto& t = o.result.modified_result;
        const auto& b = o.result.baseline_result;
        const double gain = t.full.amplitude_deg - b.full.amplitude_deg;
        const double reserve_loss = b.power_reserve_h - t.power_reserve_h;
        r.numbers = {{"amplitude, fully wound", num(t.full.amplitude_deg, 1) + " deg (" + sgn(gain, 1) + ")"}, {"power reserve", num(t.power_reserve_h, 1) + " h (" + sgn(-reserve_loss, 1) + ")"}};
        for (const auto& p : o.problems) r.feedback.push_back("not yet: " + p);
        bool ok = o.problems.empty();
        ok &= pass(gain >= 8.0, "amplitude gained " + num(gain, 1) + " deg.", "amplitude gained only " + num(gain, 1) + " deg: raise barrel_torque_nmm further (the gain is less than proportional).");
        ok &= pass(t.full.amplitude_deg <= 315.0, "the amplitude stays under 315 deg.", "amplitude " + num(t.full.amplitude_deg, 1) + " deg is above 315: the roller jewel would knock the guard pin.");
        ok &= pass(reserve_loss <= 2.0, "the power reserve is kept.", "the power reserve fell by " + num(reserve_loss, 1) + " h: keep enough barrel turns.");
        r.passed = ok;
    } else if (id == "escapement") {
        const StudyOutcome o = run_learner_study(c, in.study_toml, {"drop_angle_deg", "impulse_efficiency", "unlock_friction_ratio", "unlock_angle_deg", "lift_angle_deg", "impulse_lead_deg", "escape_wheel_inertia_mgcm2"},
                                                 "drop_angle_deg, impulse_efficiency, unlock_friction_ratio, unlock_angle_deg, lift_angle_deg, impulse_lead_deg and escape_wheel_inertia_mgcm2");
        const auto& t = o.result.modified_result;
        const auto& b = o.result.baseline_result;
        const double gain = t.full.amplitude_deg - b.full.amplitude_deg;
        r.numbers = {{"amplitude, fully wound", num(t.full.amplitude_deg, 1) + " deg (" + sgn(gain, 1) + ")"}, {"isochronism slope", num(t.isochronism_s_per_day_per_deg, 4) + " s/day per deg"},
                     {"wheel to balance efficiency", num(t.budget.efficiency * 100, 0) + " %"}};
        for (const auto& p : o.problems) r.feedback.push_back("not yet: " + p);
        bool ok = o.problems.empty();
        ok &= pass(gain >= 4.0, "amplitude gained " + num(gain, 1) + " deg.", "amplitude gained only " + num(gain, 1) + " deg: cut a loss (smaller drop, less unlocking friction, a lighter escape wheel) or raise the impulse efficiency.");
        ok &= pass(std::abs(t.isochronism_s_per_day_per_deg) <= 0.05, "isochronism is still good.",
                   "the rate now changes " + num(t.isochronism_s_per_day_per_deg, 3) + " s/day per degree of amplitude: that change cost isochronism.");
        r.passed = ok;
    } else if (id == "jewelling") {
        if (!in.answer) throw Error("give your hole diameter in mm with --answer");
        const double d_jewel = 2.0, hole = *in.answer;
        const double interference = d_jewel - hole;
        const fea::Material& plate = fea::find_material("brass_leaded");
        const fea::Material& ruby = fea::find_material("ruby_synthetic");
        const double p = interference > 0 ? jewel_pressure(interference, d_jewel / 2, 4.0, plate, ruby) : 0.0;
        r.numbers = {{"hole", num(hole, 4) + " mm"}, {"interference", num(interference * 1000, 1) + " µm"}, {"contact pressure", num(p, 0) + " MPa"}};
        // Run the same DFM rule the rest of the tool uses, on a plate with this seat.
        manufacturing::PartGeometry g;
        g.name = "plate";
        g.thickness = 0.5;
        mesh::Ring outer{{0, 0}, {20, 0}, {20, 10}, {0, 10}}, seat;
        // An inscribed 360-gon scaled so that its area is exactly that of the circle: the rule measures a hole by its area.
        const double polygon_radius = hole / 2 * std::sqrt(2 * kPi / (360 * std::sin(2 * kPi / 360)));
        for (int i = 0; i < 360; ++i) seat.push_back({5 + polygon_radius * std::cos(2 * kPi * i / 360), 5 + polygon_radius * std::sin(2 * kPi * i / 360)});
        g.region = {{outer, seat}};
        const auto findings = manufacturing::run_dfm(manufacturing::find_process("jewelling"), g);
        const bool in_window = !findings.empty() && findings.front().severity == manufacturing::Severity::Info;
        r.passed = pass(in_window, "a " + num(hole, 4) + " mm hole gives " + num(interference * 1000, 1) + " µm of interference.",
                        findings.empty() ? "that hole is not close enough to a 2.0 mm jewel to count as its seat." : findings.front().message + "; " + findings.front().fix);
    } else if (id == "skeletonise") {
        double w = 0, h = 0;
        if (!parse_cutout(in.cutout, w, h)) throw Error("give the window as WIDTHxHEIGHT in mm, e.g. --cutout 10x5");
        if (w > 12 || h > 6) {
            r.feedback.push_back("not yet: a window of " + num(w, 1) + " x " + num(h, 1) + " mm would reach the pivot holes or the plate's edge; keep it within 12 x 6 mm.");
            return r;
        }
        const auto base = bridge_case(0, 0);
        const auto cut = bridge_case(w, h);
        const double saved = 1.0 - cut.mass_g / base.mass_g;
        r.numbers = {{"mass", num(cut.mass_g, 3) + " g (" + num(saved * 100, 1) + " % less)"}, {"peak von Mises stress", num(cut.max_von_mises_mpa, 1) + " MPa"}, {"safety factor", num(cut.safety_factor, 1)}};
        bool ok = true;
        ok &= pass(saved >= 0.20, "mass is " + num(saved * 100, 1) + " % lower.", "only " + num(saved * 100, 1) + " % of the mass is gone: make the window larger.");
        ok &= pass(cut.safety_factor >= 2.0, "the safety factor is " + num(cut.safety_factor, 1) + ".", "the safety factor fell to " + num(cut.safety_factor, 1) + ": the frame is too slender, make the window smaller or shorter.");
        r.passed = ok;
    }
    return r;
}

std::string reference_solution(const std::string& id, const Context& c) {
    find_lesson(id);
    if (id == "train") return num(escape_rph(c), 1);
    if (id == "timegrapher") return num(dynamics::wind_point(c.calibre, c.chain, 0.5, dynamics::Position::CrownUp).rate_s_per_day, 1);
    if (id == "hairspring") return study_text({{"hairspring_length_mm", std::pow(1.0 + 10.0 / 86400.0, -2.0)}}, "10 s/day faster");
    if (id == "mainspring") return study_text({{"barrel_torque_nmm", 1.12}}, "stronger mainspring");
    if (id == "escapement") return study_text({{"drop_angle_deg", 0.5}, {"unlock_friction_ratio", 0.7}}, "less drop and unlocking friction");
    if (id == "jewelling") return "1.990";
    return "10x5";
}

}  // namespace ohs::lessons
