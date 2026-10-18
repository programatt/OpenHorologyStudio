// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "core/error.hpp"
#include "dynamics/bench.hpp"
#include "dynamics/calibre.hpp"
#include "dynamics/detailed.hpp"
#include "dynamics/oscillator.hpp"
#include "dynamics/study.hpp"
#include "dynamics/timegrapher.hpp"
#include "dynamics/timekeeping.hpp"
#include "dynamics/torque_chain.hpp"
#include "kinematics/kinematics.hpp"
#include "rigging/rigging.hpp"
#include "support.hpp"

using namespace ohs;
using namespace ohs::dynamics;
using Catch::Approx;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

// A calibre with every loss switched off, so the oscillator is an ideal spring and balance.
Calibre lossless() {
    Calibre c = default_calibre();
    c.air_damping_nms = 0;
    c.pivot_friction_flat_nm = 0;
    c.escape_wheel_inertia_mgcm2 = 0;
    return c;
}

std::vector<Event> run_events(const Calibre& c, const RunConditions& cond, RunSummary* summary = nullptr) {
    EventLog log;
    const RunSummary s = simulate_oscillator(c, cond, log.sink());
    if (summary) *summary = s;
    return log.events;
}

std::vector<double> turn_amplitudes(const std::vector<Event>& events) {
    std::vector<double> a;
    for (const Event& e : events) {
        if (e.type == EventType::Turn) a.push_back(std::abs(e.theta));
    }
    return a;
}

// The generated test train: barrel, centre, third, fourth, escape (ratio 6720, 4 meshes).
model::Movement test_movement() {
    rigging::RigOptions opt;
    opt.beat_rate_vph = 25200;
    return rigging::auto_rig("test", test::synthetic_train(), opt);
}

TorqueChain test_chain(const Calibre& c) { return direct_chain(6720, 4, c, 15, 8.0); }

}  // namespace

TEST_CASE("calibre: derived hairspring gives the beat rate, files round-trip") {
    const Calibre c = default_calibre();
    CHECK(c.natural_frequency_hz() == Approx(3.5).epsilon(1e-9));
    CHECK(default_calibre(21600).natural_frequency_hz() == Approx(3.0).epsilon(1e-9));

    // k = E b h^3 / (12 L): doubling the length halves the stiffness, doubling the thickness gives x8.
    Calibre longer = c;
    longer.hairspring_length_mm *= 2;
    CHECK(longer.hairspring_stiffness() == Approx(c.hairspring_stiffness() / 2));
    Calibre thicker = c;
    thicker.hairspring_thickness_mm *= 2;
    CHECK(thicker.hairspring_stiffness() == Approx(c.hairspring_stiffness() * 8));

    const Calibre back = calibre_from_toml(to_toml(c));
    for (const std::string& name : parameter_names()) {
        Calibre a = c, b = back;
        CHECK(parameter(a, name) == Approx(parameter(b, name)).epsilon(1e-12));
    }
    CHECK(back.torque_curve.size() == c.torque_curve.size());

    CHECK_THROWS_AS(calibre_from_toml("[calibre]\nnonsense = 1.0\n"), Error);
    CHECK_THROWS_AS(calibre_from_toml("name = 'x'"), Error);
    CHECK_THROWS_AS(calibre_from_toml("[calibre]\nbarrel_turns = 'many'\n"), Error);
    CHECK_THROWS_AS(position_from_string("sideways"), Error);

    // Without an explicit length the file re-derives it, so changing the inertia keeps the beat rate.
    const Calibre heavy = calibre_from_toml("[calibre]\nbalance_inertia_mgcm2 = 8.0\n");
    CHECK(heavy.natural_frequency_hz() == Approx(3.5).epsilon(1e-9));
}

TEST_CASE("mainspring torque curve interpolates and clamps") {
    Calibre c = default_calibre();
    CHECK(c.mainspring_torque_nmm(1.0) == Approx(c.barrel_torque_nmm));
    CHECK(c.mainspring_torque_nmm(0.0) == Approx(0.0));
    CHECK(c.mainspring_torque_nmm(2.0) == Approx(c.barrel_torque_nmm));
    CHECK(c.mainspring_torque_nmm(0.75) == Approx(c.barrel_torque_nmm * 0.94));
    double last = -1;
    for (int i = 0; i <= 100; ++i) {
        const double t = c.mainspring_torque_nmm(i / 100.0);
        CHECK(t >= last);
        last = t;
    }
}

TEST_CASE("torque chain follows ratio and mesh efficiency") {
    const model::Movement m = test_movement();
    const auto train = kinematics::analyze_train(m, 25200);
    Calibre c = default_calibre();
    const TorqueChain chain = build_chain(m, train, c);

    CHECK(chain.meshes == 4);
    CHECK(chain.speed_ratio == Approx(6720));  // 8 * 8 * 7.5 * 14
    CHECK(chain.escape_teeth == 15);
    CHECK(chain.barrel_period_hours == Approx(8.0));
    REQUIRE(chain.stages.size() == 5);
    CHECK(chain.stages.front().fraction == Approx(1.0));

    const double barrel = c.barrel_torque_nmm * 1e-3;
    CHECK(escape_torque_nm(chain, c, 1.0) == Approx(barrel * std::pow(0.9, 4) / 6720));

    // Power: the escape wheel gets exactly the barrel's power times the mesh efficiencies.
    const double barrel_power = barrel * 2 * std::numbers::pi / (8.0 * 3600.0);
    CHECK(escape_power_w(chain, c, 1.0) == Approx(barrel_power * std::pow(0.9, 4)).epsilon(1e-9));

    // Each mesh's loss is the torque that a lossless mesh would pass on minus what it does.
    const auto rows = torque_table(chain, c, 1.0);
    REQUIRE(rows.size() == 5);
    double previous = rows[0].torque_nmm;
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const double step = chain.stages[i - 1].speed_ratio / chain.stages[i].speed_ratio;
        CHECK(rows[i].torque_nmm == Approx(previous * step * 0.9));
        CHECK(rows[i].lost_nmm == Approx(previous * step * 0.1));
        previous = rows[i].torque_nmm;
    }

    // Pivot friction at the escape wheel is subtracted and cannot make the torque negative.
    c.escape_pivot_loss_nmm = 1e-4;
    CHECK(escape_torque_nm(chain, c, 1.0) < barrel * std::pow(0.9, 4) / 6720);
    c.escape_pivot_loss_nmm = 1.0;
    CHECK(escape_torque_nm(chain, c, 1.0) == 0.0);
}

TEST_CASE("torque chain errors when the escape wheel is not connected") {
    model::Movement m = test_movement();
    m.meshes.clear();
    const auto train = kinematics::analyze_train(m, 25200);
    CHECK_THROWS_AS(build_chain(m, train, default_calibre()), Error);
}

TEST_CASE("verification: undamped balance keeps the analytic period and its energy") {
    const Calibre c = lossless();
    RunConditions cond;
    cond.wheel_torque_nm = 0.0;
    cond.initial_amplitude_deg = 200;
    cond.duration_s = 20;
    const auto events = run_events(c, cond);

    const double period = 2 * std::numbers::pi * std::sqrt(c.inertia() / c.hairspring_stiffness());
    std::vector<double> turns;
    for (const Event& e : events) {
        if (e.type == EventType::Turn) turns.push_back(e.t);
    }
    REQUIRE(turns.size() > 100);
    const double half = (turns.back() - turns.front()) / static_cast<double>(turns.size() - 1);
    CHECK(2 * half == Approx(period).epsilon(1e-6));
    for (double a : turn_amplitudes(events)) CHECK(a / kDeg == Approx(200).epsilon(1e-6));
}

TEST_CASE("verification: viscous damping decays the swing as exp(-c t / 2I)") {
    Calibre c = lossless();
    const double target = 0.9;  // 10 % loss in 30 s
    c.air_damping_nms = -2 * c.inertia() * std::log(target) / 30.0;
    RunConditions cond;
    cond.wheel_torque_nm = 0.0;
    cond.initial_amplitude_deg = 200;
    cond.duration_s = 30;
    RunSummary run;
    const auto events = run_events(c, cond, &run);
    const auto amps = turn_amplitudes(events);
    REQUIRE_FALSE(amps.empty());
    CHECK(amps.back() / (200 * kDeg) == Approx(target).epsilon(5e-3));
}

TEST_CASE("verification: Coulomb friction removes 2 T / k of amplitude per half swing") {
    Calibre c = lossless();
    c.pivot_friction_flat_nm = 2e-9;
    RunConditions cond;
    cond.wheel_torque_nm = 0.0;
    cond.initial_amplitude_deg = 200;
    cond.duration_s = 15;
    const auto amps = turn_amplitudes(run_events(c, cond));
    REQUIRE(amps.size() > 20);
    const double expected = 2 * c.pivot_friction_flat_nm / c.hairspring_stiffness();
    for (std::size_t i = 1; i < amps.size(); ++i) CHECK(amps[i - 1] - amps[i] == Approx(expected).epsilon(2e-3));
}

TEST_CASE("verification: steady amplitude matches the energy balance") {
    const Calibre c = default_calibre();
    const TorqueChain chain = test_chain(c);
    for (double wind : {1.0, 0.5, 0.1}) {
        const double torque = escape_torque_nm(chain, c, wind);
        const SteadyState ss = steady_state(c, torque);
        REQUIRE_FALSE(ss.stalled);
        const double estimate = estimate_amplitude_deg(c, torque, Position::DialUp);
        INFO("wind " << wind);
        CHECK(ss.m.amplitude_deg == Approx(estimate).epsilon(0.03));
        CHECK(ss.m.amplitude_deg > 200);
        CHECK(ss.m.amplitude_deg < 320);
    }
}

TEST_CASE("amplitude grows with drive torque and falls with friction (random calibres)") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int trial = 0; trial < 12; ++trial) {
        Calibre c = default_calibre();
        c.pivot_friction_flat_nm = std::pow(10.0, -10.0 + 1.5 * u(rng));
        c.air_damping_nms = std::pow(10.0, -12.0 + 1.5 * u(rng));
        c.lift_angle_deg = 40 + 20 * u(rng);
        const double t1 = 0.8e-7 + 0.4e-7 * u(rng);
        const double a_low = steady_state(c, t1).m.amplitude_deg;
        const double a_high = steady_state(c, 1.3 * t1).m.amplitude_deg;
        INFO("trial " << trial);
        CHECK(a_high > a_low);
        Calibre rough = c;
        rough.pivot_friction_flat_nm *= 1.5;
        CHECK(steady_state(rough, t1).m.amplitude_deg < a_low);
        // The estimate and the simulation agree on the ordering and roughly on size.
        CHECK(a_low == Approx(estimate_amplitude_deg(c, t1, Position::DialUp)).epsilon(0.05));
    }
}

TEST_CASE("a drive too weak to unlock stalls the balance") {
    const Calibre c = default_calibre();
    const SteadyState ss = steady_state(c, 1e-9);
    CHECK(ss.stalled);
    RunConditions cond;
    cond.wheel_torque_nm = 0;
    cond.duration_s = 600;
    RunSummary run;
    run_events(c, cond, &run);
    CHECK(run.stalled);
    CHECK(run.end_time < 600);
}

TEST_CASE("runs are deterministic") {
    const Calibre c = default_calibre();
    RunConditions cond;
    cond.wheel_torque_nm = 1.0e-7;
    cond.duration_s = 10;
    const auto a = run_events(c, cond);
    const auto b = run_events(c, cond);
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].t == b[i].t);
        CHECK(a[i].theta == b[i].theta);
    }
}

TEST_CASE("event order: unlock, impulse, drop each beat, turn between swings") {
    const Calibre c = default_calibre();
    RunConditions cond;
    cond.wheel_torque_nm = 1.0e-7;
    cond.duration_s = 8;
    const auto events = run_events(c, cond);
    int unlock = 0, impulse = 0, drop = 0, turn = 0;
    for (std::size_t i = 0; i + 1 < events.size(); ++i) {
        const Event& e = events[i];
        unlock += e.type == EventType::UnlockStart;
        impulse += e.type == EventType::ImpulseStart;
        drop += e.type == EventType::Drop;
        turn += e.type == EventType::Turn;
        if (e.type == EventType::UnlockStart) CHECK(events[i + 1].type == EventType::ImpulseStart);
        if (e.type == EventType::ImpulseStart) CHECK(events[i + 1].type == EventType::Drop);
        CHECK(events[i + 1].t >= e.t);
    }
    // 7 beats a second: about 56 of each in 8 s.
    CHECK(std::abs(unlock - 56) <= 2);
    CHECK(std::abs(impulse - 56) <= 2);
    CHECK(std::abs(drop - 56) <= 2);
    CHECK(std::abs(turn - 56) <= 2);
    // Impulse starts exactly half a lift before the (lead-shifted) dead point.
    for (const Event& e : events) {
        if (e.type == EventType::ImpulseStart) {
            CHECK(e.direction * e.theta + c.impulse_lead_deg * kDeg == Approx(-c.lift_angle_deg * kDeg / 2).margin(1e-9));
        }
    }
}

TEST_CASE("timekeeping: beat error from an off-centre hairspring follows 2 d / (A w)") {
    Calibre c = default_calibre();
    c.impulse_lead_deg = 0;
    c.beat_offset_deg = 0.5;
    const double torque = escape_torque_nm(test_chain(c), c, 1.0);
    const SteadyState ss = steady_state(c, torque);
    const double w0 = 2 * std::numbers::pi * c.natural_frequency_hz();
    const double expected_ms = 2 * (0.5 * kDeg) / (ss.m.amplitude_deg * kDeg * w0) * 1e3;
    CHECK(ss.m.beat_error_ms == Approx(expected_ms).epsilon(0.15));

    c.beat_offset_deg = 0;
    CHECK(steady_state(c, torque).m.beat_error_ms < 1e-3);
}

TEST_CASE("timekeeping: positions, gravity and friction") {
    const Calibre c = default_calibre();
    const TorqueChain chain = test_chain(c);
    const PositionalResult r = positional_error(c, chain, 1.0);
    REQUIRE(r.points.size() == 6);
    auto at = [&](Position p) { return r.points[static_cast<std::size_t>(p)]; };
    // Flat positions are identical: gravity has no torque about a vertical staff.
    CHECK(at(Position::DialUp).amplitude_deg == Approx(at(Position::DialDown).amplitude_deg));
    CHECK(at(Position::DialUp).rate_s_per_day == Approx(at(Position::DialDown).rate_s_per_day));
    // The hanging positions carry the weight on the pivot sides: more friction, lower amplitude.
    CHECK(r.flat_to_vertical_amplitude_deg > 5);
    for (Position p : {Position::CrownUp, Position::CrownDown, Position::CrownLeft, Position::CrownRight}) {
        CHECK(at(p).amplitude_deg < at(Position::DialUp).amplitude_deg);
    }
    // Opposite hanging positions put the unpoise on opposite sides of the cycle: opposite rate shifts.
    const double flat = at(Position::DialUp).rate_s_per_day;
    CHECK((at(Position::CrownUp).rate_s_per_day - flat) * (at(Position::CrownDown).rate_s_per_day - flat) < 0);
    CHECK(r.spread_s_per_day > 1);

    // A perfectly poised balance removes the rate differences that gravity causes between the
    // hanging positions (friction still differs from flat).
    Calibre poised = c;
    poised.unpoise_um = 0;
    const PositionalResult q = positional_error(poised, chain, 1.0);
    CHECK(q.points[2].rate_s_per_day == Approx(q.points[3].rate_s_per_day).margin(1e-6));
    CHECK(q.spread_s_per_day < r.spread_s_per_day);
}

TEST_CASE("timekeeping: amplitude falls with the spring, power reserve behaves") {
    const Calibre c = default_calibre();
    const TorqueChain chain = test_chain(c);
    const auto points = amplitude_vs_wind(c, chain, 5);
    REQUIRE(points.size() == 5);
    for (std::size_t i = 1; i < points.size(); ++i) {
        CHECK(points[i].amplitude_deg <= points[i - 1].amplitude_deg + 1e-6);
        CHECK(points[i].hours >= points[i - 1].hours);
    }
    CHECK(points.front().hours == 0.0);
    CHECK(points.back().hours == Approx(50.0));  // 6.25 turns of an 8 h barrel

    const double reserve = power_reserve_hours(c, chain);
    CHECK(reserve > 40);
    CHECK(reserve <= 50.0 + 1e-9);

    Calibre strong = c;
    strong.min_running_amplitude_deg = 100;  // the floor is lower than anything the spring reaches
    CHECK(power_reserve_hours(strong, chain) == Approx(50.0).epsilon(0.02));  // the spring torque only vanishes at the very end

    Calibre demanding = c;
    demanding.min_running_amplitude_deg = 240;  // only the upper part of the wind reaches it
    const double shorter = power_reserve_hours(demanding, chain);
    CHECK(shorter < reserve);
    CHECK(shorter > 0);

    Calibre weak = c;
    weak.barrel_torque_nmm = 0.05;
    CHECK(power_reserve_hours(weak, chain) == 0.0);
}

TEST_CASE("timekeeping: regulating hits the target rate") {
    const Calibre c = default_calibre();
    const TorqueChain chain = test_chain(c);
    const Calibre fast = regulate(c, chain, 12.0, 0.5);
    CHECK(wind_point(fast, chain, 0.5).rate_s_per_day == Approx(12.0).margin(0.05));
    CHECK(fast.hairspring_length_mm < c.hairspring_length_mm);  // a shorter spring runs faster
    const Calibre slow = regulate(c, chain, -30.0, 0.5);
    CHECK(wind_point(slow, chain, 0.5).rate_s_per_day == Approx(-30.0).margin(0.05));
}

TEST_CASE("timekeeping: rate follows sqrt(k/I) when the hairspring changes") {
    const Calibre base = regulate(default_calibre(), test_chain(default_calibre()), 0.0, 0.5);
    const TorqueChain chain = test_chain(base);
    Calibre shorter = base;
    shorter.hairspring_length_mm *= 0.998;
    const double r0 = wind_point(base, chain, 0.5).rate_s_per_day;
    const double r1 = wind_point(shorter, chain, 0.5).rate_s_per_day;
    const double predicted = (std::sqrt(1 / 0.998) - 1) * 86400;
    CHECK(r1 - r0 == Approx(predicted).epsilon(0.03));
}

TEST_CASE("timekeeping: isochronism defect comes from an impulse before the dead point") {
    Calibre c = default_calibre();
    c.impulse_lead_deg = 0;
    c.unlock_friction_ratio = 0;
    const TorqueChain chain = test_chain(c);
    const double centred = isochronism_slope({wind_point(c, chain, 1.0), wind_point(c, chain, 0.1)});
    c.impulse_lead_deg = 12;
    const double early = isochronism_slope({wind_point(c, chain, 1.0), wind_point(c, chain, 0.1)});
    CHECK(std::abs(early - centred) > 0.008);

}

TEST_CASE("timekeeping: the energy budget closes at steady state") {
    const Calibre c = default_calibre();
    const TorqueChain chain = test_chain(c);
    const WindPoint p = wind_point(c, chain, 1.0);
    const EnergyBudget b = energy_budget(c, p.wheel_torque_nm, p.amplitude_deg, Position::DialUp);
    const double net = b.delivered_j - b.lost_unlock_j - b.lost_wheel_hit_j - b.lost_friction_j - b.lost_air_j;
    CHECK(std::abs(net) < 0.03 * b.delivered_j);
    CHECK(b.efficiency > 0.5);
    CHECK(b.efficiency < 1.0);
    CHECK(b.wheel_work_j == Approx(p.wheel_torque_nm * std::numbers::pi / 15));
}

TEST_CASE("virtual timegrapher reads the simulation from sounds alone") {
    Calibre c = default_calibre();
    c.beat_offset_deg = 0.4;
    const TorqueChain chain = test_chain(c);
    RunConditions cond;
    cond.wheel_torque_nm = escape_torque_nm(chain, c, 1.0);
    cond.duration_s = 140;
    const auto events = run_events(c, cond);
    const Measurement truth = measure(events, c, 120, 140);
    const TimegrapherTrace tr = virtual_timegrapher(events, c, 120, 140);

    CHECK(tr.reading.beats == truth.beats);
    CHECK(tr.reading.rate_s_per_day == Approx(truth.rate_s_per_day).margin(0.05));
    CHECK(tr.reading.amplitude_deg == Approx(truth.amplitude_deg).epsilon(0.02));
    CHECK(tr.reading.beat_error_ms == Approx(truth.beat_error_ms).epsilon(0.05));
    CHECK(tr.beats[1].tock);
    CHECK_FALSE(tr.beats[2].tock);
    CHECK(tr.beats[5].interval_ms == Approx(1000.0 * 3600 / 25200).epsilon(0.02));

    const std::string art = trace_ascii(tr);
    CHECK_THAT(art, Catch::Matchers::ContainsSubstring("o"));
    CHECK_THAT(art, Catch::Matchers::ContainsSubstring("x"));
    CHECK_THAT(art, Catch::Matchers::ContainsSubstring("rate"));
    CHECK_THAT(trace_csv(tr), Catch::Matchers::StartsWith("index,time_s,tock"));
    CHECK(trace_ascii(virtual_timegrapher({}, c, 0, 1)).find("not enough") != std::string::npos);
}

TEST_CASE("virtual timegrapher noise is seeded and averages out") {
    const Calibre c = default_calibre();
    RunConditions cond;
    cond.wheel_torque_nm = escape_torque_nm(test_chain(c), c, 1.0);
    cond.duration_s = 200;
    const auto events = run_events(c, cond);
    const auto clean = virtual_timegrapher(events, c, 120, 200);
    const auto a = virtual_timegrapher(events, c, 120, 200, 200.0, 5);
    const auto b = virtual_timegrapher(events, c, 120, 200, 200.0, 5);
    const auto other = virtual_timegrapher(events, c, 120, 200, 200.0, 6);
    CHECK(a.reading.rate_s_per_day == b.reading.rate_s_per_day);
    CHECK(a.reading.rate_s_per_day != other.reading.rate_s_per_day);
    CHECK(a.reading.rate_s_per_day != clean.reading.rate_s_per_day);
    // Over 560 beats, 200 us rms of jitter moves the fitted rate by only a few s/day.
    CHECK(std::abs(a.reading.rate_s_per_day - clean.reading.rate_s_per_day) < 5.0);
    CHECK(a.reading.beat_error_ms > clean.reading.beat_error_ms);
}

TEST_CASE("verification: detailed contact model agrees with the reduced model") {
    Calibre c = default_calibre();
    c.contact_frequency_hz = 6000;  // keeps the test fast; the contact is still 1000x stiffer than the beat
    const TorqueChain chain = test_chain(c);
    for (auto [wind, pos] : {std::pair{1.0, Position::DialUp}, std::pair{0.3, Position::CrownUp}}) {
        const Agreement a = compare_levels(c, escape_torque_nm(chain, c, wind), pos, 20.0, 10.0);
        INFO("wind " << wind << " " << to_string(pos));
        CHECK(std::abs(a.amplitude_diff_percent) < 1.5);
        CHECK(std::abs(a.rate_diff_s_per_day) < 0.5);
        CHECK(a.level3.beats > 60);
    }
}

TEST_CASE("detailed model reproduces the wheel pick-up loss that the reduced model assumes") {
    // A heavy escape wheel takes energy at pick-up and drop; both models must lose amplitude alike.
    Calibre c = default_calibre();
    c.contact_frequency_hz = 6000;
    c.escape_wheel_inertia_mgcm2 = 0.2;  // 4x the default
    const double torque = escape_torque_nm(test_chain(c), c, 1.0);
    const Agreement heavy = compare_levels(c, torque, Position::DialUp, 20.0, 10.0);
    Calibre light = c;
    light.escape_wheel_inertia_mgcm2 = 0.05;
    CHECK(heavy.level2.amplitude_deg < steady_state(light, torque).m.amplitude_deg - 20);
    CHECK(std::abs(heavy.amplitude_diff_percent) < 2.0);
}

TEST_CASE("bench readings parse and report errors with line numbers") {
    const auto r = parse_readings_csv(
        "# my OM10\nhours,position,amplitude_deg,rate_s_per_day,beat_error_ms\n"
        "0,dial_up,285,3.5,0.1\n24, crown_up ,250,,\n");
    REQUIRE(r.size() == 2);
    CHECK(r[0].amplitude_deg == 285);
    CHECK(r[0].has_rate);
    CHECK(r[1].position == Position::CrownUp);
    CHECK_FALSE(r[1].has_rate);
    CHECK(r[1].beat_error_ms < 0);

    CHECK_THROWS_AS(parse_readings_csv(""), Error);
    CHECK_THROWS_AS(parse_readings_csv("position,amplitude_deg\ndial_up,250\n"), Error);
    try {
        parse_readings_csv("hours,position,amplitude_deg\n0,dial_up,250\n5,dial_up,abc\n");
        FAIL("expected a parse error");
    } catch (const Error& e) {
        CHECK_THAT(std::string(e.what()), Catch::Matchers::ContainsSubstring("line 3"));
    }
    CHECK_THROWS_AS(parse_readings_csv("hours,position\n0,upside_down\n"), Error);
}

TEST_CASE("bench calibration recovers the friction of a 'real' movement") {
    // The truth: a movement with twice the default pivot friction. Its timegrapher readings
    // (amplitude through the unwinding, dial up and hanging) are what the calibration sees.
    Calibre truth = default_calibre();
    truth.pivot_friction_flat_nm = 1.7e-9;
    const TorqueChain chain = test_chain(truth);
    std::vector<BenchReading> readings;
    for (double hours : {0.0, 20.0, 40.0}) {
        for (Position p : {Position::DialUp, Position::CrownUp}) {
            const WindPoint w = wind_point(truth, chain, 1.0 - hours / 50.0, p);
            BenchReading r;
            r.hours = hours;
            r.position = p;
            r.amplitude_deg = w.amplitude_deg;
            r.rate_s_per_day = w.rate_s_per_day;
            r.has_rate = true;
            readings.push_back(r);
        }
    }

    const Calibre start = default_calibre();
    const Fit before = compare_with_bench(start, chain, readings);
    CHECK(before.rms_amplitude_deg > 10);

    const Calibration cal = calibrate(start, chain, readings);
    CHECK(cal.calibre.pivot_friction_flat_nm == Approx(1.7e-9).epsilon(0.03));
    CHECK(cal.after.rms_amplitude_deg < 0.5);
    CHECK(cal.after.rms_amplitude_deg < before.rms_amplitude_deg / 10);
    // The trend over the unwinding agrees: measured and simulated amplitude slopes in deg/h.
    CHECK(cal.after.amplitude_slope_simulated == Approx(cal.after.amplitude_slope_measured).epsilon(0.05));
    CHECK(cal.after.amplitude_slope_measured < 0);
    ;
    CHECK(cal.fitted == std::vector<std::string>{"pivot_friction_flat_nm"});
    CHECK_THROWS_AS(calibrate(start, chain, {}), Error);
}

TEST_CASE("what-if: changes apply, parse and explain themselves") {
    Calibre c = default_calibre();
    apply(c, {"balance_inertia_mgcm2", Change::Mode::Scale, 1.1});
    CHECK(c.balance_inertia_mgcm2 == Approx(5.5));
    apply(c, {"lift_angle_deg", Change::Mode::Set, 44});
    CHECK(c.lift_angle_deg == 44);
    apply(c, {"drop_angle_deg", Change::Mode::Add, 0.5});
    CHECK(c.drop_angle_deg == Approx(1.5));
    CHECK_THROWS_AS(apply(c, {"warp_factor", Change::Mode::Set, 9}), Error);

    const StudyDefinition d = study_from_toml(
        "[study]\nname = 'longer spring'\nverify = true\n[[change]]\nparameter = 'hairspring_length_mm'\nscale = 1.01\n"
        "[[change]]\nparameter = 'escape_teeth'\nset = 17\n");
    CHECK(d.name == "longer spring");
    CHECK(d.verify);
    REQUIRE(d.changes.size() == 2);
    CHECK(d.changes[1].mode == Change::Mode::Set);
    CHECK_THROWS_AS(study_from_toml("[study]\nname='x'\n"), Error);
    CHECK_THROWS_AS(study_from_toml("[[change]]\nparameter='x'\nscale=1\nset=2\n"), Error);
    CHECK_THROWS_AS(study_from_toml("[[change]]\nscale=1\n"), Error);
}

TEST_CASE("what-if: a longer hairspring slows the watch by the amount sqrt(k/I) predicts") {
    const Calibre base = default_calibre();
    const TorqueChain chain = test_chain(base);
    StudyDefinition d;
    d.changes = {{"hairspring_length_mm", Change::Mode::Scale, 1.002}};
    const StudyResult r = run_study(base, chain, d);

    auto metric = [&](const std::string& name) -> const MetricRow& {
        for (const MetricRow& m : r.metrics) {
            if (m.name == name) return m;
        }
        FAIL("no metric " << name);
        return r.metrics.front();
    };
    const MetricRow& rate = metric("rate, dial up, half wind");
    CHECK(rate.baseline == Approx(0.0).margin(0.05));  // the baseline is regulated
    const double predicted = (std::sqrt(1 / 1.002) - 1) * 86400;
    CHECK(rate.delta() == Approx(predicted).epsilon(0.03));
    CHECK(metric("amplitude, full wind").delta() == Approx(0).margin(1.0));

    REQUIRE(r.causes.size() == 1);
    CHECK_THAT(r.causes[0], Catch::Matchers::ContainsSubstring("sqrt(k/I)"));
    CHECK_THAT(r.causes[0], Catch::Matchers::ContainsSubstring("s/day predicted"));
    CHECK_THAT(to_text(r), Catch::Matchers::ContainsSubstring("power reserve"));
}

TEST_CASE("what-if: more friction lowers amplitude and the explanation says why") {
    const Calibre base = default_calibre();
    const TorqueChain chain = test_chain(base);
    StudyDefinition d;
    d.regulate = false;
    d.changes = {{"pivot_friction_flat_nm", Change::Mode::Scale, 2.0}};
    const StudyResult r = run_study(base, chain, d);
    CHECK(r.modified_result.full.amplitude_deg < r.baseline_result.full.amplitude_deg - 10);
    CHECK(r.modified_result.power_reserve_h <= r.baseline_result.power_reserve_h);
    CHECK_THAT(r.causes[0], Catch::Matchers::ContainsSubstring("friction and air take"));
    CHECK_FALSE(r.baseline_check.has_value());
}

TEST_CASE("what-if: a fine pallet change moves energy per beat as the geometry says") {
    const Calibre base = default_calibre();
    const TorqueChain chain = test_chain(base);
    StudyDefinition d;
    d.regulate = false;
    d.changes = {{"escape_teeth", Change::Mode::Set, 20}};
    const StudyResult r = run_study(base, chain, d);
    // Wheel work per beat is torque * pi / teeth: 15/20 of the baseline.
    CHECK(r.modified_result.budget.wheel_work_j == Approx(r.baseline_result.budget.wheel_work_j * 15.0 / 20.0));
    CHECK(r.modified_result.full.amplitude_deg < r.baseline_result.full.amplitude_deg);
    CHECK_THAT(r.causes[0], Catch::Matchers::ContainsSubstring("pi/teeth"));
}
