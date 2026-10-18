// SPDX-License-Identifier: Apache-2.0
#include "dynamics/bench.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "core/error.hpp"
#include "dynamics/timekeeping.hpp"

namespace ohs::dynamics {

namespace {

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> cells;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) {
        const auto b = cell.find_first_not_of(" \t\r");
        const auto e = cell.find_last_not_of(" \t\r");
        cells.push_back(b == std::string::npos ? "" : cell.substr(b, e - b + 1));
    }
    if (!line.empty() && line.back() == ',') cells.emplace_back();
    return cells;
}

double number(const std::string& cell, int line, const char* what) {
    try {
        std::size_t used = 0;
        const double v = std::stod(cell, &used);
        if (used != cell.size()) throw std::invalid_argument("trailing");
        return v;
    } catch (const std::exception&) {
        throw Error("readings line " + std::to_string(line) + ": " + what + " '" + cell + "' is not a number");
    }
}

double rms(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0;
    for (double x : v) s += x * x;
    return std::sqrt(s / static_cast<double>(v.size()));
}

double slope(const std::vector<std::pair<double, double>>& pts) {
    double n = static_cast<double>(pts.size()), sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (auto [x, y] : pts) {
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double den = n * sxx - sx * sx;
    return n < 2 || std::abs(den) < 1e-12 ? 0.0 : (n * sxy - sx * sy) / den;
}

}  // namespace

std::vector<BenchReading> parse_readings_csv(const std::string& text) {
    std::vector<BenchReading> out;
    std::stringstream in(text);
    std::string line;
    int lineno = 0;
    std::vector<std::string> header;
    while (std::getline(in, line)) {
        ++lineno;
        if (line.find_first_not_of(" \t\r") == std::string::npos || line[0] == '#') continue;
        if (header.empty()) {
            header = split(line);
            for (const char* need : {"hours", "position"}) {
                if (std::find(header.begin(), header.end(), need) == header.end()) {
                    throw Error(std::string("readings line ") + std::to_string(lineno) + ": the header needs a '" + need +
                                "' column");
                }
            }
            continue;
        }
        const auto cells = split(line);
        BenchReading r;
        for (std::size_t i = 0; i < header.size() && i < cells.size(); ++i) {
            if (cells[i].empty()) continue;
            const std::string& h = header[i];
            if (h == "hours") r.hours = number(cells[i], lineno, "hours");
            else if (h == "position") r.position = position_from_string(cells[i]);
            else if (h == "amplitude_deg") r.amplitude_deg = number(cells[i], lineno, "amplitude");
            else if (h == "rate_s_per_day") {
                r.rate_s_per_day = number(cells[i], lineno, "rate");
                r.has_rate = true;
            } else if (h == "beat_error_ms") r.beat_error_ms = number(cells[i], lineno, "beat error");
        }
        out.push_back(r);
    }
    if (header.empty()) throw Error("readings file is empty");
    return out;
}

std::vector<BenchReading> load_readings_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw Error("cannot read " + path);
    std::stringstream buf;
    buf << in.rdbuf();
    return parse_readings_csv(buf.str());
}

Fit compare_with_bench(const Calibre& c, const TorqueChain& chain, const std::vector<BenchReading>& readings) {
    Fit fit;
    const double total = full_wind_hours(c, chain);
    std::vector<double> amp_err, rate_err;
    std::vector<std::pair<double, double>> meas_trend, sim_trend;
    double rate_offset = 0;
    int rate_n = 0;
    for (const BenchReading& r : readings) {
        const double wind = total > 0 ? std::clamp(1.0 - r.hours / total, 0.0, 1.0) : 0.5;
        const WindPoint p = wind_point(c, chain, wind, r.position);
        fit.rows.push_back({r, p.amplitude_deg, p.rate_s_per_day});
        if (r.amplitude_deg >= 0) {
            amp_err.push_back(p.amplitude_deg - r.amplitude_deg);
            if (r.position == Position::DialUp) {
                meas_trend.emplace_back(r.hours, r.amplitude_deg);
                sim_trend.emplace_back(r.hours, p.amplitude_deg);
            }
        }
        if (r.has_rate) {
            rate_err.push_back(p.rate_s_per_day - r.rate_s_per_day);
            rate_offset += rate_err.back();
            ++rate_n;
        }
    }
    fit.rms_amplitude_deg = rms(amp_err);
    if (rate_n > 0) {
        rate_offset /= rate_n;  // the regulator setting is free: compare shapes, not the absolute rate
        for (double& e : rate_err) e -= rate_offset;
    }
    fit.rms_rate_s_per_day = rms(rate_err);
    fit.amplitude_slope_measured = slope(meas_trend);
    fit.amplitude_slope_simulated = slope(sim_trend);
    return fit;
}

namespace {

// Golden-section search for the minimum of f(x) on [lo, hi].
template <class F>
double golden(F f, double lo, double hi, int iterations) {
    const double phi = 0.6180339887498949;
    double a = lo, b = hi;
    double c = b - phi * (b - a), d = a + phi * (b - a);
    double fc = f(c), fd = f(d);
    for (int i = 0; i < iterations; ++i) {
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - phi * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + phi * (b - a);
            fd = f(d);
        }
    }
    return 0.5 * (a + b);
}

double amplitude_rms(const Calibre& c, const TorqueChain& chain, const std::vector<BenchReading>& readings) {
    const double total = full_wind_hours(c, chain);
    std::vector<double> err;
    for (const BenchReading& r : readings) {
        if (r.amplitude_deg < 0) continue;
        const double wind = total > 0 ? std::clamp(1.0 - r.hours / total, 0.0, 1.0) : 0.5;
        err.push_back(wind_point(c, chain, wind, r.position).amplitude_deg - r.amplitude_deg);
    }
    return rms(err);
}

}  // namespace

Calibration calibrate(const Calibre& calibre, const TorqueChain& chain, const std::vector<BenchReading>& readings,
                      bool fit_damping) {
    if (std::none_of(readings.begin(), readings.end(), [](const BenchReading& r) { return r.amplitude_deg >= 0; })) {
        throw Error("calibration needs at least one reading with an amplitude");
    }
    Calibration out;
    out.calibre = calibre;
    out.before = compare_with_bench(calibre, chain, readings);

    auto fit_parameter = [&](const char* name, double lo, double hi) {
        double& value = parameter(out.calibre, name);
        auto cost = [&](double logv) {
            double& v = parameter(out.calibre, name);
            v = std::pow(10.0, logv);
            return amplitude_rms(out.calibre, chain, readings);
        };
        const double best = golden(cost, std::log10(lo), std::log10(hi), 16);
        value = std::pow(10.0, best);
        out.fitted.push_back(name);
    };
    fit_parameter("pivot_friction_flat_nm", 1e-12, 1e-8);
    if (fit_damping) {
        fit_parameter("air_damping_nms", 1e-14, 1e-9);
        fit_parameter("pivot_friction_flat_nm", 1e-12, 1e-8);
    }
    out.after = compare_with_bench(out.calibre, chain, readings);
    return out;
}

}  // namespace ohs::dynamics
