#include <liftoff-physics/body.h>
#include <iostream>
#include <cmath>
#include "liftoff-physics/drag.h"
#include "telemetry_flight_profile.h"
#include "thrust.h"
#include "rocket.h"
#include "recording_vdb.h"
#include "data_plotter.h"
#include "pidf_controller.h"
#include "velocity_flight_profile.h"
#include <TAxis.h>
#include <TSystem.h>
#include <TROOT.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <gmpxx.h>

static const double TICKS_PER_SEC = 1;
static const double TIME_STEP = 1.0 / TICKS_PER_SEC;

static const double ACCEL_G = 9.80665;

// Coefficient of drag
// https://space.stackexchange.com/questions/16883/whats-the-atmospheric-drag-coefficient-of-a-falcon-9-at-launch-sub-sonic-larg#16885
static const double F9_CD = 0.25;
// Frontal surface area, m^2
// https://www.spacex.com/sites/spacex/files/falcon_users_guide_10_2019.pdf
static const double F9_A = M_PI * 2.6 * 2.6;

// Merlin 1D Max Thrust @ SL, N
// https://www.spacex.com/sites/spacex/files/falcon_users_guide_10_2019.pdf
static const double MERLIN_MAX_THRUST = 854000;
// Merlin 1D I_sp (or as good of a guess as people get), s
// https://en.wikipedia.org/wiki/Falcon_Heavy#cite_note-5
static const double MERLIN_ISP = 282;
// Merlin 1D nozzle exit area,
// Estimates: https://forum.nasaspaceflight.com/index.php?topic=32983.45
// Estimates: https://www.reddit.com/r/spacex/comments/4icycu/basic_analysis_of_the_merlin_1d_engine/d2x26pn/
// 0.95 m seems to be a fair diameter compromise
static const double MERLIN_A = M_PI * 0.475 * 0.475;

// Required precision for floating point values in order to perform the necessary polynomial
// regressions
// (pretty sure this is actually around 80, but 256 just to be safe :>)
static const int REQ_PRECISION = 256;

static long double to_ticks(int seconds) {
    return seconds * TICKS_PER_SEC;
}

static long double to_ticks(int minutes, int seconds) {
    return (minutes * 60 + seconds) * TICKS_PER_SEC;
}

static double kmh_to_mps(double kmh) {
    return kmh * 1000 / 3600;
}

static double km_to_m(double km) {
    return km * 1000;
}

static int signum(double x) {
    return (x > 0) - (x < 0);
}

// Laplace expansion to compute the determinant of a 2d vector
static mpf_class det(const std::vector<std::vector<mpf_class>> &v) {
    if (v.size() == 2 && v[0].size() == 2) {
        return v[0][0] * v[1][1] - v[0][1] * v[1][0];
    }

    mpf_class result{0, REQ_PRECISION};
    for (int col = 0; col < v[0].size(); ++col) {
        int cofactor_sign = col % 2 == 0 ? 1 : -1;

        std::vector<std::vector<mpf_class>> c;
        for (int c_row = 0, v_row = 1; v_row < v.size(); ++c_row, ++v_row) {
            c.emplace_back();

            for (int v_col = 0; v_col < v[0].size(); ++v_col) {
                if (v_col == col) {
                    continue;
                }

                c[c_row].push_back(v[v_row][v_col]);
            }
        }

        result += v[0][col] * cofactor_sign * det(c);
    }

    return result;
}

// Linear solver using Cramer's rule
// https://neutrium.net/mathematics/least-squares-fitting-of-a-polynomial/
static std::vector<double>
linsolve(int order, const std::vector<std::vector<mpf_class>> &m, const std::vector<mpf_class> &b) {
    const mpf_class &det_m{det(m)};

    std::vector<double> coeffs;
    for (int i = 0; i <= order; ++i) {
        std::vector<std::vector<mpf_class>> m_i;
        for (int mi_row = 0; mi_row <= order; ++mi_row) {
            m_i.emplace_back();

            for (int mi_col = 0; mi_col <= order; ++mi_col) {
                if (mi_col == i) {
                    m_i[mi_row].emplace_back(b[mi_row]);
                } else {
                    m_i[mi_row].push_back(m[mi_row][mi_col]);
                }
            }
        }

        const mpf_class &det_mi{det(m_i)};
        const mpf_class &coeff = det_mi / det_m;
        coeffs.push_back(coeff.get_d());
    }

    return coeffs;
}

// Polynomial regression utilizing a Vandermonde matrix
static std::vector<double> fit(int order, const std::vector<double> &x, const std::vector<double> &y) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x/y are not the same size");
    }

    double n = x.size();

    std::vector<std::vector<mpf_class>> m;
    for (int a_row = 0; a_row <= order; ++a_row) {
        m.emplace_back();

        for (int a_col = 0; a_col <= order; ++a_col) {
            mpf_class sum{0};
            for (int i = 0; i < n; ++i) {
                mpf_class term{x[i]};
                mpf_pow_ui(term.get_mpf_t(), term.get_mpf_t(), a_row + a_col);

                sum += term;
            }

            m[a_row].push_back(sum);
        }
    }

    std::vector<mpf_class> b;
    for (int b_row = 0; b_row <= order; ++b_row) {
        mpf_class sum{0};
        for (int i = 0; i < n; ++i) {
            sum += std::pow(x[i], b_row) * y[i];
        }

        b.push_back(sum);
    }

    return linsolve(order, m, b);
}

// Adapted from: https://stackoverflow.com/questions/15191088/how-to-do-a-polynomial-fit-with-fixed-points
static std::vector<double> fit(int order, const std::vector<double> &x, const std::vector<double> &y,
                               const std::map<double, double> &forced_points) {
    std::vector<std::vector<mpf_class>> m;
    std::vector<mpf_class> b;

    int m_dim = order + 1 + x.size();
    for (int i = 0; i < m_dim; ++i) {
        m.emplace_back();
        b.emplace_back();

        for (int j = 0; j < m_dim; ++j) {
            m[i].emplace_back();
        }
    }

    // TODO

    return linsolve(order, m, b);
}

static double polyval(const std::vector<double> &poly, double x) {
    double val = 0;
    for (int i = 0; i < poly.size(); ++i) {
        val += poly[i] * pow(x, i);
    }

    return val;
}

static std::map<double, double>::iterator
find_event_time(std::map<double, double>::iterator begin, std::map<double, double> &velocities, bool at_drop) {
    double prev_v = begin->second;
    for (auto it = begin; it != velocities.end(); ++it) {
        double v = it->second;
        if (at_drop && v < prev_v || !at_drop && v > prev_v) {
            return it;
        }

        prev_v = v;
    }

    return velocities.end();
}

static void
setup_flight_profile(telemetry_flight_profile &raw, telemetry_flight_profile &fitted, const std::string &path) {
    // JCSAT-18/KACIFIC1

    // This actually isn't a ballistic trajectory (I don't think,
    // I didn't look that closely at the animation) but it should
    // be close enough in theory :)
    // https://everydayastronaut.com/prelaunch-preview-falcon-9-block-5-jcsat-18-kacific-1/
    raw.set_ballistic_range(651000);

    std::ifstream data_file{path};
    if (!data_file.good()) {
        std::cout << "Cannot find file '" << path << "'" << std::endl;
        data_file.close();
        return;
    }

    std::map<double, double> velocity_telem;
    std::map<double, double> altitude_telem;

    std::string line;
    while (std::getline(data_file, line)) {
        const auto &json = nlohmann::json::parse(line);
        double time = json["time"];
        double velocity = json["velocity"];
        double altitude = json["altitude"];

        velocity_telem[time] = velocity;
        altitude_telem[time] = km_to_m(altitude);

        raw.put_velocity(time, velocity);
        raw.put_altitude(time, km_to_m(altitude));
    }
    data_file.close();

    std::vector<double> st;
    double last_unique_time = -1;
    double last_unique_value = -1;
    for (auto it = velocity_telem.begin(); it != velocity_telem.end(); it++) {
        double t = it->first;
        double v = it->second;
        if (v != last_unique_value || it == --velocity_telem.end()) {
            fitted.put_velocity(t, v);

            if (!st.empty()) {
                double slope = (v - last_unique_value) / (t - last_unique_time);
                for (const auto &interp_t : st) {
                    double dt = interp_t - last_unique_time;
                    fitted.put_velocity(interp_t, last_unique_value + slope * dt);
                }
                st.clear();
            }

            last_unique_time = t;
            last_unique_value = v;
        } else {
            st.push_back(t);
        }
    }

    last_unique_time = -1;
    last_unique_value = -1;

    for (auto it = altitude_telem.begin(); it != altitude_telem.end(); it++) {
        double t = it->first;
        double v = it->second;

        if (v != last_unique_value || it == --altitude_telem.end()) {
            // fitted.put_altitude(t, v);
            altitude_telem[t] = v;

            if (!st.empty()) {
                double slope = (v - last_unique_value) / (t - last_unique_time);
                for (const auto &interp_t : st) {
                    double dt = interp_t - last_unique_time;
                    // fitted.put_altitude(interp_t, last_unique_value + slope * dt);
                    altitude_telem[interp_t] = last_unique_value + slope * dt;
                }
                st.clear();
            }

            last_unique_time = t;
            last_unique_value = v;
        } else {
            st.push_back(t);
        }
    }

    auto meco_ptr = find_event_time(++velocity_telem.begin(), velocity_telem, true);
    auto ses_1_ptr = find_event_time(meco_ptr, velocity_telem, false);
    auto seco_1_ptr = find_event_time(ses_1_ptr, velocity_telem, true);

    std::vector<double> events = {meco_ptr->first, ses_1_ptr->first, seco_1_ptr->first};
    std::cout << "meco = " << events[0] << std::endl;
    std::cout << "ses_1 = " << events[1] << std::endl;
    std::cout << "seco_1 = " << events[2] << std::endl;

    int n_events = events.size();

    std::vector<std::vector<double>> times;
    std::vector<std::vector<double>> legs;
    times.reserve(n_events);
    legs.reserve(n_events);
    for (int k = 0; k < n_events; ++k) {
        times.emplace_back();
        legs.emplace_back();
    }

    for (auto &it : altitude_telem) {
        double t = it.first;
        double alt = it.second;
        for (int i = 0; i < n_events; ++i) {
            if (t < events[i]) {
                times[i].push_back(t);
                legs[i].push_back(alt);
                break;
            }
        }
    }

    std::vector<std::vector<double>> alt_fit;
    for (int l = 0; l < n_events; ++l) {
        alt_fit.push_back(fit(5, times[l], legs[l]));
    }

    double launch_min_alt = DBL_MAX;
    for (auto &it : altitude_telem) {
        double t = it.first;
        double alt = it.second;
        for (int i = 0; i < n_events; ++i) {
            if (t < events[i]) {
                alt = polyval(alt_fit[i], t);
                break;
            }
        }

        if (alt < 0) {
            alt = 0;
        }

        if (alt <= launch_min_alt) {
            launch_min_alt = alt;
            continue;
        } else if (launch_min_alt != DBL_MIN) {
            for (auto &fixer_it : altitude_telem) {
                double fixer_time = fixer_it.first;
                if (fixer_time >= t) {
                    break;
                }

                fitted.put_altitude(fixer_time, (fixer_time / t) * launch_min_alt);
            }
            launch_min_alt = DBL_MIN;
        }

        fitted.put_altitude(t, alt);
    }

    bool meco_passed = false;
    double meco_coast_t = *(times[0].end() - 1).base();
    double meco_coast_alt = *(legs[0].end() - 1).base();

    double ses_coast_t = events[1];
    double ses_coast_alt = fitted.get_altitude(ses_coast_t);

    std::ofstream transfer_file{"transfer.csv"};
    transfer_file.clear();
    transfer_file << "time (s),raw altitude (m),fitted altitude (m)\n";
    for (const auto &entry : altitude_telem) {
        transfer_file << entry.first << "," << raw.get_altitude(entry.first) << ","
                      << fitted.get_altitude(entry.first)
                      << "\n";
    }
    transfer_file.close();
}

static liftoff::vector
adjust_velocity(pidf_controller &pidf, const liftoff::vector &cur_v, double mag_v) {
    double target_x_velocity;
    double target_y_velocity;

    if (pidf.get_setpoint() == 0) {
        target_x_velocity = 0;
        target_y_velocity = mag_v;
    } else {
        double error = pidf.compute_error();
        target_y_velocity = error / pidf.get_time_step();
        if (std::abs(target_y_velocity) > mag_v) {
            target_y_velocity = signum(target_y_velocity) * mag_v;
        }

        target_x_velocity = std::sqrt(mag_v * mag_v - target_y_velocity * target_y_velocity);
    }

    return {target_x_velocity, target_y_velocity, 0};
}

static void adjust_altitude(telemetry_flight_profile &fitted, double break_even, double max_time) {
    double last_t = 0;
    double last_alt = 0;
    double v_integral = 0;
    /* for (auto &it : fitted.get_altitudes()) {
        double t = it.first;
        double alt = it.second; */
    for (int i = 0; i < max_time / fitted.get_time_step(); ++i) {
        double t = i * fitted.get_time_step();
        double alt = fitted.get_altitude(t);
        double v = fitted.get_velocity(t);

        double dt = fitted.get_time_step();
        v_integral += v * dt;
        if (t < break_even) {
            fitted.put_altitude(t, v_integral);
        } else {
            double target_error = alt - last_alt;
            double prev_alt = fitted.get_altitude(t - dt);
            double target_alt = prev_alt + target_error;
            if (target_alt >= alt) {
                break;
            }

            fitted.put_altitude(t, target_alt);
        }

        last_t = t;
        last_alt = alt;
    }
}

void run_telemetry_profile(data_plotter *plotter, velocity_flight_profile &result) {
    // Source: https://www.spaceflightinsider.com/hangar/falcon-9/
    const double stage_1_dry_mass_kg = 25600;
    const double stage_1_fuel_mass_kg = 395700;
    const double stage_2_dry_mass_kg = 3900;
    const double stage_2_fuel_mass_kg = 92670;
    const double payload_mass_kg = 6800;

    double total_mass = stage_1_dry_mass_kg + stage_1_fuel_mass_kg +
                        stage_2_dry_mass_kg + stage_2_fuel_mass_kg +
                        payload_mass_kg;

    double time_step = 1;
    recording_vdb body{total_mass, 4, time_step};

    telemetry_flight_profile raw{time_step};
    telemetry_flight_profile fitted{time_step};
    setup_flight_profile(raw, fitted, "./data/data.json");

    std::map<double, double> &raw_velocities = raw.get_velocities();
    // double max_time = (--velocities.end())->first;
    double max_time = 180;

    double last_corrected_time = 0;
    while (true) {
        bool valid = true;
        double last_t = 0;
        double last_alt = 0;
        /* for (const auto &it : fitted.get_altitudes()) {
            double t = it.first;
            double alt = it.second; */
        for (int i = 0; i < max_time / time_step; ++i) {
            double t = i * time_step;
            double alt = fitted.get_altitude(t);

            double dt = t - last_t;
            double target_error = alt - last_alt;
            double target_v = target_error / dt;
            double v = fitted.get_velocity(t);
            if (v < target_v && last_corrected_time < t) {
                last_corrected_time = t;
                adjust_altitude(fitted, t, max_time);

                valid = false;
                break;
            }

            last_t = t;
            last_alt = alt;
        }

        if (valid) {
            break;
        }
    }

    std::cout << "pitch time = " << last_corrected_time << std::endl;

    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};

    // Telemetry
    const liftoff::vector &p{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    const liftoff::vector &j{d_mot[3]};

    auto *p_plot = new TGraph();
    p_plot->SetTitle("Position");
    p_plot->GetYaxis()->SetTitle("Altitude (meters)");
    p_plot->GetXaxis()->SetTitle("Downrange Distance (meters)");
    auto *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    auto *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Acceleration (meters/second^2)");
    auto *j_plot = new TGraph();
    j_plot->SetTitle("Jerk");
    j_plot->GetYaxis()->SetTitle("Y Jerk (meters/second^3)");

    plotter->add_plot(p_plot);
    plotter->add_plot(v_plot);
    plotter->add_plot(a_plot);
    plotter->add_plot(j_plot);

    for (int i = 1; i <= 4; ++i) {
        TVirtualPad *pad = plotter->get_canvas()->GetPad(i);
        pad->SetGridx();
        pad->SetGridy();

        if (i != 1) {
            TGraph *plot = plotter->get_plot(i);
            plot->GetXaxis()->SetTitle("Time (seconds)");
        }
    }

    std::vector<double> recorded_drag;
    recorded_drag.push_back(0);

    pidf_controller pidf{time_step, 0, 0, 0, 0};

    std::vector<double> t_v;
    std::vector<double> vx_v;
    std::vector<double> vy_v;

    int pause_ticks = 0;
    for (int i = 0; i < max_time / time_step; ++i) {
        double cur_time_s = i * time_step;

        // Computation
        body.pre_compute();

        pidf.set_last_state(p.get_y());

        double telem_velocity = fitted.get_velocity(cur_time_s);
        double telem_alt = fitted.get_altitude(cur_time_s);
        if (!std::isnan(telem_velocity) && !std::isnan(telem_alt)) {
            pidf.set_setpoint(telem_alt);

            const liftoff::vector &new_velocity = adjust_velocity(pidf, v, telem_velocity);
            body.set_velocity(new_velocity);
        }

        double drag_x = liftoff::calc_drag_earth(F9_CD, p.get_y(), v.get_x(), F9_A);
        double drag_y = liftoff::calc_drag_earth(F9_CD, p.get_y(), v.get_y(), F9_A);
        liftoff::vector cur_drag{drag_x, drag_y, 0};
        recorded_drag.push_back(cur_drag.get_y());

        body.compute_motion();
        body.post_compute();

        if (!plotter->is_valid()) {
            return;
        }

        // Plotting
        // p_plot->SetPoint(i, p.get_x(), p.get_y());
        p_plot->SetPoint(i, cur_time_s, v.get_x());

        v_plot->SetPoint(i, cur_time_s, v.magnitude());
        // v_plot->SetPoint(i, cur_time_s, v.get_x() == 0 ? M_PI / 2 : std::atan(v.get_y() / v.get_x()));
        v_plot->SetPoint(i, cur_time_s, v.get_y());

        a_plot->SetPoint(i, cur_time_s, a.magnitude());

        j_plot->SetPoint(i, cur_time_s, j.magnitude());
        // j_plot->SetPoint(i, cur_time_s, raw.get_altitude(cur_time_s) - p.get_y());

        std::cout << cur_time_s << ", " << v.magnitude() << ", " << p.get_y() << ", " << pidf.compute_error()
                  << ", "
                  << p.get_x() << ", " << v.get_x() << ", " << v.get_y() << ", " << a.get_x() << ", " << a.get_y()
                  << ", " << a.magnitude() << ", " << j.get_x() << ", " << j.get_y() << ", " << j.magnitude()
                  << ", "
                  << pidf.get_setpoint() << ", " << pidf.get_last_state() << ", " << raw.get_altitude(cur_time_s)
                  << ", " << raw.get_altitude(cur_time_s) - p.get_y() << std::endl;

        t_v.push_back(cur_time_s);
        vx_v.push_back(v.get_x());
        vy_v.push_back(v.get_y());

        plotter_handle_gui(false);

        pause_ticks++;
        /* if (pause_ticks >= 10) {
            plotter->update_plots();
            plotter->await(1000000);

            pause_ticks = 0;
        } */
    }

    plotter->update_plots();
}

void run_test_rocket(data_plotter *plotter) {
    telemetry_flight_profile profile{TIME_STEP};
    telemetry_flight_profile fitted{TIME_STEP};
    setup_flight_profile(profile, fitted, "./data/data.json");

    // https://www.youtube.com/watch?v=sbXgZg9JmkI
    double meco_time_s = 155;

    pidf_controller pidf{TIME_STEP, 0, 0, 0, 0};

    double merlin_p_e = liftoff::calc_pressure_earth(0) * 1000;
    std::vector<engine> engines;
    for (int i = 0; i < 9; ++i) {
        engine e{MERLIN_MAX_THRUST, MERLIN_ISP};
        engines.push_back(e);
    }

    // These numbers come from up there ^^
    const double stage_1_dry_mass_kg = 25600;
    const double stage_1_fuel_mass_kg = 395700;
    const double stage_2_dry_mass_kg = 3900;
    const double stage_2_fuel_mass_kg = 92670;
    const double payload_mass_kg = 6800;

    rocket body{stage_1_dry_mass_kg + stage_2_dry_mass_kg + payload_mass_kg + stage_2_fuel_mass_kg,
                stage_1_fuel_mass_kg,
                engines,
                4, TIME_STEP};

    std::vector<liftoff::vector> &forces = body.get_forces();
    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};

    // Telemetry
    const liftoff::vector &p{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    // const liftoff::vector &j{d_mot[3]};

    auto *p_plot = new TGraph();
    p_plot->SetTitle("Altitude");
    p_plot->GetYaxis()->SetTitle("Altitude (meters)");
    p_plot->GetXaxis()->SetTitle("Downrange Distance (meters)");
    auto *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    auto *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Accleration (meters/second^2)");
    auto *j_plot = new TGraph();
    j_plot->SetTitle("Atmospheric Drag");
    j_plot->GetYaxis()->SetTitle("Drag Force (Newtons)");

    plotter->add_plot(p_plot);
    plotter->add_plot(v_plot);
    plotter->add_plot(a_plot);
    plotter->add_plot(j_plot);

    for (int i = 1; i <= 4; ++i) {
        TVirtualPad *pad = plotter->get_canvas()->GetPad(i);
        pad->SetGridx();
        pad->SetGridy();

        if (i != 1) {
            TGraph *plot = plotter->get_plot(i);
            plot->GetXaxis()->SetTitle("Time (seconds)");
        }
    }

    // Initial state
    liftoff::vector w{0, -ACCEL_G * body.get_mass(), 0};
    liftoff::vector n{0, ACCEL_G * body.get_mass(), 0};
    forces.push_back(w);
    forces.push_back(n);
    forces.resize(4);

    liftoff::vector cached_n = n;

    int pause_ticks = 0;
    long double sim_duration_ticks = to_ticks(200); // to_ticks(15, 0);
    for (int i = 1; i < sim_duration_ticks; ++i) {
        double cur_time_s = i * TIME_STEP;

        // Computation
        body.pre_compute();

        // Normal force computation
        auto find_n = std::find(forces.begin(), forces.end(), cached_n);

        liftoff::vector new_n;
        if (p.get_y() < 0) {
            for (const auto &force : forces) {
                if (force.get_y() < 0) {
                    new_n.add({0, -force.get_y(), 0});
                }
            }

            // Hitting the ground
            if (v.get_y() < 0) {
                body.set_velocity({});
            }
        }

        *find_n = cached_n = new_n;

        // Recompute weight vector
        double cur_mass = body.get_mass();
        liftoff::vector cur_weight = {0, -ACCEL_G * cur_mass, 0};
        forces.at(0) = cur_weight;

        // Recompute drag for new velocity
        double v_mag = v.magnitude();
        double drag_sign = -((v.get_y() > 0) - (v.get_y() < 0));
        double drag_y = liftoff::calc_drag_earth(F9_CD, p.get_y(), v_mag, F9_A);
        liftoff::vector cur_drag{0, drag_sign * drag_y, 0};
        forces.at(2) = cur_drag;

        // Recompute thrust
        std::vector<engine> &cur_engines{body.get_engines()};

        double prop_rem = body.get_prop_mass();
        if (prop_rem <= 0) {
            std::cout << "No propellant (tick=" << i << ")" << std::endl;
        }

        pidf.set_last_state(p.get_y());

        double target_v = profile.get_velocity();
        double target_alt = profile.get_altitude();

        if (cur_time_s >= meco_time_s) {
            for (auto &e : cur_engines) {
                e.set_throttle(0);
            }
        } else if (!std::isnan(target_v) && !std::isnan(target_alt)) {
            pidf.set_setpoint(target_alt);

            double accel = target_v - v.magnitude();
            double f = body.get_mass() * accel;
            double f_pe = f / engines.size();

            for (auto &e : cur_engines) {
                e.set_throttle(f_pe / e.get_max_thrust());
            }
        }

        liftoff::vector cur_thrust;
        for (const auto &e : cur_engines) {
            double throttle_pct = e.get_throttle();
            // Pretty egregious estimate, eek
            // https://www.grc.nasa.gov/www/k-12/airplane/rockth.html
            double free_stream_pressure = liftoff::calc_pressure_earth(p.get_y()) * 1000;
            // cba to figure out how the engine throttle actually affects engine performance here
            double thrust_adjustment = throttle_pct * (merlin_p_e - free_stream_pressure) * MERLIN_A;
            double engine_thrust = e.get_thrust() + thrust_adjustment;
            cur_thrust.add({0, engine_thrust, 0});

            double rate = e.get_prop_flow_rate();
            double mass_flow = rate / ACCEL_G;
            double total_prop_mass = mass_flow * TIME_STEP;
            body.drain_propellant(total_prop_mass);
        }

        if (!std::isnan(target_v)) {
            const liftoff::vector &adjusted_v = adjust_velocity(pidf, v, target_v);
            double thrust_x = adjusted_v.get_x() * cur_thrust.get_y() / target_v;
            double thrust_y = adjusted_v.get_y() * cur_thrust.get_y() / target_v;

            cur_thrust.set({thrust_x, thrust_y, 0});
        }
        forces.at(3) = cur_thrust;

        profile.step();

        body.compute_forces();
        body.compute_motion();
        body.post_compute();

        if (!plotter->is_valid()) {
            return;
        }

        p_plot->SetPoint(i, p.get_x(), p.get_y());
        v_plot->SetPoint(i, cur_time_s, v.magnitude());
        a_plot->SetPoint(i, cur_time_s, a.magnitude());
        j_plot->SetPoint(i, cur_time_s, cur_drag.magnitude());

        plotter_handle_gui(false);

        pause_ticks++;
        if (pause_ticks >= 3000) {
            plotter->update_plots();
            plotter->await(1000000);

            pause_ticks = 0;
        }
    }

    plotter->update_plots();
    std::cout << "Remaining propellant: " << body.get_prop_mass() << "kg" << std::endl;
}

int main() {
    int fake_argc = 0;
    char *fake_argv[1];
    TApplication app("SpaceX JCSAT-18/KACIFC1 Flight Sim", &fake_argc, fake_argv);

    velocity_flight_profile result{TIME_STEP};
    auto *telemetry_plotter = new data_plotter(app, "Flight Data Replay", 2, 2);
    run_telemetry_profile(telemetry_plotter, result);

    /* auto *sim_plotter = new data_plotter(app, "Flight Simulation", 2, 2);
    run_test_rocket(sim_plotter); */

    while (telemetry_plotter->is_valid() /* + sim_plotter->is_valid() */ > 0) {
        plotter_handle_gui(true);
    }

    app.Terminate();

    return 0;
}
