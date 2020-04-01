#include <liftoff-physics/body.h>
#include <iostream>
#include "liftoff-physics/drag.h"
#include "telemetry_flight_profile.h"
#include "thrust.h"
#include "rocket.h"
#include "recording_vdb.h"
#include "data_plotter.h"
#include "pidf_controller.h"
#include <TAxis.h>
#include <TSystem.h>
#include <TROOT.h>

static const double TICKS_PER_SEC = 3;
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

static long double to_ticks(int seconds) {
    return seconds * TICKS_PER_SEC;
}

static long double to_ticks(int minutes, int seconds) {
    return (minutes * 60 + seconds) * TICKS_PER_SEC;
}

static double kmh_to_mps(double kmh) {
    return kmh * 1000 / 3600;
}

static int signum(double x) {
    return (x > 0) - (x < 0);
}

static telemetry_flight_profile setup_flight_profile(telemetry_flight_profile &profile) {
    // JCSAT-18/KACIFIC1

    // This actually isn't a ballistic trajectory (I don't think,
    // I didn't look that closely at the animation) but it should
    // be close enough in theory :)
    // https://everydayastronaut.com/prelaunch-preview-falcon-9-block-5-jcsat-18-kacific-1/
    profile.set_ballistic_range(651000);

    profile.put_velocity(5, 28);
    profile.put_velocity(10, 98);
    profile.put_velocity(15, 178);
    profile.put_velocity(20, 264);
    profile.put_velocity(25, 355);
    profile.put_velocity(30, 458);
    profile.put_velocity(35, 567);
    profile.put_velocity(40, 684);
    profile.put_velocity(45, 816);
    profile.put_velocity(50, 931);
    profile.put_velocity(55, 1013);
    profile.put_velocity(60, 1109);
    profile.put_velocity(65, 1275);
    profile.put_velocity(70, 1454);
    profile.put_velocity(75, 1658);
    profile.put_velocity(80, 1868);
    profile.put_velocity(85, 2106);
    profile.put_velocity(90, 2359);
    profile.put_velocity(95, 2644);
    profile.put_velocity(100, 2951);
    profile.put_velocity(105, 3290);
    profile.put_velocity(110, 3649);
    profile.put_velocity(115, 4040);
    profile.put_velocity(120, 4458);
    profile.put_velocity(125, 4896);
    profile.put_velocity(130, 5367);
    profile.put_velocity(135, 5874);
    profile.put_velocity(140, 6428);
    profile.put_velocity(145, 7012);
    profile.put_velocity(150, 7559);
    profile.put_velocity(155, 8139);
    profile.put_velocity(160, 8180);
    profile.put_velocity(165, 8114);
    profile.put_velocity(170, 8148);
    profile.put_velocity(175, 8232);

    profile.put_altitude(5, 0);
    profile.put_altitude(10, 0.1);
    profile.put_altitude(15, 0.3);
    profile.put_altitude(20, 0.6);
    profile.put_altitude(25, 1.0);
    profile.put_altitude(30, 1.6);
    profile.put_altitude(35, 2.3);
    profile.put_altitude(40, 3.2);
    profile.put_altitude(45, 4.2);
    profile.put_altitude(50, 5.3);
    profile.put_altitude(55, 6.6);
    profile.put_altitude(60, 7.9);
    profile.put_altitude(65, 9.4);
    profile.put_altitude(70, 10.9);
    profile.put_altitude(75, 12.7);
    profile.put_altitude(80, 14.6);
    profile.put_altitude(85, 16.7);
    profile.put_altitude(90, 18.8);
    profile.put_altitude(95, 21.2);
    profile.put_altitude(100, 23.6);
    profile.put_altitude(105, 26.2);
    profile.put_altitude(110, 29.1);
    profile.put_altitude(115, 32.0);
    profile.put_altitude(120, 35.2);
    profile.put_altitude(125, 38.5);
    profile.put_altitude(130, 42.1);
    profile.put_altitude(135, 45.9);
    profile.put_altitude(140, 49.9);
    profile.put_altitude(145, 54.1);
    profile.put_altitude(150, 58.5);
    profile.put_altitude(155, 63.1);
    profile.put_altitude(160, 67.7);
    profile.put_altitude(165, 72.0);
    profile.put_altitude(170, 76.2);
    profile.put_altitude(175, 80.2);
    // profile.put_altitude(507, 0);

    return profile;
}

static liftoff::vector adjust_velocity(pidf_controller &pidf, double mag) {
    double target_y_velocity = pidf.compute_error() / pidf.get_time_step();

    liftoff::vector result;
    if (std::abs(target_y_velocity) >= mag) {
        result.set_y(signum(target_y_velocity) * mag);
    } else {
        result.set_x(sqrt(mag * mag - target_y_velocity * target_y_velocity));
        result.set_y(target_y_velocity);
    }

    return result;
}

void run_telemetry_profile(data_plotter *plotter) {
    // Source: https://www.spaceflightinsider.com/hangar/falcon-9/
    const double stage_1_dry_mass_kg = 25600;
    const double stage_1_fuel_mass_kg = 395700;
    const double stage_2_dry_mass_kg = 3900;
    const double stage_2_fuel_mass_kg = 92670;
    const double payload_mass_kg = 6800;

    double total_mass = stage_1_dry_mass_kg + stage_1_fuel_mass_kg +
                        stage_2_dry_mass_kg + stage_2_fuel_mass_kg +
                        payload_mass_kg;

    double time_step = 5;
    recording_vdb body{total_mass, 4, time_step};

    telemetry_flight_profile profile{time_step};
    setup_flight_profile(profile);

    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};

    // Telemetry
    const liftoff::vector &p{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    const liftoff::vector &j{d_mot[3]};

    auto *p_plot = new TGraph();
    p_plot->SetTitle("Position");
    p_plot->GetYaxis()->SetTitle("Altitude (m)");
    p_plot->GetXaxis()->SetTitle("Downrange Distance (m)");
    auto *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    auto *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Accleration (meters/second^2)");
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

    int pause_ticks = 0;
    for (int i = 1; i < 200 / time_step; ++i) {
        // Computation
        body.pre_compute();

        pidf.set_last_state(p.get_y());
        /* if (i <= 175 / 5) {
            j_plot->SetPoint(i, i * time_step, pidf.compute_error());
        } */

        double telem_velocity = profile.get_velocity();
        double telem_alt = profile.get_altitude();
        if (!std::isnan(telem_velocity) && !std::isnan(telem_alt)) {
            pidf.set_setpoint(telem_alt * 1000);

            double mag = kmh_to_mps(telem_velocity);
            const liftoff::vector &new_velocity = adjust_velocity(pidf, mag);
            body.set_velocity(new_velocity);
        }

        profile.step();

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
        double cur_time_s = i * time_step;
        p_plot->SetPoint(i, p.get_x(), p.get_y());
        v_plot->SetPoint(i, cur_time_s, v.magnitude());
        a_plot->SetPoint(i, cur_time_s, a.magnitude());
        j_plot->SetPoint(i, cur_time_s, j.magnitude());

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
    const liftoff::vector &y{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    // const liftoff::vector &j{d_mot[3]};

    auto *y_plot = new TGraph();
    y_plot->SetTitle("Altitude");
    y_plot->GetYaxis()->SetTitle("Altitude (meters)");
    auto *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    auto *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Accleration (meters/second^2)");
    auto *j_plot = new TGraph();
    j_plot->SetTitle("Atmospheric Drag");
    j_plot->GetYaxis()->SetTitle("Drag Force (Newtons)");

    plotter->add_plot(y_plot);
    plotter->add_plot(v_plot);
    plotter->add_plot(a_plot);
    plotter->add_plot(j_plot);

    for (int i = 1; i <= 4; ++i) {
        TVirtualPad *pad = plotter->get_canvas()->GetPad(i);
        pad->SetGridx();
        pad->SetGridy();

        TGraph *plot = plotter->get_plot(i);
        plot->GetXaxis()->SetTitle("Time (seconds)");
    }

    // Initial state
    liftoff::vector w{0, -ACCEL_G * body.get_mass(), 0};
    liftoff::vector n{0, ACCEL_G * body.get_mass(), 0};
    forces.push_back(w);
    forces.push_back(n);
    forces.resize(4);

    liftoff::vector cached_n = n;

    int pause_ticks = 0;
    long double sim_duration_ticks = to_ticks(15, 0);
    for (int i = 1; i < sim_duration_ticks; ++i) {
        // Computation
        body.pre_compute();

        // Normal force computation
        auto find_n = std::find(forces.begin(), forces.end(), cached_n);

        liftoff::vector new_n;
        if (y.get_y() < 0) {
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
        double drag_y = liftoff::calc_drag_earth(F9_CD, y.get_y(), v_mag, F9_A);
        liftoff::vector cur_drag{0, drag_sign * drag_y, 0};
        forces.at(2) = cur_drag;

        // Recompute thrust
        std::vector<engine> &cur_engines{body.get_engines()};

        double prop_rem = body.get_prop_mass();
        if (prop_rem <= 0) {
            std::cout << "No propellant (tick=" << i << ")" << std::endl;
        }

        if (i >= 0) {
            for (auto &e : cur_engines) {
                e.set_throttle(.9);
            }
        }

        if (i >= to_ticks(10)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.73);
            }
        }

        if (i > to_ticks(20)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.85);
            }
        }

        if (i > to_ticks(40)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.85);
            }
        }

        if (i > to_ticks(60)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.75);
            }
        }

        if (i > to_ticks(80)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.93);
            }
        }

        if (i > to_ticks(90)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.95);
            }
        }

        if (i > to_ticks(100)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.97);
            }
        }

        if (i > to_ticks(120)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.93);
            }
        }

        if (i > to_ticks(130)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.89);
            }
        }

        if (i > to_ticks(140)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.83);
            }
        }

        if (i > to_ticks(150)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.77);
            }
        }

        if (i > to_ticks(160)) {
            for (auto &e : cur_engines) {
                e.set_throttle(0);
            }
        }

        if (i > to_ticks(630)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.6);
            }
        }

        if (i > to_ticks(632)) {
            for (auto &e : cur_engines) {
                e.set_throttle(0);
            }
        }

        if (i > to_ticks(640)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.2);
            }
        }

        if (i > to_ticks(648)) {
            for (auto &e : cur_engines) {
                e.set_throttle(0);
            }
        }

        if (i > to_ticks(650)) {
            for (auto &e : cur_engines) {
                e.set_throttle(.2);
            }
        }

        if (i > to_ticks(658)) {
            for (auto &e : cur_engines) {
                e.set_throttle(0);
            }
        }

        liftoff::vector cur_thrust;
        for (auto &e : cur_engines) {
            double throttle_pct = e.get_throttle();
            // Pretty egregious estimate, eek
            // https://www.grc.nasa.gov/www/k-12/airplane/rockth.html
            double free_stream_pressure = liftoff::calc_pressure_earth(y.get_y()) * 1000;
            // cba to figure out how the engine throttle actually affects engine performance here
            double thrust_adjustment = throttle_pct * (merlin_p_e - free_stream_pressure) * MERLIN_A;
            double engine_thrust = e.get_thrust() + thrust_adjustment;
            cur_thrust.add({0, engine_thrust, 0});

            double rate = e.get_prop_flow_rate();
            double mass_flow = rate / ACCEL_G;
            double total_prop_mass = mass_flow * TIME_STEP;
            body.drain_propellant(total_prop_mass);
        }

        forces.at(3) = cur_thrust;

        body.compute_forces();
        body.compute_motion();
        body.post_compute();

        if (!plotter->is_valid()) {
            return;
        }

        double cur_time_s = i * TIME_STEP;
        y_plot->SetPoint(i, cur_time_s, y.get_y());
        v_plot->SetPoint(i, cur_time_s, v.get_y());
        a_plot->SetPoint(i, cur_time_s, a.get_y());
        j_plot->SetPoint(i, cur_time_s, cur_drag.get_y());

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

    auto *telemetry_plotter = new data_plotter(app, "Flight Data Replay", 2, 2);
    run_telemetry_profile(telemetry_plotter);

    // auto *sim_plotter = new data_plotter(app, "Flight Simulation", 2, 2);
    // run_test_rocket(sim_plotter);

    while (telemetry_plotter->is_valid() /* + sim_plotter->is_valid() */ > 0) {
        plotter_handle_gui(true);
    }

    app.Terminate();

    return 0;
}
