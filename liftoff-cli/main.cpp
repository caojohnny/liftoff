#include <liftoff-physics/body.h>
#include <iostream>
#include "recording_fdb.h"
#include "liftoff-physics/drag.h"
#include "telemetry_flight_profile.h"
#include "thrust.h"
#include "rocket.h"
#include "recording_vdb.h"
#include "data_plotter.h"
#include <TAxis.h>
#include <TSystem.h>

static const long double TICKS_PER_SEC = 5;
static const long double TIME_STEP = 1.0 / TICKS_PER_SEC;

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

static long double to_ticks(int seconds) {
    return seconds * TICKS_PER_SEC;
}

static long double to_ticks(int minutes, int seconds) {
    return (minutes * 60 + seconds) * TICKS_PER_SEC;
}

static double kmh_to_mps(double kmh) {
    return kmh * 1000 / 3600;
}

static telemetry_flight_profile setup_flight_profile(telemetry_flight_profile &profile) {
    // JCSAT-18/KACIFIC1

    // This actually isn't a ballistic trajectory (I don't think,
    // I didn't look that closely at the animation) but it should
    // be close enough in theory :)
    // https://everydayastronaut.com/prelaunch-preview-falcon-9-block-5-jcsat-18-kacific-1/
    profile.set_ballistic_range(651);

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
    profile.put_altitude(507, 0);

    return profile;
}

void run_telemetry_profile() {
    // Source: https://www.spaceflightinsider.com/hangar/falcon-9/
    const double stage_1_dry_mass_kg = 25600;
    const double stage_1_fuel_mass_kg = 395700;
    const double stage_2_dry_mass_kg = 3900;
    const double stage_2_fuel_mass_kg = 92670;
    const double payload_mass_kg = 6800;

    double total_mass = stage_1_dry_mass_kg + stage_1_fuel_mass_kg +
                        stage_2_dry_mass_kg + stage_2_fuel_mass_kg +
                        payload_mass_kg;

    recording_vdb body{total_mass, 4, static_cast<double>(TIME_STEP)};

    telemetry_flight_profile profile{static_cast<double>(TIME_STEP)};
    setup_flight_profile(profile);

    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};

    // Telemetry
    const liftoff::vector &y{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    const liftoff::vector &j{d_mot[3]};

    // Plotting
    int fake_argc = 0;
    char *fake_argv[0];
    const char *app_name = "SpaceX JCSAT-18/KACIFC1 Flight Sim";
    TApplication app(app_name, &fake_argc, fake_argv);
    data_plotter plotter(app, app_name, 2, 2);

    TGraph *y_plot = new TGraph();
    y_plot->SetTitle("Altitude");
    y_plot->GetYaxis()->SetTitle("Altitude (meters)");
    TGraph *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    TGraph *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Accleration (meters/second^2)");
    TGraph *j_plot = new TGraph();
    j_plot->SetTitle("Jerk");
    j_plot->GetYaxis()->SetTitle("Y Jerk (meters/second^3)");

    plotter.add_plot(y_plot);
    plotter.add_plot(v_plot);
    plotter.add_plot(a_plot);
    plotter.add_plot(j_plot);

    for (int i = 1; i <= 4; ++i) {
        TGraph *plot = plotter.get_plot(i);
        plot->GetXaxis()->SetTitle("Time (seconds)");
    }

    std::vector<double> recorded_drag;
    recorded_drag.push_back(0);

    int complete_ticks = 0;
    int pause_ticks = 0;
    for (long double i = 1; i < to_ticks(200); ++i) {
        // Computation
        body.pre_compute();

        double telem_velocity = profile.get_velocity();
        if (!isnan(telem_velocity)) {
            body.set_velocity({0, kmh_to_mps(telem_velocity), 0});
        }

        profile.step();

        double drag_y = liftoff::calc_drag_earth(F9_CD, y.get_y(), v.magnitude(), F9_A);
        liftoff::vector cur_drag{0, drag_y, 0};
        recorded_drag.push_back(cur_drag.get_y());

        body.compute_motion();

        // Completion logic
        if (y.get_y() < 0 && complete_ticks == 0) {
            body.set_velocity({});
        }

        body.post_compute();

        if (!plotter.is_valid()) {
            break;
        }

        // Plotting
        double cur_time_s = i * TIME_STEP;
        y_plot->SetPoint(i, cur_time_s, y.get_y());
        v_plot->SetPoint(i, cur_time_s, v.get_y());
        a_plot->SetPoint(i, cur_time_s, a.get_y());
        j_plot->SetPoint(i, cur_time_s, j.get_y());

        plotter.ensure_open_loop(false);

        pause_ticks++;
        if (pause_ticks >= 500) {
            plotter.update_plots();
            plotter.await(1000000);

            pause_ticks = 0;
        }
    }

    plotter.update_plots();
    while (plotter.is_valid()) {
        plotter.ensure_open_loop(true);
    }
}

void run_test_rocket() {
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
                4, (double) TIME_STEP};

    std::vector<liftoff::vector> &forces = body.get_forces();
    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};

    // Telemetry
    const liftoff::vector &y{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};
    const liftoff::vector &j{d_mot[3]};

    // Plotting
    int fake_argc = 0;
    char *fake_argv[0];
    const char *app_name = "SpaceX JCSAT-18/KACIFC1 Flight Sim";
    TApplication app(app_name, &fake_argc, fake_argv);
    data_plotter plotter(app, app_name, 2, 2);

    TGraph *y_plot = new TGraph();
    y_plot->SetTitle("Altitude");
    y_plot->GetYaxis()->SetTitle("Altitude (meters)");
    TGraph *v_plot = new TGraph();
    v_plot->SetTitle("Velocity");
    v_plot->GetYaxis()->SetTitle("Y Velocity (meters/second)");
    TGraph *a_plot = new TGraph();
    a_plot->SetTitle("Acceleration");
    a_plot->GetYaxis()->SetTitle("Y Accleration (meters/second^2)");
    TGraph *j_plot = new TGraph();
    j_plot->SetTitle("Atmospheric Drag");
    j_plot->GetYaxis()->SetTitle("Drag Force (Newtons)");

    plotter.add_plot(y_plot);
    plotter.add_plot(v_plot);
    plotter.add_plot(a_plot);
    plotter.add_plot(j_plot);

    for (int i = 1; i <= 4; ++i) {
        TGraph *plot = plotter.get_plot(i);
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
    long double sim_duration_ticks = to_ticks(200); // to_ticks(12, 0);
    for (long double i = 1; i < sim_duration_ticks; ++i) {
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
        double drag_y = liftoff::calc_drag_earth(F9_CD, y.get_y(), v_mag, F9_A);
        int signum = (v_mag > 0) - (v_mag < 0);
        liftoff::vector cur_drag{0, signum * drag_y, 0};
        forces.at(2) = cur_drag;

        // Recompute thrust
        liftoff::vector cur_thrust;
        std::vector<engine> &cur_engines{body.get_engines()};

        double prop_rem = body.get_prop_mass();
        if (prop_rem <= 0) {
            std::cout << "No propellant (tick=" << i << ")" << std::endl;
        }

        if (i >= 0) {
            for (auto &e : cur_engines) {
                e.throttle(.81);
            }
        }

        if (i > to_ticks(175)) {
            for (auto &e : cur_engines) {
                e.throttle(0);
            }
        }

        for (auto &e : cur_engines) {
            cur_thrust.add({0, e.get_thrust(), 0});

            double rate = e.get_prop_flow_rate();
            double mass_flow = rate / ACCEL_G;
            double total_prop_mass = mass_flow * TIME_STEP;
            body.drain_propellant(total_prop_mass);
        }
        forces.at(3) = cur_thrust;

        body.compute_forces();
        body.compute_motion();
        body.post_compute();

        if (!plotter.is_valid()) {
            break;
        }

        double cur_time_s = i * TIME_STEP;
        y_plot->SetPoint(i, cur_time_s, y.get_y());
        v_plot->SetPoint(i, cur_time_s, v.get_y());
        a_plot->SetPoint(i, cur_time_s, a.get_y());
        j_plot->SetPoint(i, cur_time_s, drag_y);

        plotter.ensure_open_loop(false);

        pause_ticks++;
        if (pause_ticks >= 1000) {
            plotter.update_plots();
            plotter.await(1000000);

            pause_ticks = 0;
        }
    }

    plotter.update_plots();
    std::cout << "Remaining propellant: " << body.get_prop_mass() << "kg" << std::endl;

    while (plotter.is_valid()) {
        plotter.ensure_open_loop(true);
    }
}

int main() {
    // run_telemetry_profile();
    run_test_rocket();

    return 0;
}
