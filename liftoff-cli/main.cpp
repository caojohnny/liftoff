#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>
#include "recording_fdb.h"
#include "liftoff-physics/drag.h"
#include "telemetry_flight_profile.h"

namespace mpl = matplotlibcpp;

static const long double TICKS_PER_SEC = 10;
static const long double TIME_STEP = 1.0 / TICKS_PER_SEC;

// https://space.stackexchange.com/questions/16883/whats-the-atmospheric-drag-coefficient-of-a-falcon-9-at-launch-sub-sonic-larg#16885
static const double F9_CD = 0.25;
// https://www.spacex.com/sites/spacex/files/falcon_users_guide_10_2019.pdf
static const double F9_A = M_PI * 2.6 * 2.6;

static long double to_ticks(int seconds) {
    return seconds * TICKS_PER_SEC;
}

static long double to_ticks(int minutes, int seconds) {
    return (minutes * 60 + seconds) * TICKS_PER_SEC;
}

static double to_mps(double kmh) {
    return kmh * 1000 / 3600;
}

static recording_fdb setup_rocket() {
    // Source: https://www.spaceflightinsider.com/hangar/falcon-9/
    const double stage_1_dry_mass_kg = 25600;
    const double stage_1_fuel_mass_kg = 395700;
    const double stage_2_dry_mass_kg = 3900;
    const double stage_2_fuel_mass_kg = 92670;
    const double payload_mass_kg = 6800;

    double total_mass = stage_1_dry_mass_kg + stage_1_fuel_mass_kg +
                        stage_2_dry_mass_kg + stage_2_fuel_mass_kg +
                        payload_mass_kg;
    recording_fdb body{total_mass, 4, (double) TIME_STEP};

    return body;
}

static telemetry_flight_profile setup_flight_profile(telemetry_flight_profile &profile) {
    // JCSAT-18/KACIFIC1
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

    return profile;
}

int main() {
    recording_fdb body{setup_rocket()};
    telemetry_flight_profile profile{static_cast<double>(TIME_STEP)};
    setup_flight_profile(profile);

    // std::vector<liftoff::vector> &forces = body.get_forces();
    const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};
    const liftoff::vector &pos{d_mot[0]};
    const liftoff::vector &v{d_mot[1]};
    const liftoff::vector &a{d_mot[2]};

    const std::vector<double> &time = body.get_elapsed_times();
    const vector_record &pos_data = body.get_data(0);
    const vector_record &v_data = body.get_data(1);
    const vector_record &a_data = body.get_data(2);
    const vector_record &j_data = body.get_data(3);

    // Initial state
    liftoff::vector w{0, -9.8 * body.get_mass(), 0};
    liftoff::vector n{0, 9.8 * body.get_mass(), 0};
    // forces.push_back(w);
    // forces.push_back(n);

    std::vector<double> recorded_drag;
    recorded_drag.push_back(0);

    int complete_ticks = 0;
    for (long double i = 1; i < to_ticks(3, 0); ++i) {
        // Normal force computation
        /* if (pos.get_y() <= 0) {
            forces.erase(std::find(forces.begin(), forces.end(), n), forces.end());

            liftoff::vector net_force;
            for (const auto &force : forces) {
                net_force.add(force);
            }

            forces.emplace_back(0, -net_force.get_y(), 0);
        } */

        // Computation
        body.pre_compute();

        double y_velocity = profile.get_velocity();
        profile.step();
        if (!isnan(y_velocity)) {
            body.set_velocity({0, to_mps(y_velocity), 0});
        }

        double drag_y = liftoff::calc_drag_earth(F9_CD, pos.get_y(), v.magnitude(), F9_A);
        liftoff::vector cur_drag{0, drag_y, 0};
        recorded_drag.push_back(cur_drag.get_y());

        // body.compute_forces();
        body.compute_motion();

        // Completion logic
        if (pos.get_y() <= 0 && complete_ticks == 0) {
            body.set_velocity({});
        }

        body.post_compute();

        // Plotting
        if ((long) i % (long) TICKS_PER_SEC == 0) {
            mpl::clf();
            // mpl::named_plot("X vs Y", pos_data.get_x(), pos_data.get_y());
            // mpl::named_plot("Y Position", time, pos_data.get_y());
            mpl::named_plot("Y Velocity", time, v_data.get_y());
            mpl::named_plot("Y Acceleration", time, a_data.get_y());
            mpl::named_plot("Y Drag", time, recorded_drag);
            // mpl::named_plot("Y Jerk", time, j_data.get_y());
            // mpl::named_plot("X Velocity", time, v_data.get_x());
            // mpl::named_plot("X Acceleration", time, a_data.get_x());
            // mpl::named_plot("X Jerk", time, j_data.get_x());
            mpl::legend();
            mpl::pause(0.0000001);
        }
    }

    mpl::clf();
    // mpl::named_plot("X vs Y", pos_data.get_x(), pos_data.get_y());
    mpl::named_plot("Y Position", time, pos_data.get_y());
    mpl::named_plot("Y Velocity", time, v_data.get_y());
    mpl::named_plot("Y Drag", time, recorded_drag);
    // mpl::named_plot("Y Acceleration", time, a_data.get_y());
    // mpl::named_plot("Y Jerk", time, j_data.get_y());
    // mpl::named_plot("X Velocity", time, v_data.get_x());
    // mpl::named_plot("X Acceleration", time, a_data.get_x());
    // mpl::named_plot("X Jerk", time, j_data.get_x());
    mpl::legend();
    mpl::show();

    return 0;
}
