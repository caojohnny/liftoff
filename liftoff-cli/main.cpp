#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>
#include "recording_fdb.h"
#include "drag.h"

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

static int ev_counter = 0;

static void drive_velocity(recording_fdb &body, long double ticks, int time, int ev_num, double kmh) {
    if (ticks >= to_ticks(time) && ev_counter == ev_num) {
        body.set_velocity({0, to_mps(kmh), 0});
        ev_counter++;

        if (ev_counter == 15) {
            std::cout << "Max-Q @ " << ticks / TICKS_PER_SEC << std::endl;
        }

        if (ev_counter == 31) {
            std::cout << "MECO @ " << ticks / TICKS_PER_SEC << std::endl;
        }
    }
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

int main() {
    recording_fdb body{setup_rocket()};
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

        // JCSAT-18/KACIFIC1
        drive_velocity(body, i, 5, 0, 28);
        drive_velocity(body, i, 10, 1, 98);
        drive_velocity(body, i, 15, 2, 178);
        drive_velocity(body, i, 20, 3, 264);
        drive_velocity(body, i, 25, 4, 355);
        drive_velocity(body, i, 30, 5, 458);
        drive_velocity(body, i, 35, 6, 567);
        drive_velocity(body, i, 40, 7, 684);
        drive_velocity(body, i, 45, 8, 816);
        drive_velocity(body, i, 50, 9, 931);
        drive_velocity(body, i, 55, 10, 1013);
        drive_velocity(body, i, 60, 11, 1109);
        drive_velocity(body, i, 65, 12, 1275);
        drive_velocity(body, i, 70, 13, 1454);
        drive_velocity(body, i, 75, 14, 1658);
        drive_velocity(body, i, 80, 15, 1868);
        drive_velocity(body, i, 85, 16, 2106);
        drive_velocity(body, i, 90, 17, 2359);
        drive_velocity(body, i, 95, 18, 2644);
        drive_velocity(body, i, 100, 19, 2951);
        drive_velocity(body, i, 105, 20, 3290);
        drive_velocity(body, i, 110, 21, 3649);
        drive_velocity(body, i, 115, 22, 4040);
        drive_velocity(body, i, 120, 23, 4458);
        drive_velocity(body, i, 125, 24, 4896);
        drive_velocity(body, i, 130, 25, 5367);
        drive_velocity(body, i, 135, 26, 5874);
        drive_velocity(body, i, 140, 27, 6428);
        drive_velocity(body, i, 145, 28, 7012);
        drive_velocity(body, i, 150, 29, 7559);
        drive_velocity(body, i, 155, 30, 8139);
        drive_velocity(body, i, 160, 31, 8180);
        drive_velocity(body, i, 165, 32, 8114);
        drive_velocity(body, i, 170, 33, 8148);
        drive_velocity(body, i, 175, 34, 8232);

        double drag_y = calc_drag_earth(F9_CD, pos.get_y(), v.get_y(), F9_A);
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
