#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>
#include "recording_fdb.h"

static const long long TICKS_PER_SEC = 10;
static const long double TIME_STEP = 1.0 / TICKS_PER_SEC;

namespace mpl = matplotlibcpp;

int main() {
    recording_fdb body{5, 4, (double) TIME_STEP};
    std::vector<liftoff::vector> &forces = body.get_forces();
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
    forces.push_back(w);
    body.set_position({0, 20, 0});
    body.set_velocity({1, 20, 0});

    int complete_ticks = 0;
    for (long long i = 1; i < LONG_LONG_MAX; ++i) {
        // Computation
        body.pre_compute();
        body.compute_forces();
        body.compute_motion();

        // Landing logic
        if (pos.get_y() <= 0 && complete_ticks == 0) {
            body.set_velocity({});

            complete_ticks++;
        }

        if (complete_ticks > 0) {
            complete_ticks++;

            forces[forces.size() - 1] = {};

            liftoff::vector net_force;
            for (const auto &force : forces) {
                net_force.add(force);
            }

            forces[forces.size() - 1] = {0, -net_force.get_y(), 0};
        }

        body.post_compute();

        // End logic
        if (complete_ticks >= TICKS_PER_SEC) {
            break;
        }

        mpl::clf();
        mpl::named_plot("X vs Y", pos_data.get_x(), pos_data.get_y());
        mpl::named_plot("Y Velocity", time, v_data.get_y());
        mpl::named_plot("Y Acceleration", time, a_data.get_y());
        // mpl::named_plot("Y Jerk", time, j_data.get_y());
        mpl::named_plot("X Velocity", time, v_data.get_x());
        mpl::named_plot("X Acceleration", time, a_data.get_x());
        // mpl::named_plot("X Jerk", time, j_data.get_x());
        mpl::legend();
        mpl::pause(0.0000001);
    }

    mpl::show();

    return 0;
}
