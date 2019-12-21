#include <iomanip>
#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>
#include <liftoff-physics/force_driven_body.h>

static const long long TICKS_PER_SEC = 1000000;
static const double TIME_STEP = 1.0 / TICKS_PER_SEC;

namespace mpl = matplotlibcpp;

int main() {
    std::cout << std::setprecision(16);

    liftoff::force_driven_body body{5, 4};
    liftoff::vector w{0, -9.8 * body.get_mass(), 0};
    std::vector<liftoff::vector> &forces = body.get_forces();

    body.set_position({0, 20, 0});
    body.set_velocity({1, 20, 0});
    forces.push_back(w);

    int complete_ticks = 0;
    std::vector<double> time;
    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> v_y;
    for (long long i = 1; i < LONG_LONG_MAX; ++i) {
        body.pre_compute();
        body.compute_forces(TIME_STEP);
        body.compute_motion(TIME_STEP);
        body.post_compute();

        const std::vector<liftoff::vector> &d_mot{body.get_d_mot()};
        const liftoff::vector &pos{d_mot[0]};
        const liftoff::vector &v{d_mot[1]};
        if (pos.get_y() <= 0 && complete_ticks == 0) {
            std::cout << "Hit the ground at " << i << " ticks or " << (i * TIME_STEP) << " sec" << std::endl;
            std::cout << "v_f = " << v.magnitude() << std::endl;

            body.set_velocity({});
            body.drive_derivatives(1, TIME_STEP);

            complete_ticks++;
        }

        if (complete_ticks > 0) {
            complete_ticks++;

            liftoff::vector net_force;
            for (const auto &force : forces) {
                net_force.add(force);
            }

            forces[forces.size() - 1] = {0, -net_force.get_y(), 0};
        }

        if (complete_ticks >= TICKS_PER_SEC) {
            break;
        }

        /* time.push_back(i * TIME_STEP);
        pos_x.push_back(pos.get_x());
        pos_y.push_back(pos.get_y());
        v_y.push_back(v.get_y()); */
    }

    /* mpl::named_plot("X vs Y", pos_x, pos_y);
    mpl::named_plot("Y Velocity", time, v_y);
    mpl::legend();
    mpl::show(); */

    return 0;
}
