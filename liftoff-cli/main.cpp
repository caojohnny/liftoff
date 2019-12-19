#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>
#include <algorithm>

static const int TICKS_PER_SEC = 100;
static const double FORCE_TIME_SEC = 0.5;
static const double SEC_PER_TICK = 1.0 / TICKS_PER_SEC;

namespace plt = matplotlibcpp;

int main() {
    liftoff::body_impl body(5);

    liftoff::vector initial_position(0, 20, 0);
    body.set_position(initial_position);

    auto *g(new liftoff::vector(0, -9.8 * body.get_mass(), 0));
    auto *initial_force(new liftoff::vector(10, 20 * body.get_mass(), 0));

    auto &forces = const_cast<std::vector<liftoff::vector *> &>(body.get_forces());
    forces.push_back(g);
    forces.push_back(initial_force);

    std::vector<double> time;
    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> v_y;
    std::vector<double> a_y;
    for (int i = 0; i < 999999; ++i) {
        if (i > FORCE_TIME_SEC * TICKS_PER_SEC) {
            forces.erase(std::remove(forces.begin(), forces.end(), initial_force), forces.end());
        }

        body.compute_forces(SEC_PER_TICK);
        body.compute_motion(SEC_PER_TICK);

        auto &d_mot = const_cast<std::vector<liftoff::vector *> &>(body.get_d_mot());
        liftoff::vector &pos(*d_mot[0]);

        if (pos.get_y() <= 0) {
            std::cout << "Hit the ground at " << i << " ticks or " << (i * SEC_PER_TICK) << " sec" << std::endl;
            break;
        }

        time.push_back(i * SEC_PER_TICK);
        pos_x.push_back(d_mot[0]->get_x());
        pos_y.push_back(d_mot[0]->get_y());
        v_y.push_back(d_mot[1]->get_y());
        a_y.push_back(d_mot[2]->get_y());
    }

    plt::named_plot("X vs Y", pos_x, pos_y);
    // plt::named_plot("X Position", time, pos_x);
    // plt::named_plot("Y Position", time, pos_y);
    // plt::named_plot("Y Velocity", time, v_y);
    // plt::named_plot("Y Acceleration", time, a_y);
    plt::legend();
    plt::show();

    return 0;
}
