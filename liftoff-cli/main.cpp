#include <iomanip>
#include <mpl/matplotlibcpp.h>
#include <liftoff-physics/body.h>

static const long long TICKS_PER_SEC = 100;
static const double FORCE_TIME_SEC = 0.5;
static const double SEC_PER_TICK = 1.0 / TICKS_PER_SEC;

namespace mpl = matplotlibcpp;

int main() {
    std::cout << std::setprecision(16);

    liftoff::body_impl body{5, 4, 2};
    body.set_position({0, 1.6, 0});
    body.set_velocity({1, 20, 0});

    liftoff::vector w{0, -9.8 * body.get_mass(), 0};
    liftoff::vector n{0, 9.8 * body.get_mass(), 0};
    liftoff::vector initial_force{0, 20 * body.get_mass(), 0};

    std::vector<liftoff::vector> &forces{body.get_forces()};
    forces.push_back(w);
    // forces.push_back(initial_force);

    bool removed = false;
    int complete_ticks = 0;

    std::vector<double> time;
    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> v_y;
    std::vector<double> a_y;
    std::vector<double> j_y;
    for (long long i = 1; i < LONG_LONG_MAX; ++i) {
        if (i > FORCE_TIME_SEC * TICKS_PER_SEC && !removed) {
            forces.erase(std::remove(forces.begin(), forces.end(), initial_force), forces.end());
            removed = true;
        }

        body.pre_compute();
        body.compute_forces(SEC_PER_TICK);
        body.compute_motion(SEC_PER_TICK);
        body.post_compute();

        std::vector<liftoff::vector> &d_mot{body.get_d_mot()};
        liftoff::vector &pos{d_mot[0]};
        liftoff::vector &v{d_mot[1]};
        liftoff::vector &a{d_mot[2]};
        liftoff::vector &j{d_mot[3]};

        if (pos.get_y() <= 0 && complete_ticks == 0) {
            std::cout << "Hit the ground at " << i << " ticks or " << (i * SEC_PER_TICK) << " sec" << std::endl;

            body.set_velocity({});

            complete_ticks++;
        }

        if (complete_ticks > 0) {
            complete_ticks++;

            forces.erase(std::remove(forces.begin(), forces.end(), n), forces.end());

            liftoff::vector net_force;
            for (auto &force : forces) {
                net_force.add(force);
            }

            n.set({0, -net_force.get_y(), 0});
            std::cout << n.to_string() << std::endl;
            forces.push_back(n);
        }

        if (complete_ticks >= TICKS_PER_SEC) {
            break;
        }

        time.push_back(i * SEC_PER_TICK);
        pos_x.push_back(pos.get_x());
        pos_y.push_back(pos.get_y());
        v_y.push_back(v.get_y());
        a_y.push_back(a.get_y());
        j_y.push_back(j.get_y());
    }

    mpl::named_plot("X vs Y", pos_x, pos_y);
    // mpl::named_plot("X Position", time, pos_x);
    // mpl::named_plot("Y Position", time, pos_y);
    mpl::named_plot("Y Velocity", time, v_y);
    mpl::named_plot("Y Acceleration", time, a_y);
    // mpl::named_plot("Y Jerk", time, j_y);
    mpl::legend();
    mpl::show();

    return 0;
}
