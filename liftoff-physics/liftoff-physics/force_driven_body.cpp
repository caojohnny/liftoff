#include "force_driven_body.h"

namespace liftoff {
    static const d_idx_t FORCE_DRIVER_IDX = 2;

    force_driven_body::force_driven_body(double db_mass, int db_derivatives, double db_time_step) :
            driven_body(db_mass, FORCE_DRIVER_IDX, db_derivatives, db_time_step) {
    }

    std::vector<vector> &force_driven_body::get_forces() {
        return forces;
    }

    void force_driven_body::set_position(const vector &position) {
        set_component(0, position);
    }

    void force_driven_body::set_velocity(const vector &velocity) {
        set_component(1, velocity);
    }

    void force_driven_body::set_acceleration(const vector &acceleration) {
        set_component(2, acceleration);
    }

    void force_driven_body::pre_compute() {
        // Ensure driver forces are present for the
        // initial conditions
        if (initial) {
            compute_forces();
        }

        driven_body::pre_compute();
    }

    void force_driven_body::compute_forces() {
        if (d_mot.capacity() < 3) {
            return;
        }

        vector net_force;
        for (const auto &force : forces) {
            net_force.add(force);
        }

        vector mass_v{get_mass()};
        set_acceleration(net_force.div(mass_v));
    }
}
