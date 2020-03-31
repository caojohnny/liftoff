#include "velocity_driven_body.h"

namespace liftoff {
    static const d_idx_t VELOCITY_DRIVER_IDX = 1;

    velocity_driven_body::velocity_driven_body(double body_mass, int body_derivatives, double body_time_step) :
            driven_body(body_mass, VELOCITY_DRIVER_IDX, body_derivatives, body_time_step) {
    }

    void velocity_driven_body::set_position(const vector &position) {
        set_component(0, position);
    }

    void velocity_driven_body::set_velocity(const vector &velocity) {
        set_component(1, velocity);
    }
}
