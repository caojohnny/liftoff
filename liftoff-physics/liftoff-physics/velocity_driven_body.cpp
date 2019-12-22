#include "velocity_driven_body.h"

static const int VELOCITY_DRIVER_IDX = 1;

liftoff::velocity_driven_body::velocity_driven_body(double mass, int derivatives, double time_step) :
        liftoff::driven_body(mass, VELOCITY_DRIVER_IDX, derivatives, time_step) {
}

void liftoff::velocity_driven_body::set_position(const liftoff::vector &position) {
    set_component(0, position);
}

void liftoff::velocity_driven_body::set_velocity(const liftoff::vector &velocity) {
    set_component(1, velocity);
}
