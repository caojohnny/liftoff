#include "force_driven_body.h"

static const int FORCE_DRIVER_IDX = 2;

liftoff::force_driven_body::force_driven_body(double mass, int derivatives, double time_step) :
        liftoff::driven_body(mass, FORCE_DRIVER_IDX, derivatives, time_step) {
}

std::vector<liftoff::vector> &liftoff::force_driven_body::get_forces() {
    return forces;
}

void liftoff::force_driven_body::set_position(const liftoff::vector &position) {
    set_component(0, position);
}

void liftoff::force_driven_body::set_velocity(const liftoff::vector &velocity) {
    set_component(1, velocity);
}

void liftoff::force_driven_body::set_acceleration(const liftoff::vector &acceleration) {
    set_component(2, acceleration);
}

void liftoff::force_driven_body::compute_forces() {
    if (d_mot.capacity() < 3) {
        return;
    }

    liftoff::vector net_force;
    for (const auto &force : forces) {
        net_force.add(force);
    }

    liftoff::vector mass_v{mass};
    set_acceleration(net_force.div(mass_v));
}
