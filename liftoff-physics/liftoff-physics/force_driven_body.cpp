#include "force_driven_body.h"

static const int FORCE_DRIVER_IDX = 2;

liftoff::force_driven_body::force_driven_body(double mass, int derivatives) :
        liftoff::driven_body(mass, derivatives) {
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

void liftoff::force_driven_body::pre_compute() {
    driven_body::pre_compute();
}

void liftoff::force_driven_body::compute_forces(double time_step) {
    if (d_mot.capacity() < 3) {
        return;
    }

    liftoff::vector net_force;
    for (const auto &force : forces) {
        net_force.add(force);
    }

    liftoff::vector &cur_accel{d_mot[2]};
    liftoff::vector mass_v{mass};
    set_acceleration(net_force.div(mass_v));
}

void liftoff::force_driven_body::compute_motion(double time_step) {
    drive_derivatives(FORCE_DRIVER_IDX, time_step);
    drive_integrals(FORCE_DRIVER_IDX, time_step);
}

void liftoff::force_driven_body::post_compute() {
    driven_body::post_compute();
}

void liftoff::force_driven_body::drive_derivatives(int driver_idx, double time_step) {
    driven_body::drive_derivatives(driver_idx, time_step);
}

void liftoff::force_driven_body::drive_integrals(int driver_idx, double time_step) {
    driven_body::drive_integrals(driver_idx, time_step);
}
