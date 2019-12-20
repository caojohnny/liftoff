#include <iostream>
#include "body.h"

liftoff::body::body(double mass, int derivatives) : mass(mass) {
    for (int i = 0; i < derivatives; ++i) {
        d_mot.emplace_back();
    }
}

double liftoff::body::get_mass() const {
    return mass;
}

void liftoff::body::set_mass(double mass) {
    body::mass = mass;
}

std::vector<liftoff::vector> &liftoff::body::get_forces() {
    return forces;
}

std::vector<liftoff::vector> &liftoff::body::get_d_mot() {
    return d_mot;
}

void liftoff::body::pre_compute() {
}

void liftoff::body::compute_forces(double elapsed_time) {
}

void liftoff::body::compute_motion(double elapsed_time) {
}

void liftoff::body::post_compute() {
}

liftoff::body_impl::body_impl(double mass, int derivatives, int driver_idx) : liftoff::body(mass, derivatives),
                                                                              driver_idx(driver_idx) {
    for (int i = 0; i < derivatives; ++i) {
        prev_state.emplace_back();
    }
}

void liftoff::body_impl::set_position(const liftoff::vector &position) {
    if (d_mot.size() < 1) {
        return;
    }

    if (driver_idx == 0) {
        driver_updated = true;
    }

    liftoff::vector &prev{prev_state[0]};
    liftoff::vector &pos{d_mot[0]};
    prev.set(pos);
    pos.set(position);
}

void liftoff::body_impl::set_velocity(const liftoff::vector &velocity) {
    if (d_mot.size() < 2) {
        return;
    }

    if (driver_idx == 1) {
        driver_updated = true;
    }

    liftoff::vector &prev{prev_state[1]};
    liftoff::vector &v{d_mot[1]};
    prev.set(v);
    v.set(velocity);
}

void liftoff::body_impl::set_acceleration(const liftoff::vector &acceleration) {
    if (d_mot.size() < 3) {
        return;
    }

    if (driver_idx == 2) {
        driver_updated = true;
    }

    liftoff::vector &prev{prev_state[2]};
    liftoff::vector &a{d_mot[2]};
    prev.set(a);
    a.set(acceleration);
}

void liftoff::body_impl::pre_compute() {
    body::post_compute();
}

void liftoff::body_impl::compute_forces(double elapsed_time) {
    if (d_mot.capacity() < 3) {
        return;
    }

    liftoff::vector net_force;
    for (auto &force : forces) {
        net_force.add(force);
    }

    liftoff::vector &cur_accel{d_mot[2]};
    liftoff::vector mass_v{mass};
    set_acceleration(net_force.div(mass_v));
}

void liftoff::body_impl::compute_motion(double elapsed_time) {
    liftoff::vector time_v{elapsed_time};

    std::vector<liftoff::vector> adjusted_mot;

    // Copy everything to the prev state
    for (int i = 0; i < d_mot.size(); ++i) {
        if (!driver_updated) {
            prev_state[i].set(d_mot[i]);
        }
        adjusted_mot.push_back(liftoff::vector{d_mot[i]}.mul(time_v));
    }

    // Drive derivatives
    for (int i = driver_idx + 1; i < d_mot.size(); ++i) {
        liftoff::vector &prev_driving_vec{prev_state[i - 1]};
        liftoff::vector &cur_driving_vec{d_mot[i - 1]};
        d_mot[i].set(cur_driving_vec).sub(prev_driving_vec).div(time_v);
    }

    // Drive integrals
    for (int i = driver_idx - 1; i >= 0; --i) {
        liftoff::vector &cur_driving_vec{adjusted_mot[i + 1]};
        d_mot[i].add(cur_driving_vec);
    }
}

void liftoff::body_impl::post_compute() {
    driver_updated = false;
}
