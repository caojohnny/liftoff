#include "body.h"

liftoff::body::body(double mass, int derivatives) : mass(mass) {
    for (int i = 0; i < derivatives; ++i) {
        auto *vec(new liftoff::vector());
        d_mot.push_back(vec);
    }
}

double liftoff::body::get_mass() const {
    return mass;
}

void liftoff::body::set_mass(double mass) {
    body::mass = mass;
}

const std::vector<liftoff::vector *> &liftoff::body::get_forces() const {
    return forces;
}

const std::vector<liftoff::vector *> &liftoff::body::get_d_mot() const {
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

liftoff::body_impl::body_impl(double mass) : body(mass, 4), driver_idx(2) {
}

liftoff::body_impl::body_impl(double mass, int derivatives, int driver_idx) : body(mass, derivatives),
                                                                              driver_idx(driver_idx) {
}

liftoff::body_impl::~body_impl() {
    for (int i = 0; i < d_mot.size(); ++i) {
        delete(d_mot[i]);
    }
}

void liftoff::body_impl::set_position(const liftoff::vector &position) {
    if (d_mot.capacity() < 1) {
        return;
    }
    
    liftoff::vector &pos(*d_mot[0]);
    pos.set(position);
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
        net_force.add(*force);
    }

    liftoff::vector &accel(*d_mot[2]);
    accel.set(net_force.div(mass));
}

void liftoff::body_impl::compute_motion(double elapsed_time) {
    std::vector<liftoff::vector *> old_d_mot = d_mot;

    // Drive derivatives
    for (int i = driver_idx + 1; i < d_mot.size(); ++i) {
        liftoff::vector cur_driving_vec(*old_d_mot[i - 1]);
        d_mot[i]->add(cur_driving_vec.mul(elapsed_time));
    }

    // Drive integrals
    for (int i = driver_idx - 1; i >= 0; --i) {
        liftoff::vector cur_driving_vec(*old_d_mot[i + 1]);
        d_mot[i]->add(cur_driving_vec.mul(elapsed_time));
    }
}

void liftoff::body_impl::post_compute() {
    body::post_compute();
}
