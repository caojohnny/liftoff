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

const std::vector<liftoff::vector> &liftoff::body::get_d_mot() const {
    return d_mot;
}

void liftoff::body::set_component(int derivative, const liftoff::vector &component) {
    if (d_mot.size() < derivative + 1) {
        return;
    }

    liftoff::vector &cur{d_mot[derivative]};
    cur.set(component);
}
