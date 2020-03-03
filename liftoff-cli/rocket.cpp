#include "rocket.h"

rocket::rocket(double dry_mass, double prop_mass, std::vector<engine> engines, int derivatives, double time_step) :
        recording_fdb(dry_mass, derivatives, time_step), prop_mass(prop_mass), engines(std::move(engines)) {
}

double rocket::get_mass() const {
    return body::get_mass() + prop_mass;
}

double rocket::get_prop_mass() const {
    return prop_mass;
}

void rocket::drain_propellant(double drain_mass) {
    prop_mass -= drain_mass;
}

std::vector<engine> &rocket::get_engines() {
    return engines;
}
