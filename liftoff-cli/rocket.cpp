#include "rocket.h"

rocket::rocket(double rocket_dry_mass, double rocket_prop_mass, std::vector<engine> rocket_engines, int fdb_derivatives,
               double fdb_time_step) :
        liftoff::force_driven_body(rocket_dry_mass, fdb_derivatives, fdb_time_step), prop_mass(rocket_prop_mass),
        engines(std::move(rocket_engines)) {
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

void rocket::set_prop_mass(double new_prop_mass) {
    prop_mass = new_prop_mass;
}

std::vector<engine> &rocket::get_engines() {
    return engines;
}
