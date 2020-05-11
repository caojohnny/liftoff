#include <cmath>
#include "telemetry_flight_profile.h"

telemetry_flight_profile::telemetry_flight_profile(double tfp_time_step) : time_step(tfp_time_step) {
}

double telemetry_flight_profile::get_time_step() const {
    return time_step;
}

double telemetry_flight_profile::get_current_time() const {
    return current_time;
}

void telemetry_flight_profile::set_ballistic_range(double range) {
    ballistic_range = range;
}

double telemetry_flight_profile::get_downrange_distance() const {
    return ballistic_range * time_step;
}

void telemetry_flight_profile::put_velocity(double time, double next_velocity) {
    velocity[time] = next_velocity;
}

void telemetry_flight_profile::put_altitude(double time, double next_altitude) {
    altitude[time] = next_altitude;
}

void telemetry_flight_profile::step() {
    current_time += time_step;
}

double telemetry_flight_profile::get_telemetry_value(double time, const std::map<double, double> &map) const {
    auto it = map.lower_bound(time);
    if (it == map.end()) {
        return NAN;
    }

    return it->second;
}

double telemetry_flight_profile::get_velocity() const {
    return get_telemetry_value(current_time, velocity);
}

double telemetry_flight_profile::get_altitude() const {
    return get_telemetry_value(current_time, altitude);
}

double telemetry_flight_profile::get_velocity(double time) const {
    return get_telemetry_value(time, velocity);
}

double telemetry_flight_profile::get_altitude(double time) const {
    return get_telemetry_value(time, altitude);
}

std::map<double, double> &telemetry_flight_profile::get_velocities() {
    return velocity;
}

std::map<double, double> &telemetry_flight_profile::get_altitudes() {
    return altitude;
}

void telemetry_flight_profile::reset() {
    current_time = 0;
}
