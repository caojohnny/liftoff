#include <cmath>
#include "telemetry_flight_profile.h"

telemetry_flight_profile::telemetry_flight_profile(double time_step) : time_step(time_step) {
}

void telemetry_flight_profile::set_ballistic_range(double range) {
    telemetry_flight_profile::ballistic_range = range;
}

void telemetry_flight_profile::put_velocity(double time, double velocity) {
    telemetry_flight_profile::velocity[time] = velocity;
}

void telemetry_flight_profile::put_altitude(double time, double altitude) {
    telemetry_flight_profile::altitude[time] = altitude;
}

void telemetry_flight_profile::step() {
    current_time += time_step;
}

double telemetry_flight_profile::get_telemetry_value(std::map<double, double> map) const {
    auto it = map.lower_bound(current_time - time_step);
    if (it == map.end()) {
        return NAN;
    }

    auto entry = *it;
    if (entry.first <= current_time) {
        return entry.second;
    }

    return NAN;
}

double telemetry_flight_profile::get_velocity() const {
    return get_telemetry_value(velocity);
}

double telemetry_flight_profile::get_altitude() const {
    return get_telemetry_value(altitude);
}

void telemetry_flight_profile::reset() {
    current_time = 0;
}

double telemetry_flight_profile::get_downrange_distance() const {
    return ballistic_range * time_step;
}
