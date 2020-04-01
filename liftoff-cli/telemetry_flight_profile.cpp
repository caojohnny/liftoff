#include <cmath>
#include "telemetry_flight_profile.h"

telemetry_flight_profile::telemetry_flight_profile(double tfp_time_step) : time_step(tfp_time_step) {
}

void telemetry_flight_profile::set_ballistic_range(double range) {
    ballistic_range = range;
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

double telemetry_flight_profile::get_telemetry_value(std::map<double, double> map, bool prev) const {
    if (prev) {
        auto it = map.lower_bound(current_time - time_step);
        if (it == map.end()) {
            return NAN;
        }

        return (*it).second;
    } else {
        auto it = map.upper_bound(current_time);
        if (it == map.end()) {
            return NAN;
        }

        return (*it).second;
    }
}

double telemetry_flight_profile::get_velocity() const {
    return get_telemetry_value(velocity, true);
}

double telemetry_flight_profile::get_altitude() const {
    return get_telemetry_value(altitude, false);
}

void telemetry_flight_profile::reset() {
    current_time = 0;
}

double telemetry_flight_profile::get_downrange_distance() const {
    return ballistic_range * time_step;
}
