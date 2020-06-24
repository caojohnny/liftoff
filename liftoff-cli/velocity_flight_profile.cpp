#include "velocity_flight_profile.h"

#include <cmath>

velocity_flight_profile::velocity_flight_profile(double vfp_time_step) : time_step(vfp_time_step) {

}

double velocity_flight_profile::get_telemetry_value(double time, const std::map<double, double> &map) const {
    auto it = map.lower_bound(time);
    if (it == map.end()) {
        return NAN;
    }

    double dt = std::abs(it->first - time);
    double val = it->second;
    if (it != map.begin()) {
        auto check_it = --it;
        if (std::abs(check_it->first - time) < dt) {
            return check_it->second;
        }
    }

    return val;
}

double velocity_flight_profile::get_time_step() const {
    return time_step;
}

double velocity_flight_profile::get_current_time() const {
    return current_time;
}

void velocity_flight_profile::put_vx(double time, double next_v) {
    vx[time] = next_v;
}

void velocity_flight_profile::put_vy(double time, double next_v) {
    vy[time] = next_v;
}

void velocity_flight_profile::step() {
    current_time += time_step;
}

double velocity_flight_profile::get_vx() const {
    return get_telemetry_value(current_time, vx);
}

double velocity_flight_profile::get_vy() const {
    return get_telemetry_value(current_time, vy);
}

double velocity_flight_profile::get_vx(double time) const {
    return get_telemetry_value(time, vx);
}

double velocity_flight_profile::get_vy(double time) const {
    return get_telemetry_value(time, vy);
}

std::map<double, double> &velocity_flight_profile::get_all_vx() {
    return vx;
}

std::map<double, double> &velocity_flight_profile::get_all_vy() {
    return vy;
}

void velocity_flight_profile::reset() {
    current_time = 0;
}
