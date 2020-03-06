#include "thrust.h"

engine::engine(double max_thrust, double i_sp) : max_thrust(max_thrust), i_sp(i_sp) {
}

double engine::get_max_thrust() const {
    return max_thrust;
}

double engine::get_i_sp() const {
    return i_sp;
}

void engine::set_throttle(double pct) {
    throttle_pct = pct;
}

double engine::get_throttle() const {
    return throttle_pct;
}

double engine::get_thrust() const {
    return throttle_pct * max_thrust;
}

double engine::get_prop_flow_rate() const {
    return get_thrust() / i_sp;
}
