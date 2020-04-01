#include "pidf_controller.h"

// Implementation based on:
// https://agenttroll.github.io/blog/2019/01/16/intro-to-control-theory.html

pidf_controller::pidf_controller(double pc_time_step, double pc_p_gain, double pc_i_gain, double pc_d_gain,
                                 double pc_f_gain) :
        time_step(pc_time_step), p_gain(pc_p_gain), i_gain(pc_i_gain), d_gain(pc_d_gain), f_gain(pc_f_gain) {
}

double pidf_controller::get_time_step() {
    return time_step;
}

void pidf_controller::set_p_gain(double new_p_gain) {
    p_gain = new_p_gain;
}

double pidf_controller::get_p_gain() {
    return p_gain;
}

void pidf_controller::set_i_gain(double new_i_gain) {
    i_gain = new_i_gain;
}

double pidf_controller::get_i_gain() {
    return i_gain;
}

void pidf_controller::set_d_gain(double new_d_gain) {
    d_gain = new_d_gain;
}

double pidf_controller::get_d_gain() {
    return d_gain;
}

void pidf_controller::set_f_gain(double new_f_gain) {
    f_gain = new_f_gain;
}

double pidf_controller::get_f_gain() {
    return f_gain;
}

void pidf_controller::set_setpoint(double new_setpoint) {
    setpoint = new_setpoint;
}

double pidf_controller::get_setpoint() {
    return setpoint;
}

void pidf_controller::set_last_state(double new_last_state) {
    last_state = new_last_state;
}

double pidf_controller::get_last_state() {
    return last_state;
}

double pidf_controller::compute_error() {
    return setpoint - last_state;
}

double pidf_controller::compute_pidf() {
    double err = compute_error();

    accum_error += err * time_step;
    double der = (err - last_error) / time_step;
    last_error = err;

    double p = p_gain * err;
    double i = i_gain * accum_error;
    double d = d_gain * der;
    double f = f_gain * setpoint;

    return p + i + d + f;
}
