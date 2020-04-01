#ifndef LIFTOFF_CLI_PIDF_CONTROLLER_H
#define LIFTOFF_CLI_PIDF_CONTROLLER_H

class pidf_controller {
private:
    const double time_step;

    double p_gain;
    double i_gain;
    double d_gain;
    double f_gain;

    double setpoint;
    double last_state;

    double accum_error;
    double last_error{0};
public:
    pidf_controller(double pc_time_step, double pc_p_gain, double pc_i_gain, double pc_d_gain, double pc_f_gain);

    double get_time_step();

    void set_p_gain(double new_p_gain);

    double get_p_gain();

    void set_i_gain(double new_i_gain);

    double get_i_gain();

    void set_d_gain(double new_d_gain);

    double get_d_gain();

    void set_f_gain(double new_f_gain);

    double get_f_gain();

    void set_setpoint(double new_setpoint);

    double get_setpoint();

    void set_last_state(double new_last_state);

    double get_last_state();

    double compute_error();

    double compute_pidf();
};

#endif // LIFTOFF_CLI_PIDF_CONTROLLER_H
