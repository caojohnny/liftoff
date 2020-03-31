#ifndef LIFTOFF_CLI_THRUST_H
#define LIFTOFF_CLI_THRUST_H

class engine {
private:
    const double max_thrust;

    // I_sp = thrust / flow rate
    const double i_sp;

    double throttle_pct;
public:
    engine(double engine_max_thrust, double engine_i_sp);

    double get_max_thrust() const;

    double get_i_sp() const;

    void set_throttle(double pct);

    double get_throttle() const;

    double get_thrust() const;

    double get_prop_flow_rate() const;
};

class thrust_interface {
};

class thrust_controller {
private:
    double time_step;
public:
    thrust_controller(thrust_interface ti, double time_step);
};

#endif // LIFTOFF_CLI_THRUST_H
