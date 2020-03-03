#ifndef LIFTOFF_CLI_THRUST_H
#define LIFTOFF_CLI_THRUST_H

class engine {
private:
    double max_thrust;

    // I_sp = thrust / flow rate
    double i_sp;

    double throttle_pct;
public:
    engine(double max_thrust, double i_sp);

    void throttle(double pct);

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
