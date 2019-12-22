#ifndef LIFTOFF_CLI_THRUST_H
#define LIFTOFF_CLI_THRUST_H

class engine {
private:
    double max_thrust;
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
