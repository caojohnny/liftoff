#ifndef LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H
#define LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H

#include <map>

class velocity_flight_profile {
    const double time_step;
    double current_time{0};
    std::map<double, double> vx;
    std::map<double, double> vy;

    double get_telemetry_value(double time, const std::map<double, double> &map) const;

public:
    explicit velocity_flight_profile(double vfp_time_step);

    double get_time_step() const;

    double get_current_time() const;

    void put_vx(double time, double next_v);

    void put_vy(double time, double next_v);

    void step();

    double get_vx() const;

    double get_vy() const;

    double get_vx(double time) const;

    double get_vy(double time) const;

    std::map<double, double> &get_all_vx();

    std::map<double, double> &get_all_vy();

    void reset();
};

#endif // LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H
