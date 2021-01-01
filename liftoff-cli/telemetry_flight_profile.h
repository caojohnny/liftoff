#ifndef LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
#define LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H

#include <map>

class telemetry_flight_profile {
private:
    const double time_step;
    double current_time{0};
    double ballistic_range{0};
    std::map<double, double> velocity;
    std::map<double, double> altitude;

    double get_telemetry_value(double time, const std::map<double, double> &map) const;

public:
    explicit telemetry_flight_profile(double tfp_time_step);

    double get_time_step() const;

    double get_current_time() const;

    void set_range(double range);

    double get_downrange_distance() const;

    void put_velocity(double time, double next_velocity);

    void put_altitude(double time, double next_altitude);

    void step();

    double get_velocity() const;

    double get_altitude() const;

    double get_velocity(double time) const;

    double get_altitude(double time) const;

    std::map<double, double> &get_velocities();

    std::map<double, double> &get_altitudes();

    void reset();
};

#endif // LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
