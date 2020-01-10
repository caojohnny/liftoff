#ifndef LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
#define LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H

#include <map>

class telemetry_flight_profile {
private:
    const double time_step;
    double current_time{0};
    std::map<double, double> velocity;
    std::map<double, double> altitude;

    double get_telemetry_value(std::map<double, double> map) const;

public:
    explicit telemetry_flight_profile(double time_step);

    void put_velocity(double time, double velocity);

    void put_altitude(double time, double altitude);

    void step();

    double get_velocity() const;

    double get_altitude() const;

    void reset();
};

#endif // LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
