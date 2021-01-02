/**
 * @file
 */

#ifndef LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H
#define LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H

#include <map>

/**
 * @brief Represents a flight profile consisting of the
 * time to vertical and horizontal components of velocity.
 */
class velocity_flight_profile {
    /**
     * The time step used to iterate the flight profile.
     */
    const double time_step;
    /**
     * The current time to obtain the telemetry values.
     */
    double current_time{0};

    /**
     * Mapping of time values to horizontal velocity
     * component values.
     */
    std::map<double, double> vx;
    /**
     * Mapping of time values to vertiacal velocity
     * component values.
     */
    std::map<double, double> vy;

    /**
     * Obtains the telemetry value from the given mapping
     * at the given time.
     *
     * @param time the time to obtain the telemetry value
     * @param map the map from which to obtain the value
     * @return the telemetry value, or NAN if no value is
     * present
     */
    double get_telemetry_value(double time, const std::map<double, double> &map) const;

public:
    /**
     * Creates a new empty velocity flight profile with the
     * given time step delta.
     *
     * @param vfp_time_step the time step used to iterate
     * the flight profile
     */
    explicit velocity_flight_profile(double vfp_time_step);

    /**
     * Obtains the time delta used to step through the
     * flight profile.
     *
     * @return the time delta
     */
    double get_time_step() const;

    /**
     * Obtains the current time offset stored in this
     * profile.
     *
     * @return the current time
     */
    double get_current_time() const;

    /**
     * Records the given horizontal velocity value at the
     * given time offset in this flight profile.
     *
     * @param time the time value
     * @param next_v the horizontal velocity component at
     * this time
     */
    void put_vx(double time, double next_v);

    /**
     * Records the given vertical velocity value at the
     * given time offset in this flight profile.
     *
     * @param time the time value
     * @param next_v the vertical velocity component at
     * this time
     */
    void put_vy(double time, double next_v);

    /**
     * Increments the current time value stored in this
     * flight profile by the time delta to obtain the next
     * telemetry values.
     */
    void step();

    /**
     * Obtains the current horizontal velocity component at
     * the time offset given by get_current_time().
     *
     * @return the current horizontal velocity
     */
    double get_vx() const;

    /**
     * Obtains the current vertical velocity component at
     * the time offset given by get_current_time().
     *
     * @return the current vertical velocity
     */
    double get_vy() const;

    /**
     * Obtains the horizontal velocity component at the
     * given time offset.
     *
     * @param time the time at which to obtain the velocity
     * @return the horizontal velocity at that time
     */
    double get_vx(double time) const;

    /**
     * Obtains the vertical velocity component at the
     * given time offset.
     *
     * @param time the time at which to obtain the velocity
     * @return the vertical velocity at that time
     */
    double get_vy(double time) const;

    /**
     * Obtains the mapping of all time offsets to
     * horizontal velocities at those times.
     *
     * @return the map of times to horizontal velocity
     */
    std::map<double, double> &get_all_vx();

    /**
     * Obtains the mapping of all time offsets to
     * vertical velocities at those times.
     *
     * @return the map of times to vertical velocity
     */
    std::map<double, double> &get_all_vy();

    /**
     * Sets the current time stored in this profile back to
     * zero.
     */
    void reset();
};

#endif // LIFTOFF_CLI_VELOCITY_FLIGHT_PROFILE_H
