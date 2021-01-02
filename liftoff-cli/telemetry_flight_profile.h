/**
 * @file
 */

#ifndef LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
#define LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H

#include <map>

/**
 * @brief A flight profile consisting of the mapping of
 * velocity magnitude and altitude.
 */
class telemetry_flight_profile {
private:
    /**
     * The time step used to iterate the flight profile.
     */
    const double time_step;
    /**
     * The current time to obtain the telemetry values.
     */
    double current_time{0};

    /**
     * The downrange distance for this profile.
     */
    double range{0};

    /**
     * The mapping of time to velocity magnitude.
     */
    std::map<double, double> velocity;
    /**
     * The mapping of time to altitude.
     */
    std::map<double, double> altitude;

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
     * Constructs a new, empty flight profile with the
     * given time delta.
     *
     * @param tfp_time_step the time delta for iterating
     * the profile
     */
    explicit telemetry_flight_profile(double tfp_time_step);

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
     * Sets the downrange distance stored in this profile.
     *
     * @param range the downrange distance
     */
    void set_range(double range);

    /**
     * Obtains the range or the downrange distance stored
     * in this flight profile.
     *
     * @return the downrange distance
     */
    double get_downrange_distance() const;

    /**
     * Sets the telemetry value for velocity at the given
     * time to the given value.
     *
     * @param time the time at which the velocity value was
     * recorded
     * @param next_velocity the velocity value to record
     */
    void put_velocity(double time, double next_velocity);

    /**
     * Sets the telemetry value for the altitude at the
     * given time to the given value.
     *
     * @param time the time at which the altitude value was
     * recorded
     * @param next_altitude the altitude value to record
     */
    void put_altitude(double time, double next_altitude);

    /**
     * Increments the current time value stored in this
     * flight profile by the time delta to obtain the next
     * telemetry values.
     */
    void step();

    /**
     * Obtains the velocity magnitude telemetry value at
     * the current time value given by get_current_time().
     *
     * @return the current velocity magnitude
     */
    double get_velocity() const;

    /**
     * Obtains the altitude telemetry value at the current
     * time value given by get_current_time().
     *
     * @return the current altitude
     */
    double get_altitude() const;

    /**
     * Obtains the velocity magnitude at the given time
     * offset.
     *
     * @param time the time at which to retrieve the
     * velocity magnitude
     * @return the magnitude of velocity recorded at the
     * given time
     */
    double get_velocity(double time) const;

    /**
     * Obtains the altitude at the given time offset.
     *
     * @param time the time at which to retrieve the
     * altitude
     * @return the altitude recorded at the given time
     */
    double get_altitude(double time) const;

    /**
     * Obtains the mapping of time values to velocity
     * magnitudes.
     *
     * @return the time to velocity magnitude map
     */
    std::map<double, double> &get_velocities();

    /**
     * Obtains the mapping of time values to altitudes.
     *
     * @return the time to altitude map
     */
    std::map<double, double> &get_altitudes();

    /**
     * Sets the current time stored in this profile back to
     * zero.
     */
    void reset();
};

#endif // LIFTOFF_CLI_TELEMETRY_FLIGHT_PROFILE_H
