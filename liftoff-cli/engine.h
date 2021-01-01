#ifndef LIFTOFF_CLI_THRUST_H
#define LIFTOFF_CLI_THRUST_H

/**
 * @brief Represents the relevant properties of a rocket
 * engine for the purposes of dynamics simulation.
 */
class engine {
private:
    // Maximum thrust force provided by this engine
    const double max_thrust;
    // I_sp = thrust / flow rate
    const double i_sp;

    // Current percentage of maximum thrust, decimal
    double throttle_pct;
public:
    /**
     * Creates a new engine with the given properties.
     *
     * @param engine_max_thrust the max thrust force
     * @param engine_i_sp the specific impulse of the
     * engine
     */
    engine(double engine_max_thrust, double engine_i_sp);

    /**
     * Determines the maximum thrust force provided by this
     * engine at 100% throttle.
     *
     * @return the maximum thrust force
     */
    double get_max_thrust() const;

    /**
     * Determines the specific impulse of this engine. No
     * distinction is made between sea-level of vacuum.
     *
     * @return the specific impulse of the engine
     */
    double get_i_sp() const;

    /**
     * Sets the engine throttle as a decimal percentage of
     * the engine's maximum thrust provided by
     * get_max_thrust().
     *
     * @param pct the thrust percentage
     */
    void set_throttle(double pct);

    /**
     * Determines the current throttle last set by
     * set_throttle().
     *
     * @return the current engine throttle
     */
    double get_throttle() const;

    /**
     * Determines the thrust force based upon the maximum
     * thrust of the engine and the throttle percentage.
     *
     * @return the thrust force
     */
    double get_thrust() const;

    /**
     * Obtains the propellant mass flow rate using the
     * thrust provided by the engine based on throttle and
     * the specific impulse of the engine.
     *
     * @return the propellant mass flow rate through the
     * engine
     */
    double get_prop_flow_rate() const;
};

#endif // LIFTOFF_CLI_THRUST_H
