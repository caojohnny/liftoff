/**
 * @file
 */

#ifndef LANDING_PIDF_CONTROLLER_H
#define LANDING_PIDF_CONTROLLER_H

/**
 * @brief Represents the state held by a PIDF controller.
 */
class pidf_controller {
private:
    /**
     * The difference in time between measurements.
     */
    double time_step;

    /**
     * Proportional gain.
     */
    double p_gain;
    /**
     * Integral gain.
     */
    double i_gain;
    /**
     * Derivative gain.
     */
    double d_gain;
    /**
     * Feed-forward gain.
     */
    double f_gain;

    /**
     * The goal setpoint.
     */
    double setpoint{0};
    /**
     * The last state sensed by the system.
     */
    double last_state{0};

    /**
     * Error accumulator integral.
     */
    double accum_error{0};
    /**
     * The last error computed to obtain the derivative of
     * the error function.
     */
    double last_error{0};

public:
    /**
     * Creates a new PIDF controller with the given time
     * step between sensor measurements and the gain values
     * corresponding to the PIDF functions.
     *
     * @param pc_time_step the time step.
     * @param pc_p_gain the initial proportional gain.
     * @param pc_i_gain the initial integral gain.
     * @param pc_d_gain the initial derivative gain.
     * @param pc_f_gain the initial feed-forward gain.
     */
    pidf_controller(double pc_time_step, double pc_p_gain, double pc_i_gain, double pc_d_gain, double pc_f_gain);

    /**
     * The time step between measurements.
     *
     * @return the time step.
     */
    double get_time_step() const;

    /**
     * Changes the proportional gain for the next
     * computation step.
     *
     * @param new_p_gain the new proportional gain.
     */
    void set_p_gain(double new_p_gain);

    /**
     * Obtains the current proportional gain.
     *
     * @return the proportional gain.
     */
    double get_p_gain() const;

    /**
     * Changes the integral gain for the next
     * computation step.
     *
     * @param new_i_gain the new integral gain.
     */
    void set_i_gain(double new_i_gain);

    /**
     * Obtains the current integral gain.
     *
     * @return the integral gain.
     */
    double get_i_gain() const;

    /**
     * Changes the derivative gain for the next
     * computation step.
     *
     * @param new_d_gain the new derivative gain.
     */
    void set_d_gain(double new_d_gain);

    /**
     * Obtains the current derivative gain.
     *
     * @return the derivative gain.
     */
    double get_d_gain() const;

    /**
     * Changes the feed-forward gain for the next
     * computation step.
     *
     * @param new_f_gain the new feed-forward gain.
     */
    void set_f_gain(double new_f_gain);

    /**
     * Obtains the current feed-forward gain.
     *
     * @return the feed-forward gain.
     */
    double get_f_gain() const;

    /**
     * Updates the setpoint value for the next computation
     * step.
     *
     * @param new_setpoint the new setpoint value.
     */
    void set_setpoint(double new_setpoint);

    /**
     * Obtains the current setpoint target, or returns 0 if
     * no setpoint has yet been manually provided.
     *
     * @return the current setpoint.
     */
    double get_setpoint() const;

    /**
     * Updates the sensed state of the system.
     *
     * @param new_last_state the new state value.
     */
    void set_last_state(double new_last_state);

    /**
     * Obtains the last state that was provided to the
     * controller, or returns 0 if no state value has yet
     * been manually provided.
     *
     * @return the last state value.
     */
    double get_last_state() const;

    /**
     * Determines the current error between the setpoint
     * and the last state value.
     *
     * @return the current error value
     */
    double compute_error() const;

    /**
     * Computes the PIDF function output based upon the
     * current error and the provided gain values.
     *
     * @return the PIDF function output
     */
    double compute_pidf();
};

#endif // LIFTOFF_CLI_PIDF_CONTROLLER_H
