/**
 * @file
 * @brief Helper functions for copying processing telemetry
 * data
 */

#ifndef LIFTOFF_PHYSICS_TELEM_PROC_H
#define LIFTOFF_PHYSICS_TELEM_PROC_H

#include <map>
#include <vector>

namespace liftoff {
    /**
     * Obtains the iterator pointing to the position of the
     * event at which the velocity experiences a local min
     * or max.
     *
     * @param begin the beginning iterator, inclusive
     * @param velocities the velocity values
     * @param negative_dv whether the delta should be
     * negative
     * @return the pointer to the event, or to the end if
     * no event was found
     */
    std::map<double, double>::const_iterator find_event_time(std::map<double, double>::const_iterator begin,
                                                             const std::map<double, double> &velocities,
                                                             bool negative_dv);

    /**
     * Performs linear interpolation between the values
     * which are the same.
     *
     * @param out the output map
     * @param in the input map of data to interpolate
     */
    void interp_lin(std::map<double, double> &out, const std::map<double, double> &in);

    /**
     * Collects points into an indexed collection of event
     * legs delineated by the given collection of event
     * times.
     *
     * @param times the output of times
     * @param legs the output of values
     * @param in the input of all values to collect
     * @param event_times the collection of times to
     * delineate the collected values
     */
    void collect(std::vector<std::vector<double>> &times,
                 std::vector<std::vector<double>> &legs,
                 const std::map<double, double> &in,
                 const std::vector<double> &event_times);

    /**
     * Collects points into the output vector of points
     * to force fit a polynomial regression based on the
     * input mapping of values, the time subset and the
     * number points ot select.
     *
     * @param out the output collection
     * @param in the data which to pair
     * @param times the time subset
     * @param count the number of points from the
     * beginning to select, or from the end if negative
     */
    void force(std::vector<std::pair<double, double>> &out,
               const std::map<double, double> &in,
               const std::vector<double> &times,
               int count);
}

#endif // LIFTOFF_PHYSICS_TELEM_PROC_H
