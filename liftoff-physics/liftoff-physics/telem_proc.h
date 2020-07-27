#ifndef LIFTOFF_PHYSICS_TELEM_PROC_H
#define LIFTOFF_PHYSICS_TELEM_PROC_H

#include <map>
#include <vector>

namespace liftoff {
    std::map<double, double>::const_iterator find_event_time(std::map<double, double>::const_iterator begin,
                                                             const std::map<double, double> &velocities,
                                                             bool negative_dv);

    void interp_lin(std::map<double, double> &out, const std::map<double, double> &in);

    void collect(std::vector<std::vector<double>> &times,
                 std::vector<std::vector<double>> &legs,
                 const std::map<double, double> &in,
                 const std::vector<double> &event_times);

    void force(std::vector<std::pair<double, double>> &out,
               const std::map<double, double> &in,
               const std::vector<double> &times,
               int count);
}

#endif // LIFTOFF_PHYSICS_TELEM_PROC_H
