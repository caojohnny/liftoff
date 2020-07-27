#include "telem_proc.h"

#include <vector>

std::map<double, double>::const_iterator liftoff::find_event_time(std::map<double, double>::const_iterator begin,
                                                                  const std::map<double, double> &velocities,
                                                                  bool negative_dv) {
    double prev_v = begin->second;
    for (auto it = begin; it != velocities.end(); ++it) {
        double v = it->second;
        if (negative_dv && v < prev_v || !negative_dv && v > prev_v) {
            return it;
        }

        prev_v = v;
    }

    return velocities.end();
}

void liftoff::interp_lin(std::map<double, double> &out, const std::map<double, double> &in) {
    std::vector<double> st;
    double last_unique_time = -1;
    double last_unique_value = -1;

    for (auto it = in.begin(); it != in.end(); it++) {
        double t = it->first;
        double v = it->second;
        if (v != last_unique_value || it == --in.end()) {
            out[t] = v;

            if (!st.empty()) {
                double slope = (v - last_unique_value) / (t - last_unique_time);
                for (const auto &interp_t : st) {
                    double dt = interp_t - last_unique_time;
                    out[interp_t] = last_unique_value + slope * dt;
                }
                st.clear();
            }

            last_unique_time = t;
            last_unique_value = v;
        } else {
            st.push_back(t);
        }
    }
}

void liftoff::force(std::vector<std::pair<double, double>> &out,
                    const std::map<double, double> &in,
                    const std::vector<double> &times,
                    int count) {
    int added = 0;
    if (count >= 0) {
        for (auto it = times.begin(); it != times.end(); ++it) {
            double t = *it.base();
            double value = in.find(t)->second;
            out.emplace_back(t, value);

            if (++added == count) {
                break;
            }
        }
    } else {
        for (auto it = times.rbegin(); it != times.rend(); ++it) {
            double t = *it.base();
            double value = in.find(t)->second;
            out.emplace_back(t, value);

            if (++added == -count) {
                break;
            }
        }
    }
}

void liftoff::collect(std::vector<std::vector<double>> &times,
                      std::vector<std::vector<double>> &legs,
                      const std::map<double, double> &in,
                      const std::vector<double> &event_times) {
    int n_events = event_times.size();

    times.reserve(n_events);
    legs.reserve(n_events);
    for (int k = 0; k < n_events; ++k) {
        times.emplace_back();
        legs.emplace_back();
    }

    for (const auto &it : in) {
        double t = it.first;
        double alt = it.second;
        for (int i = 0; i < n_events; ++i) {
            if (t < event_times[i]) {
                times[i].push_back(t);
                legs[i].push_back(alt);
                break;
            }
        }
    }
}
