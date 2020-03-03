#include "recording_vdb.h"

recording_vdb::recording_vdb(double mass, int derivatives, double time_step) :
        liftoff::velocity_driven_body(mass, derivatives, time_step) {
    for (int i = 0; i < derivatives; ++i) {
        data.emplace_back();
    }
}

const std::vector<double> &recording_vdb::get_elapsed_times() const {
    return elapsed_times;
}

const vector_record &recording_vdb::get_data(int derivative) const {
    return data[derivative];
}

void recording_vdb::pre_compute() {
    bool initial{body::initial};
    velocity_driven_body::pre_compute();

    // Record the initial state vectors
    if (initial) {
        elapsed_times.push_back(cur_time);
        for (int i = 0; i < d_mot.size(); ++i) {
            data[i].record(d_mot[i]);
        }

        cur_time += time_step;
    }
}

void recording_vdb::post_compute() {
    velocity_driven_body::post_compute();

    elapsed_times.push_back(cur_time);
    for (int i = 0; i < d_mot.size(); ++i) {
        data[i].record(d_mot[i]);
    }

    cur_time += time_step;
}

