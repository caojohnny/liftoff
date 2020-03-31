#include "recording_vdb.h"

recording_vdb::recording_vdb(double vdb_mass, int vdb_derivatives, double vdb_time_step) :
        liftoff::velocity_driven_body(vdb_mass, vdb_derivatives, vdb_time_step) {
    for (int i = 0; i < vdb_derivatives; ++i) {
        data.emplace_back();
    }
}

const std::vector<double> &recording_vdb::get_elapsed_times() const {
    return elapsed_times;
}

const vector_record &recording_vdb::get_data(liftoff::d_idx_t derivative) const {
    return data[derivative];
}

void recording_vdb::pre_compute() {
    bool is_initial{body::initial};
    velocity_driven_body::pre_compute();

    // Record the initial state vectors
    if (is_initial) {
        elapsed_times.push_back(cur_time);
        for (liftoff::d_idx_t i = 0; i < d_mot.size(); ++i) {
            data[i].record(d_mot[i]);
        }

        cur_time += time_step;
    }
}

void recording_vdb::post_compute() {
    velocity_driven_body::post_compute();

    elapsed_times.push_back(cur_time);
    for (liftoff::d_idx_t i = 0; i < d_mot.size(); ++i) {
        data[i].record(d_mot[i]);
    }

    cur_time += time_step;
}

