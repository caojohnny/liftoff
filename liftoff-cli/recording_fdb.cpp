#include "recording_fdb.h"

recording_fdb::recording_fdb(double mass, int derivatives, double time_step) : force_driven_body(mass, derivatives,
                                                                                                 time_step) {
    for (int i = 0; i < derivatives; ++i) {
        data.emplace_back();
    }
}

const vector_record &recording_fdb::get_data(int derivative) const {
    return data[derivative];
}

void recording_fdb::post_compute() {
    force_driven_body::post_compute();

    for (int i = 0; i < d_mot.size(); ++i) {
        data[i].record(d_mot[i]);
    }
}

vector_record::vector_record() {
    for (int i = 0; i < 3; ++i) {
        dim.emplace_back();
    }
}

void vector_record::record(const liftoff::vector &vec) {
    dim[0].push_back(vec.get_x());
    dim[1].push_back(vec.get_y());
    dim[2].push_back(vec.get_z());
}

const std::vector<double> &vector_record::get_x() const {
    return dim[0];
}

const std::vector<double> &vector_record::get_y() const {
    return dim[1];
}

const std::vector<double> &vector_record::get_z() const {
    return dim[2];
}
