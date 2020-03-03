#ifndef LIFTOFF_CLI_RECORDING_VDB_H
#define LIFTOFF_CLI_RECORDING_VDB_H

#include <vector>
#include "recording_fdb.h"

class recording_vdb : public liftoff::velocity_driven_body {
private:
    double cur_time{0};
    std::vector<double> elapsed_times;
    std::vector<vector_record> data;
public:
    explicit recording_vdb(double mass, int derivatives = 4, double time_step = 1);

    const std::vector<double> &get_elapsed_times() const;

    const vector_record &get_data(int derivative) const;

    void pre_compute() override;

    void post_compute() override;
};

#endif // LIFTOFF_CLI_RECORDING_VDB_H