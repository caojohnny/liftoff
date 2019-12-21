#ifndef LIFTOFF_CLI_RECORDING_FDB_H
#define LIFTOFF_CLI_RECORDING_FDB_H

#include <liftoff-physics/force_driven_body.h>

class vector_record {
private:
    std::vector<std::vector<double>> dim;

public:
    vector_record();

    void record(const liftoff::vector &vec);

    const std::vector<double> &get_x() const;

    const std::vector<double> &get_y() const;

    const std::vector<double> &get_z() const;
};

class recording_fdb : public liftoff::force_driven_body {
private:
    std::vector<vector_record> data;
public:
    explicit recording_fdb(double mass, int derivatives = 4, double time_step = 1);

    const vector_record &get_data(int derivative) const;

    void post_compute() override;
};

#endif // LIFTOFF_CLI_RECORDING_FDB_H
