#include "driven_body.h"

namespace liftoff {
    driven_body::driven_body(double body_mass, d_idx_t db_driver_idx, int body_derivatives, double db_time_step) :
            body(body_mass, body_derivatives), driver_idx(db_driver_idx), time_step(db_time_step) {
        for (int i = 0; i < body_derivatives; ++i) {
            prev_state.emplace_back();
        }
    }

    void driven_body::set_component(d_idx_t derivative, const vector &component) {
        if (d_mot.size() <= derivative) {
            return;
        }

        body::set_component(derivative, component);

        if (!initial) {
            drive_derivatives(derivative);
        }
    }

    void driven_body::drive_derivatives(d_idx_t root_driver) {
        vector time_v{time_step};

        for (d_idx_t i = root_driver + 1; i < d_mot.size(); ++i) {
            const vector &prev_driving_vec{prev_state[i - 1]};
            const vector &cur_driving_vec{d_mot[i - 1]};
            d_mot[i].set(cur_driving_vec).sub(prev_driving_vec).div(time_v);
        }
    }

    void driven_body::drive_integrals(d_idx_t root_driver) {
        if (root_driver == 0) {
            return;
        }

        vector time_v{time_step};

        std::vector<vector> adjusted_mot;
        for (const auto &vec : d_mot) {
            adjusted_mot.push_back(vector{vec}.mul(time_v));
        }

        d_idx_t i = root_driver;
        do {
            i--;

            const vector cur_driving_vec{adjusted_mot[i + 1]};
            d_mot[i].add(cur_driving_vec);
        } while (i != 0);
    }

    void driven_body::compute_motion() {
        drive_integrals(driver_idx);
    }

    void driven_body::post_compute() {
        for (d_idx_t i = 0; i < d_mot.size(); ++i) {
            prev_state[i].set(d_mot[i]);
        }
    }
}
