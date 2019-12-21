#include <iostream>
#include "driven_body.h"

liftoff::driven_body::driven_body(double mass, int derivatives) : liftoff::body(mass, derivatives) {
    for (int i = 0; i < derivatives; ++i) {
        prev_state.emplace_back();
    }
}

void liftoff::driven_body::set_component(int derivative, const liftoff::vector &component) {
    if (d_mot.size() < derivative + 1) {
        return;
    }

    liftoff::vector &prev{prev_state[derivative]};
    liftoff::vector &cur{d_mot[derivative]};

    prev.set(cur);
    body::set_component(derivative, component);

    updated_derivatives.insert(derivative);
}

bool liftoff::driven_body::has_derivative_updated(int derivative) {
    return updated_derivatives.find(derivative) != updated_derivatives.end();
}

void liftoff::driven_body::drive_derivatives(int driver_idx, double time_step) {
    liftoff::vector time_v{time_step};

    for (int i = driver_idx + 1; i < d_mot.size(); ++i) {
        liftoff::vector &prev_driving_vec{prev_state[i - 1]};
        liftoff::vector &cur_driving_vec{d_mot[i - 1]};
        d_mot[i].set(cur_driving_vec).sub(prev_driving_vec).div(time_v);
    }
}

void liftoff::driven_body::drive_integrals(int driver_idx, double time_step) {
    liftoff::vector time_v{time_step};

    std::vector<liftoff::vector> adjusted_mot;
    for (const auto &i : d_mot) {
        adjusted_mot.push_back(liftoff::vector{i}.mul(time_v));
    }

    for (int i = driver_idx - 1; i >= 0; --i) {
        liftoff::vector cur_driving_vec{adjusted_mot[i + 1]};
        d_mot[i].add(cur_driving_vec);
    }
}

void liftoff::driven_body::pre_compute() {
    for (int i = 0; i < d_mot.size(); ++i) {
        if (!has_derivative_updated(i)) {
            prev_state[i].set(d_mot[i]);
        }
    }
}

void liftoff::driven_body::post_compute() {
    updated_derivatives.clear();
}
