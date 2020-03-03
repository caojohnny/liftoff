#include "driven_body.h"

liftoff::driven_body::driven_body(double mass, int driver_idx, int derivatives, double time_step) :
        liftoff::body(mass, derivatives), driver_idx(driver_idx), time_step(time_step) {
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

    if (updated_derivatives.insert(derivative).second) {
        prev.set(cur);
    }
    body::set_component(derivative, component);

    if (!initial) {
        drive_derivatives(derivative);
    }
}

bool liftoff::driven_body::has_derivative_updated(int derivative) {
    return updated_derivatives.find(derivative) != updated_derivatives.end();
}

void liftoff::driven_body::drive_derivatives(int driver_idx) {
    liftoff::vector time_v{time_step};

    for (int i = driver_idx + 1; i < d_mot.size(); ++i) {
        const liftoff::vector &prev_driving_vec{prev_state[i - 1]};
        const liftoff::vector &cur_driving_vec{d_mot[i - 1]};
        d_mot[i].set(cur_driving_vec).sub(prev_driving_vec).div(time_v);
    }
}

void liftoff::driven_body::drive_integrals(int driver_idx) {
    liftoff::vector time_v{time_step};

    std::vector<liftoff::vector> adjusted_mot;
    for (const auto &i : d_mot) {
        adjusted_mot.push_back(liftoff::vector{i}.mul(time_v));
    }

    for (int i = driver_idx - 1; i >= 0; --i) {
        const liftoff::vector cur_driving_vec{adjusted_mot[i + 1]};
        d_mot[i].add(cur_driving_vec);
    }
}

void liftoff::driven_body::pre_compute() {
    body::pre_compute();

    for (int i = 0; i < d_mot.size(); ++i) {
        if (!has_derivative_updated(i)) {
            prev_state[i].set(d_mot[i]);
        }
    }
}

void liftoff::driven_body::compute_motion() {
    drive_integrals(driver_idx);
}

void liftoff::driven_body::post_compute() {
    updated_derivatives.clear();
}
