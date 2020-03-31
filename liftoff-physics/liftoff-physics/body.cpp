#include "body.h"

namespace liftoff {
    body::body(double body_mass, int derivatives) : mass(body_mass) {
        for (int i = 0; i < derivatives; ++i) {
            d_mot.emplace_back();
        }
    }

    double body::get_mass() const {
        return mass;
    }

    void body::set_mass(double new_mass) {
        mass = new_mass;
    }

    const std::vector<vector> &body::get_d_mot() const {
        return d_mot;
    }

    void body::set_component(d_idx_t derivative, const vector &component) {
        if (d_mot.size() <= derivative) {
            return;
        }

        vector &cur{d_mot[derivative]};
        cur.set(component);
    }

    void body::pre_compute() {
        initial = false;
    }
}
