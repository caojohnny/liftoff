#ifndef LIFTOFF_PHYSICS_DRIVEN_BODY_H
#define LIFTOFF_PHYSICS_DRIVEN_BODY_H

#include <vector>
#include <unordered_set>

#include "vector.h"
#include "body.h"

namespace liftoff {
    class driven_body : public liftoff::body {
    protected:
        d_idx_t driver_idx;
        double time_step;
        std::unordered_set<d_idx_t> updated_derivatives;
        std::vector<liftoff::vector> prev_state;

        void set_component(d_idx_t derivative, const liftoff::vector &component) override;

        bool has_derivative_updated(d_idx_t derivative);

        virtual void drive_derivatives(d_idx_t cur_driver_idx);

        virtual void drive_integrals(d_idx_t cur_driver_idx);

    public:
        explicit driven_body(double body_mass, d_idx_t db_driver_idx, int body_derivatives = 4,
                             double db_time_step = 1);

        void pre_compute() override;

        void compute_motion() override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_DRIVEN_BODY_H
