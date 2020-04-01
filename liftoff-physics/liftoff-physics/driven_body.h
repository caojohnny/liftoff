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
        std::vector<liftoff::vector> prev_state;

        void set_component(d_idx_t derivative, const liftoff::vector &component) override;

        virtual void drive_derivatives(d_idx_t root_driver);

        virtual void drive_integrals(d_idx_t root_driver);

    public:
        explicit driven_body(double body_mass, d_idx_t db_driver_idx, int body_derivatives = 4,
                             double db_time_step = 1);

        void compute_motion() override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_DRIVEN_BODY_H
