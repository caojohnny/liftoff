#ifndef LIFTOFF_PHYSICS_DRIVEN_BODY_H
#define LIFTOFF_PHYSICS_DRIVEN_BODY_H

#include <vector>
#include <unordered_set>

#include "vector.h"
#include "body.h"

namespace liftoff {
    class driven_body : public liftoff::body {
    protected:
        std::unordered_set<int> updated_derivatives;
        std::vector<liftoff::vector> prev_state;

        void set_component(int derivative, const liftoff::vector &component) override;

        bool has_derivative_updated(int derivative);

        virtual void drive_derivatives(int driver_idx, double time_step);

        virtual void drive_integrals(int driver_idx, double time_step);

    public:
        explicit driven_body(double mass, int derivatives = 4);

        void pre_compute() override;

        void compute_motion(double time_step) override = 0;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_DRIVEN_BODY_H
