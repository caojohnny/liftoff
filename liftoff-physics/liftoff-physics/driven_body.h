#ifndef LIFTOFF_PHYSICS_DRIVEN_BODY_H
#define LIFTOFF_PHYSICS_DRIVEN_BODY_H

#include <vector>
#include <unordered_set>

#include "vector.h"
#include "body.h"

namespace liftoff {
    class driven_body : public liftoff::body {
    protected:
        int driver_idx;
        double time_step;
        std::unordered_set<int> updated_derivatives;
        std::vector<liftoff::vector> prev_state;

        void set_component(int derivative, const liftoff::vector &component) override;

        bool has_derivative_updated(int derivative);

        virtual void drive_derivatives(int driver_idx);

        virtual void drive_integrals(int driver_idx);

    public:
        explicit driven_body(double mass, int driver_idx, int derivatives = 4, double time_step = 1);

        void clear_state_changes();

        void pre_compute() override;

        void compute_motion() override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_DRIVEN_BODY_H
