#ifndef LIFTOFF_FORCE_DRIVEN_BODY_H
#define LIFTOFF_FORCE_DRIVEN_BODY_H

#include <vector>

#include "vector.h"
#include "driven_body.h"

namespace liftoff {
    class force_driven_body : public liftoff::driven_body {
    protected:
        std::vector<liftoff::vector> forces;

    public:
        explicit force_driven_body(double db_mass, int db_derivatives = 4, double db_time_step = 1);

        void set_position(const liftoff::vector &position);

        void set_velocity(const liftoff::vector &velocity);

        void set_acceleration(const liftoff::vector &acceleration);

        std::vector<liftoff::vector> &get_forces();

        void pre_compute() override;

        virtual void compute_forces();
    };
}

#endif // LIFTOFF_PHYSICS_FORCE_DRIVEN_BODY_H
