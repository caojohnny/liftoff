#ifndef LIFTOFF_VELOCITY_DRIVEN_BODY_H
#define LIFTOFF_VELOCITY_DRIVEN_BODY_H

#include "driven_body.h"

namespace liftoff {
    class velocity_driven_body : public liftoff::driven_body {
    public:
        explicit velocity_driven_body(double mass, int derivatives = 4, double time_step = 1);

        void set_position(const liftoff::vector &position);

        void set_velocity(const liftoff::vector &velocity);
    };
}

#endif // LIFTOFF_VELOCITY_DRIVEN_BODY_H
