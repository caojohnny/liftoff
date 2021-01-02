/**
 * @file
 */

#ifndef LIFTOFF_VELOCITY_DRIVEN_BODY_H
#define LIFTOFF_VELOCITY_DRIVEN_BODY_H

#include "driven_body.h"

namespace liftoff {
    /**
     * @brief Represents a body whose motion is controlled
     * by the velocity vector.
     */
    class velocity_driven_body : public liftoff::driven_body {
    public:
        /**
         * Creates a new velocity driven body with the
         * given mass. The number of motion derivatives and
         * the time step can also be provided.
         *
         * @param body_mass the body's mass
         * @param body_derivatives the number of
         * derivatives to store
         * @param body_time_step the time step between
         * computations of the new motion
         */
        explicit velocity_driven_body(double body_mass, int body_derivatives = 4, double body_time_step = 1);

        /**
         * Sets the position of the body.
         *
         * @param position the position vector
         */
        void set_position(const liftoff::vector &position);

        /**
         * Sets the velocity of the body.
         *
         * @param velocity the velocity vector
         */
        void set_velocity(const liftoff::vector &velocity);
    };
}

#endif // LIFTOFF_VELOCITY_DRIVEN_BODY_H
