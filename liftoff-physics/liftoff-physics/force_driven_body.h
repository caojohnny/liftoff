#ifndef LIFTOFF_FORCE_DRIVEN_BODY_H
#define LIFTOFF_FORCE_DRIVEN_BODY_H

#include <vector>

#include "vector.h"
#include "driven_body.h"

namespace liftoff {
    /**
     * Represents a body whose motion is determined by the
     * forces acting upon it.
     */
    class force_driven_body : public liftoff::driven_body {
    protected:
        /**
         * The forces acting upon this body.
         */
        std::vector<liftoff::vector> forces;

    public:
        /**
         * Creates a new force drive body with the given
         * mass. The derivatives can be given in addition
         * to the time step between computations.
         *
         * @param db_mass the mass of the body
         * @param db_derivatives the number of derivatives
         * to store
         * @param db_time_step the time step between
         * computations
         */
        explicit force_driven_body(double db_mass, int db_derivatives = 4, double db_time_step = 1);

        /**
         * Sets the position of this body.
         *
         * @param position the position vector
         */
        void set_position(const liftoff::vector &position);

        /**
         * Sets the velocity of this body.
         *
         * @param velocity the position vector
         */
        void set_velocity(const liftoff::vector &velocity);

        /**
         * Sets the acceleration of this body.
         *
         * @param acceleration the position vector
         */
        void set_acceleration(const liftoff::vector &acceleration);

        /**
         * Obtains the reference to the collection of
         * forces currently acting on this body.
         *
         * @return the collection of forces
         */
        std::vector<liftoff::vector> &get_forces();

        void pre_compute() override;

        /**
         * Performs a computation of the body's motion
         * based upon the forces.
         */
        virtual void compute_forces();
    };
}

#endif // LIFTOFF_PHYSICS_FORCE_DRIVEN_BODY_H
