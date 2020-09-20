#ifndef LIFTOFF_PHYSICS_DRIVEN_BODY_H
#define LIFTOFF_PHYSICS_DRIVEN_BODY_H

#include <vector>
#include <unordered_set>

#include "vector.h"
#include "body.h"

namespace liftoff {
    /**
     * Represents a body whose movement is specified by one
     * of its derivatives.
     */
    class driven_body : public liftoff::body {
    protected:
        /**
         * The index of the driving derivative.
         */
        d_idx_t driver_idx;
        /**
         * The time step between update calls to the body's
         * motion.
         */
        double time_step;
        /**
         * A snapshot collection of the previous motion
         * vectors describing the movement of this body.
         */
        std::vector<liftoff::vector> prev_state;

        void set_component(d_idx_t derivative, const liftoff::vector &component) override;

        /**
         * Updates the derivatives based upon the index of
         * the given driver derivative.
         *
         * @param root_driver the driver derivative index
         */
        virtual void drive_derivatives(d_idx_t root_driver);

        /**
         * Updates the integrals based upon the index of
         * the given driver derivative.
         *
         * @param root_driver the driver derivative index
         */
        virtual void drive_integrals(d_idx_t root_driver);

    public:
        /**
         * Creates a new motion-drive body with the given
         * mass and driver index.
         *
         * @param body_mass the mass of the body
         * @param db_driver_idx the index of the derivative
         * that drives the motion of this body
         * @param body_derivatives the number of
         * derivatives to compute in total
         * @param db_time_step the default time step
         * between calls to the computation methods
         */
        explicit driven_body(double body_mass, d_idx_t db_driver_idx, int body_derivatives = 4,
                             double db_time_step = 1);

        void compute_motion() override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_DRIVEN_BODY_H
