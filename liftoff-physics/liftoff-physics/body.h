#ifndef LIFTOFF_PHYSICS_BODY_H
#define LIFTOFF_PHYSICS_BODY_H

#include <vector>

#include "vector.h"

namespace liftoff {
    /**
     * Abstraction of the derivative index for a motion
     * vector.
     */
    typedef std::vector<int>::size_type d_idx_t;

    /**
     * Represents a physical body possessing mass and motion.
     */
    class body {
    protected:
        /**
         * Whether or not the computation has started (i.e.
         * whether pre_compute() has been called).
         */
        bool initial{true};
        /**
         * The mass of this body.
         */
        double mass;
        /**
         * The collection of vectors that describe the 3D
         * motion of this body.
         */
        std::vector<liftoff::vector> d_mot;

        /**
         * Sets the motion vector for the given derivative
         * of motion.
         *
         * @param derivative the index of the derivative
         * @param component the new value of the motion
         * vector
         */
        virtual void set_component(d_idx_t derivative, const liftoff::vector &component);

    public:
        /**
         * Creates a new body with the given mass and the
         * give number of motion derivatives to store.
         *
         * @param body_mass the mass of the body
         * @param derivatives the number of derivatives of
         * motion to store
         */
        body(double body_mass, int derivatives);

        /**
         * Obtains the mass of this body.
         *
         * @return the body mass
         */
        virtual double get_mass() const;

        /**
         * Updates the mass for this body.
         *
         * @param mass the new body mass
         */
        void set_mass(double mass);

        /**
         * Obtains the collection of motion vectors for
         * this body.
         *
         * @return the constant view of motion vectors
         */
        const std::vector<liftoff::vector> &get_d_mot() const;

        /**
         * The pre-computation method for updating the
         * motion derivatives.
         */
        virtual void pre_compute();

        /**
         * The computation method for updating the motion
         * derivatives.
         */
        virtual void compute_motion() = 0;

        /**
         * The post-computation method for updating the
         * motion derivatives.
         */
        virtual void post_compute() = 0;
    };
}

#endif // LIFTOFF_PHYSICS_BODY_H
