#ifndef LIFTOFF_PHYSICS_BODY_H
#define LIFTOFF_PHYSICS_BODY_H

#include <vector>

#include "vector.h"

namespace liftoff {
    typedef std::vector<int>::size_type d_idx_t;

    class body {
    protected:
        bool initial{true};
        double mass;
        std::vector<liftoff::vector> d_mot;

        virtual void set_component(d_idx_t derivative, const liftoff::vector &component);

    public:
        body(double body_mass, int derivatives);

        virtual double get_mass() const;

        void set_mass(double mass);

        const std::vector<liftoff::vector> &get_d_mot() const;

        virtual void pre_compute();

        virtual void compute_motion() = 0;

        virtual void post_compute() = 0;
    };
}

#endif // LIFTOFF_PHYSICS_BODY_H
