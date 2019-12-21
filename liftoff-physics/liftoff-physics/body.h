#ifndef LIFTOFF_PHYSICS_BODY_H
#define LIFTOFF_PHYSICS_BODY_H

#include <vector>

#include "vector.h"

namespace liftoff {
    class body {
    protected:
        bool initial{true};
        double mass;
        std::vector<liftoff::vector> d_mot;

        virtual void set_component(int derivative, const liftoff::vector &component);

    public:
        body(double mass, int derivatives);

        double get_mass() const;

        void set_mass(double mass);

        const std::vector<liftoff::vector> &get_d_mot() const;

        virtual void pre_compute();

        virtual void compute_motion() = 0;

        virtual void post_compute() = 0;
    };
}

#endif // LIFTOFF_PHYSICS_BODY_H
