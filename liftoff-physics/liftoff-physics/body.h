#ifndef LIFTOFF_BODY_H
#define LIFTOFF_BODY_H

#include <vector>

#include "vector.h"

namespace liftoff {
    class body {
    protected:
        double mass;
        std::vector<liftoff::vector> forces;
        std::vector<liftoff::vector> d_mot;

    public:
        body(double mass, int derivatives);

        double get_mass() const;

        void set_mass(double mass);

        std::vector<liftoff::vector> &get_forces();

        std::vector<liftoff::vector> &get_d_mot();

        virtual void pre_compute();

        virtual void compute_forces(double elapsed_time);

        virtual void compute_motion(double elapsed_time);

        virtual void post_compute();
    };

    class body_impl : public liftoff::body {
    private:
        int driver_idx;
        bool driver_updated;

        // TODO: Lockdown usage of this jesus christ
        std::vector<liftoff::vector> prev_state;

    public:
        explicit body_impl(double mass, int derivatives = 4, int driver_idx = 2);

        void set_position(const liftoff::vector &position);

        void set_velocity(const liftoff::vector &velocity);

        void set_acceleration(const liftoff::vector &acceleration);

        void pre_compute() override;

        void compute_forces(double elapsed_time) override;

        void compute_motion(double elapsed_time) override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_BODY_H
