#ifndef LIFTOFF_BODY_H
#define LIFTOFF_BODY_H

#include <vector>

#include "vector.h"

namespace liftoff {
    class body {
    protected:
        double mass;
        std::vector<liftoff::vector *> forces;
        std::vector<vector *> d_mot;

    public:
        body(double mass, int derivatives);

        double get_mass() const;

        void set_mass(double mass);

        const std::vector<liftoff::vector *> &get_forces() const;

        const std::vector<liftoff::vector *> &get_d_mot() const;

        virtual void pre_compute();

        virtual void compute_forces(double elapsed_time);

        virtual void compute_motion(double elapsed_time);

        virtual void post_compute();
    };

    class body_impl : public liftoff::body {
    private:
        int driver_idx;

    public:
        explicit body_impl(double mass);

        body_impl(double mass, int derivatives, int driver_idx);

        ~body_impl();

        void set_position(const liftoff::vector &position);

        void pre_compute() override;

        void compute_forces(double elapsed_time) override;

        void compute_motion(double elapsed_time) override;

        void post_compute() override;
    };
}

#endif // LIFTOFF_PHYSICS_BODY_H
