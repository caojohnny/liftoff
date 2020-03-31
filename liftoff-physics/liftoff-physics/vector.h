#ifndef LIFTOFF_PHYSICS_VECTOR_H
#define LIFTOFF_PHYSICS_VECTOR_H

#include <string>

namespace liftoff {
    class vector {
    private:
        double x;
        double y;
        double z;

    public:
        vector();

        explicit vector(double k);

        vector(double vec_x, double vec_y, double vec_z);

        double get_x() const;

        vector &set_x(double new_x);

        double get_y() const;

        vector &set_y(double new_y);

        double get_z() const;

        vector &set_z(double new_z);

        vector &set(const vector &vec);

        vector &add(const vector &vec);

        vector &sub(const vector &vec);

        vector &mul(const vector &vec);

        vector &div(const vector &vec);

        double magnitude() const;

        explicit operator std::string() const;

        bool operator==(const vector &rhs) const;
    };
}

#endif // LIFTOFF_PHYSICS_VECTOR_H
