#ifndef LIFTOFF_VECTOR_H
#define LIFTOFF_VECTOR_H

namespace liftoff {
    class vector {
    private:
        double x;
        double y;
        double z;

    public:
        vector();
        vector(double k);
        vector(double x, double y, double z);

        double get_x() const;
        vector set_x(double x);
        double get_y() const;
        vector set_y(double y);
        double get_z() const;
        vector set_z(double z);

        vector set(const vector &vec);
        vector add(const vector &vec);
        vector sub(const vector &vec);
        vector mul(const vector &vec);
        vector div(const vector &vec);

        void print_vector();
    };
}

#endif // LIFTOFF_PHYSICS_VECTOR_H
