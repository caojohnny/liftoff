#ifndef LIFTOFF_PHYSICS_MATRIX_H
#define LIFTOFF_PHYSICS_MATRIX_H

#include <gmpxx.h>
#include <vector>

namespace liftoff {
    class matrix {
    private:
        std::vector<std::vector<mpf_class>> cells;
    public:
        explicit matrix(size_t size);

        matrix(size_t rows, size_t columns);

        unsigned int rows() const;

        unsigned int columns() const;

        std::vector<std::vector<mpf_class>>::reference &operator[](size_t row);

        const std::vector<mpf_class> &operator[](size_t row) const;

        std::string to_matlab() const;
    };
}

#endif // LIFTOFF_PHYSICS_MATRIX_H
