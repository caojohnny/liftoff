#ifndef LIFTOFF_PHYSICS_MATRIX_H
#define LIFTOFF_PHYSICS_MATRIX_H

#include <gmpxx.h>
#include <vector>

namespace liftoff {
    /**
     * Represents a matrix of values consisting of GNU
     * Multiprecision Library values.
     */
    class matrix {
    private:
        std::vector<std::vector<mpf_class>> cells;
    public:
        /**
         * Creates a new square matrix with the given
         * dimensions.
         *
         * @param size the number of rows and columns for
         * this matrix
         */
        explicit matrix(size_t size);

        /**
         * Creates a new matrix with the given number of
         * rows and columns.
         *
         * @param rows the number of rows
         * @param columns the number of columns
         */
        matrix(size_t rows, size_t columns);

        /**
         * Determines the number of rows in this matrix.
         *
         * @return the number of rows
         */
        unsigned int rows() const;

        /**
         * Determines the number of columns in this matrix.
         *
         * @return the number of columns
         */
        unsigned int columns() const;

        /**
         * Row access operator using the array bracket
         * syntax.
         *
         * @param row the row index to obtain
         * @return the reference to the row
         */
        std::vector<std::vector<mpf_class>>::reference &operator[](size_t row);

        /**
         * Const row access operator using the array
         * bracket syntax.
         *
         * @param row the row to access
         * @return a const reference to the row
         */
        const std::vector<mpf_class> &operator[](size_t row) const;

        /**
         * Obtains the string format of the data in this
         * matrix which can be copied into MATLAB.
         *
         * @return a MATLAB formatted string version of
         * this matrix
         */
        std::string to_matlab() const;
    };
}

#endif // LIFTOFF_PHYSICS_MATRIX_H
