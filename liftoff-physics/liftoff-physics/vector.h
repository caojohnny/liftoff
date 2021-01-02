/**
 * @file
 */

#ifndef LIFTOFF_PHYSICS_VECTOR_H
#define LIFTOFF_PHYSICS_VECTOR_H

#include <string>

namespace liftoff {
    /**
     * @brief Represents a tuple of 3 floating-point values
     * in a physical vector.
     */
    class vector {
    private:
        /**
         * X coordinate of the vector.
         */
        double x;
        /**
         * Y coordinate of the vector.
         */
        double y;
        /**
         * Z coordinate of the vector.
         */
        double z;

    public:
        /**
         * Creates a new vector initialized to 0.
         */
        vector();

        /**
         * Creates a new vector whose components are
         * initialized to the value 'k'.
         *
         * @param k the value of all components
         */
        explicit vector(double k);

        /**
         * Creates a new vector with the given component
         * values.
         *
         * @param vec_x the X component value
         * @param vec_y the Y component value
         * @param vec_z the Z component value
         */
        vector(double vec_x, double vec_y, double vec_z);

        /**
         * Obtains the X component of the vector.
         *
         * @return the X component value
         */
        double get_x() const;

        /**
         * Sets the value of the X component of this
         * vector.
         *
         * @param new_x the new X component value
         * @return the instance of this vector
         */
        vector &set_x(double new_x);

        /**
         * Obtains the Y component of the vector.
         *
         * @return the Y component value
         */
        double get_y() const;

        /**
         * Sets the value of the Y component of this
         * vector.
         *
         * @param new_x the new Y component value
         * @return the instance of this vector
         */
        vector &set_y(double new_y);

        /**
         * Obtains the Z component of the vector.
         *
         * @return the Z component value
         */
        double get_z() const;

        /**
         * Sets the value of the Z component of this
         * vector.
         *
         * @param new_x the new Z component value
         * @return the instance of this vector
         */
        vector &set_z(double new_z);

        /**
         * Sets the components of this vector to those
         * of the given vector.
         *
         * @param vec the source vector
         * @return the instance of this vector
         */
        vector &set(const vector &vec);

        /**
         * Sets the components of this vector to the
         * existing component values plus those of the
         * given vector.
         *
         * @param vec the additive vector
         * @return the instance of this vector
         */
        vector &add(const vector &vec);

        /**
         * Sets the components of this vector to the
         * existing component values minus those of the
         * given vector.
         *
         * @param vec the subtracting vector
         * @return the instance of this vector
         */
        vector &sub(const vector &vec);

        /**
         * Sets the components of this vector to the
         * existing component values times those of the
         * given vector.
         *
         * @param vec the multiplying vector
         * @return the instance of this vector
         */
        vector &mul(const vector &vec);

        /**
         * Sets the components of this vector to the
         * existing component values divided those of the
         * given vector.
         *
         * @param vec the divisor vector
         * @return the instance of this vector
         */
        vector &div(const vector &vec);

        /**
         * Obtains the magnitude of this vector by taking
         * the 2-norm.
         *
         * @return the vector magnitude
         */
        double magnitude() const;

        /**
         * Creates a new string representing the values
         * contained in this vector.
         *
         * @return the string representation of this vector
         */
        explicit operator std::string() const;

        /**
         * Determines whether the given vector has
         * components that equal the components of this
         * vector.
         *
         * @param rhs the vector to check
         * @return true if the two vectors are equal
         */
        bool operator==(const vector &rhs) const;
    };
}

#endif // LIFTOFF_PHYSICS_VECTOR_H
