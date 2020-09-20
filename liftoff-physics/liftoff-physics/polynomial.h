#ifndef LIFTOFF_PHYSICS_POLYNOMIAL_H
#define LIFTOFF_PHYSICS_POLYNOMIAL_H

#include <gmpxx.h>
#include <vector>

namespace liftoff {
    /**
     * Represents a polynomial function.
     */
    class polynomial {
    private:
        std::vector<mpf_class> coefficients;
    public:
        /**
         * Initializes an empty polynomial.
         */
        polynomial();

        /**
         * Creates a polynomial with the given number of
         * terms.
         *
         * @param terms the polynomial order + 1
         */
        explicit polynomial(size_t terms);

        /**
         * Creates a polynomial with the given collection
         * of coefficients with which to contain initially.
         *
         * @param poly the coefficients to initialize this
         * polynomial
         */
        explicit polynomial(const std::vector<mpf_class> &poly);

        /**
         * Creates a polynomial with the given collection
         * of coefficients with which to contain initially.
         *
         * @param poly the coefficients to initialize this
         * polynomial
         */
        explicit polynomial(const std::vector<double> &poly);

        /**
         * Adds a new term to this polynomial. This
         * increases the polynomial order by 1 and will
         * be the coefficient of the next power at the
         * end of the polynomial.
         *
         * @param coefficient the coefficient constant
         * value
         */
        void add_term(const mpf_class &coefficient);

        /**
         * Computes the value of the function with the
         * given value of X substituted.
         *
         * @param x the function input
         * @return the function output
         */
        double val(double x) const;

        /**
         * Computes the value of the function with the
         * given value of X substituted.
         *
         * @param x the function input
         * @return the function output
         */
        mpf_class val(const mpf_class &x) const;

        /**
         * Obtains the full collection of coefficients for
         * this polynomial.
         *
         * @return the collection of coefficients, where
         * the 0th element is constant, the next the power
         * of 1 coefficient and so on
         */
        std::vector<mpf_class> get_coefficients() const;

        /**
         * Obtains the coefficient at the given order in
         * this polynomial.
         *
         * @param idx the order of the coefficient
         * @return the coefficient value
         */
        std::vector<mpf_class>::reference &operator[](size_t idx);

        /**
         * Creates a string containing the data in this
         * polynomial that can be used to transform the 'x'
         * variable in MATLAB.
         *
         * @param suffix the variable suffix to print
         * @return the MATLAB polynomial function string
         */
        std::string to_matlab(char suffix) const;
    };
}

#endif // LIFTOFF_PHYSICS_POLYNOMIAL_H
