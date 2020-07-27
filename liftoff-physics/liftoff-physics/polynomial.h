#ifndef LIFTOFF_PHYSICS_POLYNOMIAL_H
#define LIFTOFF_PHYSICS_POLYNOMIAL_H

#include <gmpxx.h>
#include <vector>

namespace liftoff {
    class polynomial {
    private:
        std::vector<mpf_class> coefficients;
    public:
        polynomial();

        explicit polynomial(size_t terms);

        explicit polynomial(const std::vector<mpf_class> &poly);

        explicit polynomial(const std::vector<double> &poly);

        void add_term(const mpf_class &coefficient);

        double val(double x) const;

        mpf_class val(const mpf_class &x) const;

        std::vector<mpf_class> get_coefficients() const;

        std::vector<mpf_class>::reference &operator[](size_t idx);

        std::string to_matlab(char suffix) const;
    };
}

#endif // LIFTOFF_PHYSICS_POLYNOMIAL_H
