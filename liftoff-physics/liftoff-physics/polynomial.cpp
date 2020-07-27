#include "polynomial.h"

// Required precision for floating point values in order to perform the necessary polynomial
// regressions
// (pretty sure this is actually around 80, but 256 just to be safe :>)
static const int REQ_PRECISION = 256;

liftoff::polynomial::polynomial() = default;

liftoff::polynomial::polynomial(size_t terms) {
    coefficients.reserve(terms);
    for (int i = 0; i < terms; ++i) {
        coefficients.emplace_back(0, REQ_PRECISION);
    }
}

liftoff::polynomial::polynomial(const std::vector<mpf_class> &poly) {
    coefficients.reserve(poly.size());
    for (const auto &coeff : poly) {
        coefficients.emplace_back(coeff, REQ_PRECISION);
    }
}

liftoff::polynomial::polynomial(const std::vector<double> &poly) {
    coefficients.reserve(poly.size());
    for (const auto &coeff : poly) {
        coefficients.emplace_back(coeff, REQ_PRECISION);
    }
}

void liftoff::polynomial::add_term(const mpf_class &coefficient) {
    coefficients.emplace_back(coefficient, REQ_PRECISION);
}

double liftoff::polynomial::val(double x) const {
    mpf_class val{0, REQ_PRECISION};
    for (int i = 0; i < coefficients.size(); ++i) {
        mpf_class x_pow{x, REQ_PRECISION};
        mpf_pow_ui(x_pow.get_mpf_t(), x_pow.get_mpf_t(), i);

        val += coefficients[i] * x_pow;
    }

    return val.get_d();
}

mpf_class liftoff::polynomial::val(const mpf_class &x) const {
    mpf_class val{0, REQ_PRECISION};
    for (int i = 0; i < coefficients.size(); ++i) {
        mpf_class x_pow{x, REQ_PRECISION};
        mpf_pow_ui(x_pow.get_mpf_t(), x_pow.get_mpf_t(), i);

        val += coefficients[i] * x_pow;
    }

    return val;
}

std::vector<mpf_class> liftoff::polynomial::get_coefficients() const {
    return coefficients;
}

std::vector<mpf_class>::reference &liftoff::polynomial::operator[](size_t idx) {
    return coefficients[idx];
}

std::string liftoff::polynomial::to_matlab(char suffix) const {
    std::string result = "y";
    result += suffix;
    result += " = ";

    for (int i = 0; i < coefficients.size(); ++i) {
        result += std::to_string(coefficients[i].get_d());
        result += "*x";
        result += suffix;
        result += ".^";
        result += std::to_string(i);

        if (i != coefficients.size() - 1) {
            result += " + ";
        }
    }

    result += ";";
    return result;
}
