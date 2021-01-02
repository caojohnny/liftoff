/**
 * @file
 * @brief Curve-fitting functions
 */

#ifndef LIFTOFF_PHYSICS_LINALG_H
#define LIFTOFF_PHYSICS_LINALG_H

#include <vector>
#include <gmpxx.h>
#include "polynomial.h"

namespace liftoff {
    /**
     * Computes a Laplace interpolating polynomial which
     * forces a polynomial through the given collection of
     * X-Y coordinates.
     *
     * @param forced_points the points which to force the
     * polynomial through
     * @return the resulting polynomial
     */
    liftoff::polynomial lip(const std::vector<std::pair<double, double>> &forced_points);

    /**
     * Computes a polynomial regression of the given data
     * points and the given points which the resulting
     * polynomial is forced to pass through.
     *
     * @param order the order of the computed polynomial
     * @param x the X points to perform regression
     * @param y the Y points to perform regression
     * @param forced_points the points which to force the
     * resulting polynomial through
     * @return the polynomial regression of the given order
     */
    liftoff::polynomial fit(unsigned int order,
                            const std::vector<double> &x,
                            const std::vector<double> &y,
                            const std::vector<std::pair<double, double>> &forced_points);
}

#endif // LIFTOFF_PHYSICS_LINALG_H
