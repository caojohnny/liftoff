#ifndef LIFTOFF_PHYSICS_LINALG_H
#define LIFTOFF_PHYSICS_LINALG_H

#include <vector>
#include <gmpxx.h>
#include "polynomial.h"

namespace liftoff {
    liftoff::polynomial lip(const std::vector<std::pair<double, double>> &forced_points);

    liftoff::polynomial fit(unsigned int order,
                            const std::vector<double> &x,
                            const std::vector<double> &y,
                            const std::vector<std::pair<double, double>> &forced_points);
}

#endif // LIFTOFF_PHYSICS_LINALG_H
