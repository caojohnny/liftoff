#ifndef LIFTOFF_PHYSICS_DRAG_H
#define LIFTOFF_PHYSICS_DRAG_H

namespace liftoff {
    double calc_drag(double cd, double rho, double v, double a);

    double calc_pressure_earth(double alt);

    double calc_rho_earth(double alt);

    double calc_drag_earth(double cd, double alt, double v, double a);
}

#endif // LIFTOFF_PHYSICS_DRAG_H
