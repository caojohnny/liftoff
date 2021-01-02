/**
 * @file
 * @brief Helper functions for computing the drag force
 * based upon the standard atmosphere.
 */

#ifndef LIFTOFF_PHYSICS_DRAG_H
#define LIFTOFF_PHYSICS_DRAG_H

namespace liftoff {
    /**
     * Computes the drag force based upon the components
     * of aerodynamic drag.
     *
     * @param cd the drag coefficient
     * @param rho the atmospheric density, kg/kg^3
     * @param v the velocity of the object, m/s
     * @param a the cross-sectional area, m^2
     * @return the drag force, in Newtons
     */
    double calc_drag(double cd, double rho, double v, double a);

    /**
     * Computes the atmospheric pressure of the Earth based
     * upon NASA's standard atmospheric model.
     *
     * @param alt the altitude from the surface of the Earth,
     * in meters
     * @return the atmospheric pressure, in kPa
     */
    double calc_pressure_earth(double alt);

    /**
     * Computes the atmospheric density of the Earth based
     * upon NASA's standard atmospheric model.
     *
     * @param alt the altitude from the surface of the Earth,
     * in meters
     * @return the atmospheric density, in kg/m^3
     */
    double calc_rho_earth(double alt);

    /**
     * Computes the aerodynamic drag using the NASA's
     * standard atmospheric model to fill in for the
     * atmospheric density of the Earth.
     *
     * @param cd the coefficient of drag
     * @param alt the altitude above Earth's surface, m
     * @param v the velocity, m/s
     * @param a the cross-sectional area, m^2
     * @return the drag force, N
     */
    double calc_drag_earth(double cd, double alt, double v, double a);
}

#endif // LIFTOFF_PHYSICS_DRAG_H
