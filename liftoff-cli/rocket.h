/**
 * @file
 */

#ifndef LIFTOFF_CLI_ROCKET_H
#define LIFTOFF_CLI_ROCKET_H

#include <vector>

#include <liftoff-physics/force_driven_body.h>

#include "engine.h"

/**
 * @brief Represents a rocket, which is a force-drive body
 * possessing propellant mass and engines.
 */
class rocket : public liftoff::force_driven_body {
private:
    /**
     * The remaining mass of propellant.
     */
    double prop_mass;
    /**
     * The collection of engines present on the rocket.
     */
    std::vector<engine> engines;

public:
    /**
     * Creates a new rocket with the given parameters for
     * the rocket and body physical properties.
     *
     * @param rocket_dry_mass the dry mass of the rocket
     * @param rocket_prop_mass the fuel mass of the rocket
     * @param rocket_engines the powerplant for the rocket
     * @param fdb_derivatives the number of derivatives for
     * the force-driven body
     * @param fdb_time_step the time step used by the
     * force-driven body
     */
    rocket(double rocket_dry_mass, double rocket_prop_mass, std::vector<engine> rocket_engines,
           int fdb_derivatives = 4, double fdb_time_step = 1);

    /**
     * Obtains the total mass (dry mass + the fuel mass) of
     * the rocket.
     *
     * @return the current mass of the rocket
     */
    double get_mass() const override;

    /**
     * Obtains the remaining propellant mass.
     *
     * @return the remaining propellant mass
     */
    double get_prop_mass() const;

    /**
     * Sets the current propellant mass present on the
     * rocket.
     *
     * @param new_prop_mass the new propellant mass toset
     */
    void set_prop_mass(double new_prop_mass);

    /**
     * Subtracts the given amount of propellant mass from
     * the rocket.
     *
     * @param drain_mass the amount of mass to drain
     */
    void drain_propellant(double drain_mass);

    /**
     * Obtains the collection of engines used by this
     * rocket.
     *
     * @return the collection of engines
     */
    std::vector<engine> &get_engines();
};


#endif // LIFTOFF_CLI_ROCKET_H
