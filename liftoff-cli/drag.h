#ifndef LIFTOFF_CLI_DRAG_H
#define LIFTOFF_CLI_DRAG_H

double calc_drag(double cd, double rho, double v, double a);

double calc_rho_earth(double alt);

double calc_drag_earth(double cd, double alt, double v, double a);

#endif // LIFTOFF_CLI_DRAG_H
