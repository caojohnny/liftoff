clear
        clc
clf

        F9_CD = 0.25;
% https://space.stackexchange.com/questions/16883/whats-the-atmospheric-drag-coefficient-of-a-falcon-9-at-launch-sub-sonic-larg#16885
F9_A = pi * 2.6 ^2;
% https://www.spacex.com/sites/spacex/files/falcon_users_guide_10_2019.pdf

steps = 2000;

x = 1
:
steps;
y_drag = zeros(1, steps);

for
i = 1
:

steps
y_drag(i)

=
calc_drag_earth(F9_CD,
0,
x(i), F9_A
);

end

plot(x, y_drag);

% https://www.grc.nasa.gov/WWW/K-12/airplane/atmosmet.html#
function ideal_rho = calc_rho_ideal_state(p, T)
ideal_rho = p / (.2869 * (T + 273.1));
end

% https://www.grc.nasa.gov/WWW/K-12/airplane/atmosmet.html#
function rho = calc_rho_earth(alt)
rho = -1;

if alt >= 25000
T = -131.21 + .00299 * alt;
p = 2.488 * ((T + 273.1) / 216.6) ^-11.388;
rho = calc_rho_ideal_state(p, T);
elseif alt
>= 11000 && alt < 25000
T = -56.46;
p = 22.65 * exp(1.73 - .000157 * alt);
rho = calc_rho_ideal_state(p, T);
elseif alt<
11000
T = 15.04 - .00649 * alt;
p = 101.29 * ((T + 273.1) / 288.08) ^5.256;
rho = calc_rho_ideal_state(p, T);
end
        end

% https://www.grc.nasa.gov/WWW/K-12/airplane/atmosmet.html#
function drag = calc_drag_earth(cd, alt, v, a)
drag = cd * calc_rho_earth(alt) * v * v / 2 * a;
end
