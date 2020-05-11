clear
clc
clf

total_time = 900; % 900 sec
time_step = 0.001; % 1 millis time between each step
steps = total_time / time_step; % step count
x_time = time_step:time_step:total_time; % x axis - array of time values

g = 9.81;
rocket_accel = g + 0.2;

y_profile = 0.7 * 100000 * sqrt(x_time); % y axis - desired flight profile (vertical motion)
a = rocket_accel * ones(1, steps); % constant acceleration profile
v_profile = x_time .* a; % constant acceleration velocity profile

p = zeros(2, steps); % position
p_no_fit = zeros(1, steps); % position, no fit towards profile
v = zeros(2, steps); % velocity
v_no_fit = zeros(1, steps); % velocity, no fit (helper for p_no_fit)

p_error = zeros(1, steps); % y axis - (profile - p) error from flight profile

for i = 1:steps - 1
    current_error = y_profile(i + 1) - p(2, i); % vertical position error from flight profile
    desired_v = v_profile(i);
    if i > 1
        cur_v = v([1 2], i-1);
    else
        cur_v = [0 0];
    end

    [vx, vy] = compute_velocity_naive(cur_v, desired_v, current_error, time_step);

    % horizontal motion
    p(1, i + 1) = p(1, i) + vx * time_step;
    v(1, i + 1) = vx;

    % vertical motion
    p(2, i + 1) = p(2, i) + vy * time_step;
    v(2, i + 1) = vy;

    % no fitting
    p_no_fit(i + 1) = p_no_fit(i) + v_no_fit(i) * time_step;
    v_no_fit(i + 1) = v_no_fit(i) + a(i) * time_step;

    p_error(i + 1) = y_profile(i) - p(2, i);
end

% plot everything
hold on
plot(x_time, p(1, 1:end), "b:");
plot(x_time, p(2, 1:end), "b--");
plot(x_time, p_no_fit, "r:");
plot(x_time, y_profile, "k-");
plot(x_time, p_error, "r-");
legend("X Pos", "Y Pos", "Unadjusted Y", "Profile", "Error", "Location", "northeast")
hold off

% naive function because this should realistically probably involve some
% kind of control algorithm such as PID
function [x_velocity, y_velocity] = compute_velocity_naive(cur_v, mag, y_error, time_step)
    y_velocity = min(mag, y_error / time_step);
    x_velocity = sqrt(mag^2 - y_velocity^2);
end
