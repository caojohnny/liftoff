clear
clc
clf

total_time = 900; % 900 sec
time_step = 0.001; % 1 millis time between each step
steps = total_time / time_step; % step count
x_time = time_step:time_step:total_time; % x axis - array of time values

g = 9.81;
rocket_accel = g + 0.2;

y_profile = 100000 * sqrt(x_time); % y axis - desired flight profile (vertical motion)
a = rocket_accel * ones(1, steps); % constant acceleration profile
a_verification = zeros(1, steps);

p = zeros(2, steps); % position
v = zeros(2, steps); % velocity

p_error = zeros(1, steps); % y axis - (profile - p) error from flight profile

for i = 1:steps - 1
    current_error = y_profile(i + 1) - p(2, i); % vertical position error from flight profile
    current_velocity = sqrt(v(1, i)^2 + v(2, i)^2); % pythagorean theorem to compute mag of velocity
    [vx, vy] = compute_velocity_naive(current_velocity, current_error, time_step);

    % compute components of acceleration
    if current_velocity > 0 % cur velocity is non-zero, use ratio of velocity components
        dax = vx * a(i) / current_velocity;
        day = vy * a(i) / current_velocity;
    else % otherwise acceleration gets applied in the vertically
        dax = 0;
        day = a(i);
    end

    % horizontal motion
    p(1, i + 1) = p(1, i) + vx * time_step;
    v(1, i + 1) = vx + dax * time_step;

    % vertical motion
    p(2, i + 1) = p(2, i) + vy * time_step;
    v(2, i + 1) = vy + day * time_step;

    p_error(i + 1) = y_profile(i) - p(2, i);
end

% plot everything
hold on
plot(x_time, p(1, 1:end), "b:");
plot(x_time, p(2, 1:end), "b--");
plot(x_time, y_profile, "k-");
plot(x_time, p_error, "r-");
hold off

% naive function because this should realistically probably involve some
% kind of control algorithm such as PID
function [x_velocity, y_velocity] = compute_velocity_naive(mag, y_error, time_step)
    target_y_velocity = y_error / time_step; % velocity needed to bring
                                             % error down to 0 over the
                                             % next time_step

    if target_y_velocity >= mag % target velocity gt available velocity
                                % go as fast as allowed by provided
                                % magnitude
        x_velocity = 0;
        y_velocity = mag;
    else % target_y_velocity is smaller than our available velocity, we
         % need to point the velocity vector towards the horizontal
         % direction

        % use pythagorean theorem to determine x component based on the
        % clamped y value
        x_velocity = sqrt(mag^2 - target_y_velocity^2);
        y_velocity = target_y_velocity;
    end
end
