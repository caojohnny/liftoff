clf
clear

force_points = [ 0 100; 500 2700 ];

x = 0:500;

rng("shuffle");
raw_data = 0.01 .* x.^2 + (20 .* rand(1, length(x)));

poly_fit = polyfit_with_fixed_points(5, x, raw_data, force_points(:, 1)', force_points(:, 2)');
fit_data = polyval(poly_fit, x);

hold on
plot(x, raw_data, "g.");
plot(force_points(:, 1), force_points(:, 2), "b*")

plot(x, fit_data, "b--");
hold off

function result = take(mat, idx)
    rows = size(idx, 1);
    cols = size(idx, 2);
    result = zeros(rows, cols);
    for i = 1:rows
        for j = 1:cols
            result(i, j) = mat(idx(i, j));
        end
    end
end

% Adapted from: https://stackoverflow.com/questions/15191088/how-to-do-a-polynomial-fit-with-fixed-points
function poly = polyfit_with_fixed_points(n, x, y, xf, yf)
    mat = zeros(n + 1 + length(xf), n + 1 + length(xf));
    vec = zeros(n + 1 + length(xf), 1);
    
    x_n = repmat(x, 2 * n + 1, 1).^((0:2 * n)');
    yx_n = sum(x_n(1:n + 1, :) .* y, 2)';
    x_n = sum(x_n, 2)';
    idx = (0:n) + (0:n)' + ones(n + 1, n + 1);
    mat(1:n + 1, 1:n + 1) = take(x_n, idx);
    xf_n = repmat(xf, n + 1, 1).^((0:n)');
    
    mat(1:n + 1, n + 2:end) = xf_n / 2;
    mat(n + 2:end, 1:n + 1) = xf_n';
    mat(n + 2:end, n + 2:end) = 0;

    vec(1:n + 1) = yx_n;
    vec(n + 2:end) = yf;

    params = mat \ vec;
    poly = fliplr(params(1:n + 1)');
end
