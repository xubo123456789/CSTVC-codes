% Parameters
m = 30; % mass in kg
g = 9.81; % gravitational acceleration in m/s^2
L1 = 0.17166; % in meters
L2 = 0.30703; % in meters
L3 = 0.01758; % in meters

% Objective function
objective = @(x) x(1)^2 + x(2)^2 + 0.6 * x(3)^2;

% Nonlinear constraints
nonlcon = @(x) deal([], [
    x(3) * sin(x(4)) + (x(1) + x(2)) * cos(x(6)) * sin(x(4) + x(5)); % First constraint
    x(3) * cos(x(4)) + (x(1) + x(2)) * cos(x(6)) * cos(x(4) + x(5)) - m * g; % Second constraint
    x(3) * L1 + (x(1) + x(2)) * sin(x(5)) * L2 - (x(1) + x(2)) * cos(x(5)) * L3; % Third constraint
    x(1) - x(2) % Fourth constraint (Tl = Tr)
]);

% Initial guess for [Tl, Tr, Tb, theta_ref, alpha_ref, beta_ref]
x0 = [1, 1, 1, 0.1, -0.1, 0.1];

% Set options for fmincon
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% Bounds for variables
lb = [0, 0, 0, 0, -pi/2, 0]; % Lower bounds
ub = [14.8 * g, 14.8 * g, 19.6 * g, pi/2, 0, pi/2]; % Upper bounds

% Run fmincon to solve the optimization problem
[x_opt, fval] = fmincon(objective, x0, [], [], [], [], lb, ub, nonlcon, options);

% Display the optimized values
disp('Optimized values:')
disp(['Tl = ', num2str(x_opt(1))])
disp(['Tr = ', num2str(x_opt(2))])
disp(['Tb = ', num2str(x_opt(3))])
disp(['theta_ref = ', num2str(x_opt(4))])
disp(['alpha_ref = ', num2str(x_opt(5))])
disp(['beta_ref = ', num2str(x_opt(6))])

disp(['Objective function value = ', num2str(fval)])
