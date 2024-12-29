% Control Targets and Control Gains
P_d = [0.0; 0.0; 0.0];          % Target Position
euler_d = [0; 0.2127; 0];       % Target Euler Angles 

% Appropriate Parameters
K_P = diag([0.44, 0.44, 0.44]);                        % Position Control Gain 
K_v = diag([32.07, 32.07, 32.07]);                    % Velocity Control Gain
K_euler = diag([2.13, 2.09, 2.13]);                   % Attitude Control Gain
K_omega = diag([33.77, 33.78, 33.87]);                % Angular Velocity Control Gain

% High Gain
% K_P = diag([1, 1, 1]);                        % Position Control Gain 
% K_v = diag([35, 35, 35]);                    % Velocity Control Gain
% K_euler = diag([2.2, 2.2, 2.2]);             % Attitude Control Gain
% K_omega = diag([34, 34, 34]);                % Angular Velocity Control Gain

% Low Gain
% K_P = diag([0.2, 0.2, 0.2]);                 % Position Control Gain 
% K_v = diag([30, 30, 30]);                    % Velocity Control Gain
% K_euler = diag([2, 2, 2]);                   % Attitude Control Gain
% K_omega = diag([30, 30, 30]);                % Angular Velocity Control Gain

% Initial State
P_0 = [0.0; 0; 0];               % Initial Position
v_0 = [0; 0; 0];                 % Initial Velocity
euler_0 = [0.1; 0.1; 0.1];       % Initial Euler Angles (no rotation)
omega_0 = [0; 0; 0];             % Initial Angular Velocity
X0 = [P_0; v_0; euler_0; omega_0];
options = odeset('MaxStep', 0.005);

% Solve the Equations
tspan = 0:0.005:10;  % 200 Hz update frequency, corresponding to 5 ms step size
[t, X] = ode45(@(t, X) rigid_body_dynamics(t, X, P_d, euler_d, K_P, K_v, K_euler, K_omega), tspan, X0);
size(X, 1)

% Assume a time range from 0 to 10 seconds; adjust as needed
t = linspace(0, 10, size(X, 1));  % Time vector

% Extract Position Components
x = X(:, 1);
y = X(:, 2);
z = X(:, 3);

% Extract Euler Angle Components
phi = X(:, 7);     % Roll Angle
theta = X(:, 8);   % Pitch Angle
psi = X(:, 9);     % Yaw Angle

% Combine Data into a Matrix
data = [t', P_d(1)-x, P_d(2)-y, P_d(3)-z, euler_d(1)-phi, euler_d(2)-theta, euler_d(3)-psi];  % Transpose t to match dimensions

% Output to Text File
% filename = 'LQR_gain.txt';  % Output File Name
% writematrix(data, filename, 'Delimiter', '\t');  % Use tab as a delimiter

% Plot Position Errors Over Time
figure;
subplot(2, 1, 1); % Divide the figure into two parts; first part for position
plot(t, P_d(1)-x, 'r-', 'DisplayName', '$\it{e}_{x}$');
hold on;
plot(t, P_d(2)-y, 'g-', 'DisplayName', '$\it{e}_{y}$');
plot(t, P_d(3)-z, 'b-', 'DisplayName', '$\it{e}_{z}$');
hold off;
xlabel('Time (s)');
ylabel('Position Error (m)');
legend('Interpreter', 'latex', 'NumColumns', 3, 'FontSize', 18);
grid on;
ylim([-0.4 0.4])

% Plot Euler Angle Errors Over Time
subplot(2, 1, 2); % Second part for Euler angles
plot(t, euler_d(1)-phi, 'r-', 'DisplayName', '$\it{e}_{\phi}$');
hold on;
plot(t, euler_d(2)-theta, 'g-', 'DisplayName', '$\it{e}_{\theta}$');
plot(t, euler_d(3)-psi, 'b-', 'DisplayName', '$\it{e}_{\psi}$');
hold off;
xlabel('Time (s)');
ylabel('Attitude Error (rad)');
legend('Interpreter', 'latex', 'NumColumns', 3, 'FontSize', 18);
grid on;
ylim([-0.2 0.2])
