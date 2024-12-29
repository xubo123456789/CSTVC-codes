% Set angle parameters
phi_ref = 0;
theta_ref = 0.2127; 
psi_ref = 0;

% Calculate the R_IB_ref matrix
Rx = [1, 0, 0;
      0, cos(phi_ref), -sin(phi_ref);
      0, sin(phi_ref), cos(phi_ref)];

Ry = [cos(theta_ref), 0, sin(theta_ref);
      0, 1, 0;
      -sin(theta_ref), 0, cos(theta_ref)];

Rz = [cos(psi_ref), -sin(psi_ref), 0;
      sin(psi_ref), cos(psi_ref), 0;
      0, 0, 1];

R_IB_ref = Rx' * Ry' * Rz'; % Calculate R_IB_ref, transpose to obtain the inverse
R_BI_ref = R_IB_ref';

% Calculate the W_ref matrix
W_ref = [1, 0, -sin(theta_ref);
         0, cos(phi_ref), cos(theta_ref) * sin(phi_ref);
         0, -sin(phi_ref), cos(theta_ref) * cos(phi_ref)];

% Construct the state matrix A and the control matrix B
A = [zeros(3), zeros(3), -R_BI_ref, zeros(3);
     zeros(3), zeros(3), zeros(3), -inv(W_ref);
     zeros(3), zeros(3), zeros(3), zeros(3);
     zeros(3), zeros(3), zeros(3), zeros(3)];

B = [zeros(3), zeros(3);
     zeros(3), zeros(3);
     eye(3), zeros(3);
     zeros(3), eye(3)];

% Define the LQR weight matrices Q and R
Q = diag([200, 200, 200, 5000, 5000, 5000, 1000, 1000, 1000, 1000, 1000, 1000]); 
R = diag([1, 1, 1, 1, 1, 1]); 

K = lqr(A, B, Q, R);

K_ref = -K;
K_v = -K_ref(1:3,7:9); % Velocity control gain
K_P = R_BI_ref*inv(K_v)*K_ref(1:3,1:3); % Position control gain
K_omega = -K_ref(4:6,10:12); % Angular velocity control gain
K_phi = inv(W_ref)*inv(K_omega)*K_ref(4:6,4:6); % Attitude control gain

disp('K_ref:');
disp(K_ref);
disp('Position control gain K_P:');
disp(K_P);
disp('Velocity control gain K_v:');
disp(K_v);
disp('Attitude control gain K_phi (K_xita):');
disp(K_phi);
disp('Angular velocity control gain K_omega:');
disp(K_omega);
