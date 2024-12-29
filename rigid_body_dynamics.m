function dXdt = rigid_body_dynamics(t, X, P_d, euler_d, K_P, K_v, K_euler, K_omega)
    m = 30;    
    g = 9.81;
    e3 = [0; 0; 1];
    eta_ref = [-0.3701; -0.3701; 0.2340; -0.2340];
    I_B = [2.95, 0.00, -0.12;  % Inertia Matrix
           0.00, 2.43, 0.00;
          -0.12, 0.00, 0.80];
   % Extract State Variables
    P_m = X(1:3);            % Position in the Inertial Frame
    v_B_m = X(4:6);          % Velocity in the Body Frame
    euler_m = X(7:9);        % Euler Angles [phi; theta; psi]
    omega_B_m = X(10:12);    % Angular Velocity (Body Frame)

    % Extract Euler angles
    phi = euler_m(1); theta = euler_m(2); psi = euler_m(3);

    % Calculate the rotation matrix (Body Frame to Inertial Frame)
    Rx = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)];

    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];

    Rz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
          0, 0, 1];

    R_IB = Rx' * Ry' * Rz'; % Calculate R_IB_ref, transpose to obtain the inverse
    R_BI = R_IB';
    phi_ref = 0; theta_ref = 0.2127; psi_ref = 0;
    Rx_ref = [1, 0, 0;
              0, cos(phi_ref), -sin(phi_ref);
              0, sin(phi_ref), cos(phi_ref)];

    Ry_ref = [cos(theta_ref), 0, sin(theta_ref);
              0, 1, 0;
              -sin(theta_ref), 0, cos(theta_ref)];

    Rz_ref = [cos(psi_ref), -sin(psi_ref), 0;
              sin(psi_ref), cos(psi_ref), 0;
              0, 0, 1];

    R_IB_ref = Rx_ref' * Ry_ref' * Rz_ref'; % Calculate R_IB_ref, transpose to obtain the inverse
    R_BI_ref = R_IB_ref';

    W = [1, 0, -sin(theta);
         0, cos(phi), sin(phi) * cos(theta);
         0, -sin(phi), cos(phi) * cos(theta)];

    % Position control law (PD control), calculate control force in the Body Frame
    v_B_d = R_IB * K_P * (P_d - P_m);
    dv_B_d = K_v * (v_B_d - v_B_m);

    % Attitude control law (PD control)
    omega_B_d = W * K_euler * (euler_d - euler_m);
    domega_B_d = K_omega * (omega_B_d - omega_B_m);

    % Calculate linear acceleration term, including gravity compensation
    linear_acceleration_term = dv_B_d - g * (R_IB_ref * e3 - R_IB * e3);

    % Combine linear acceleration and angular acceleration terms
    control_input_vector = [linear_acceleration_term; domega_B_d];

    K_hat = compute_K_hat();
    % Construct diagonal matrix for mass-related terms
    mass_diag = diag([1/m, 1/m, 1/m]);
    % Construct K matrix
    K = [mass_diag, zeros(3, 3); zeros(3, 3), inv(I_B)] * K_hat;
    Ka = pinv(K);
    % Calculate eta_d
    eta_d = eta_ref + Ka * control_input_vector;
    [F_body, tau_body] = compute_forces_and_torques(eta_d(1), eta_d(2), eta_d(3), eta_d(4), phi, theta);

    % Translational equation: acceleration (Body Frame)
    d_v_B_m_dt = F_body / m;
    % Convert velocity to the Inertial Frame to update position
    v_inertial = R_BI * v_B_m;                   % Linear velocity in the Inertial Frame
    % Euler angle differential equation
    d_euler_m_dt = inv(W) * omega_B_m;
    % Rotational equation
    d_omega_B_m_dt = inv(I_B) * (tau_body - cross(omega_B_m, I_B * omega_B_m));
    % Combine differential equations
    dXdt = [v_inertial; d_v_B_m_dt; d_euler_m_dt; d_omega_B_m_dt];
end
