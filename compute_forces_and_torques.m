function [F, tau] = compute_forces_and_torques(alpha_l, alpha_r, beta_l, beta_r, phi, theta)
    % input：
    T_l = 9*9.81; T_r = 9*9.81; T_b = 13*9.81;   % Thrust
    m = 30;                         % mass
    g = 9.81;                       % Gravitational Acceleration
    xl = 0.01758; yl = 0.225; zl = 0.30703;
    xr = 0.01758; yr = -0.225; zr = 0.30703;
    xb=-0.17166;yb=0;

    % Fx, Fy, Fz calculation
    Fx = T_l * sin(alpha_l) * cos(beta_l) + T_r * sin(alpha_r) * cos(beta_r) + m * g * sin(theta);
    Fy = -T_l * sin(beta_l) - T_r * sin(beta_r) - m * g * sin(phi) * cos(theta);
    Fz = T_l * cos(alpha_l) * cos(beta_l) + T_r * cos(alpha_r) * cos(beta_r) + T_b - m * g * cos(phi) * cos(theta);
    F = [Fx; Fy; Fz]; % Force Vector

    % tau_x, tau_y, tau_z calculation
    tau_x = T_l * zl * sin(beta_l) + T_l * yl * cos(alpha_l) * cos(beta_l) + T_r * zr * sin(beta_r) + T_r * yr * cos(alpha_r) * cos(beta_r) + T_b * yb;
    tau_y = T_l * zl * sin(alpha_l) * cos(beta_l) - T_l * xl * cos(alpha_l) * cos(beta_l) + T_r * zr * sin(alpha_r) * cos(beta_r) - T_r * xr * cos(alpha_r) * cos(beta_r) - T_b * xb;
    tau_z = -T_l * yl * sin(alpha_l) * cos(beta_l) - T_l * xl * sin(beta_l) - T_r * yr * sin(alpha_r) * cos(beta_r) - T_r * xr * sin(beta_r);
    tau = [tau_x; tau_y; tau_z]; % Torque Vector
end
