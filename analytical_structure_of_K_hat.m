% Parameters
alpha_ref = -0.3701;
beta_ref = 0.2340;
theta_ref = 0.2127;
Tl_ref = 9 * 9.81;
Tr_ref = 9 * 9.81;
xl = 0.01758; yl = 0.225; zl = 0.30703;
xr = 0.01758; yr = -0.225; zr = 0.30703;

% Matrix elements calculation
K_hat_11 = Tl_ref * cos(alpha_ref) * cos(beta_ref);
K_hat_12 = Tr_ref * cos(alpha_ref) * cos(beta_ref);
K_hat_13 = -Tl_ref * sin(alpha_ref) * sin(beta_ref);
K_hat_14 = Tr_ref * sin(alpha_ref) * sin(beta_ref);

K_hat_21 = 0;
K_hat_22 = 0;
K_hat_23 = -Tl_ref * cos(beta_ref);
K_hat_24 = -Tr_ref * cos(beta_ref);

K_hat_31 = -Tl_ref * sin(alpha_ref) * cos(beta_ref);
K_hat_32 = -Tr_ref * sin(alpha_ref) * cos(beta_ref);
K_hat_33 = -Tl_ref * cos(alpha_ref) * sin(beta_ref);
K_hat_34 = Tr_ref * cos(alpha_ref) * sin(beta_ref);

K_hat_41 = -Tl_ref * yl * sin(alpha_ref) * cos(beta_ref);
K_hat_42 = -Tr_ref * yr * sin(alpha_ref) * cos(beta_ref);
K_hat_43 = Tl_ref * zl * cos(beta_ref) - Tl_ref * yl * cos(alpha_ref) * sin(beta_ref);
K_hat_44 = Tr_ref * zr * cos(beta_ref) + Tr_ref * yr * cos(alpha_ref) * sin(beta_ref);

K_hat_51 = Tl_ref * zl * cos(alpha_ref) * cos(beta_ref) + Tl_ref * xl * sin(alpha_ref) * cos(beta_ref);
K_hat_52 = Tr_ref * zr * cos(alpha_ref) * cos(beta_ref) + Tr_ref * xr * sin(alpha_ref) * cos(beta_ref);
K_hat_53 = -Tl_ref * zl * sin(alpha_ref) * sin(beta_ref) + Tl_ref * xl * cos(alpha_ref) * sin(beta_ref);
K_hat_54 = Tr_ref * zr * sin(alpha_ref) * sin(beta_ref) - Tr_ref * xr * cos(alpha_ref) * sin(beta_ref);

K_hat_61 = -Tl_ref * yl * cos(alpha_ref) * cos(beta_ref);
K_hat_62 = -Tr_ref * yr * cos(alpha_ref) * cos(beta_ref);
K_hat_63 = Tl_ref * yl * sin(alpha_ref) * sin(beta_ref) - Tl_ref * xl * cos(beta_ref);
K_hat_64 = -Tr_ref * yr * sin(alpha_ref) * sin(beta_ref) - Tr_ref * xr * cos(beta_ref);

% Construct the K_hat matrix
K_hat = [K_hat_11, K_hat_12, K_hat_13, K_hat_14;
         K_hat_21, K_hat_22, K_hat_23, K_hat_24;
         K_hat_31, K_hat_32, K_hat_33, K_hat_34;
         K_hat_41, K_hat_42, K_hat_43, K_hat_44;
         K_hat_51, K_hat_52, K_hat_53, K_hat_54;
         K_hat_61, K_hat_62, K_hat_63, K_hat_64];

% Display the K_hat matrix
disp('K_hat matrix:');
disp(K_hat);
