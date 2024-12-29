function K_hat = compute_K_hat()

    K_hat_11 = 80.0687;   K_hat_12 = 80.0687;   K_hat_13 = 7.4048;   K_hat_14 = -7.4048;
    K_hat_21 = 0;         K_hat_22 = 0;         K_hat_23 = -85.8838; K_hat_24 = -85.8838;
    K_hat_31 = 31.0649;   K_hat_32 = 31.0649;   K_hat_33 = -19.0857; K_hat_34 = 19.0857;
    K_hat_41 = 6.9896;    K_hat_42 = -6.9896;   K_hat_43 = 22.0746;  K_hat_44 = 22.0746;
    K_hat_51 = 24.0373;   K_hat_52 = 24.0373;   K_hat_53 = 2.6090;   K_hat_54 = -2.6090;
    K_hat_61 = -18.0154;  K_hat_62 = 18.0154;   K_hat_63 = -3.1759;  K_hat_64 = -3.1759;

    K_hat = [
        K_hat_11, K_hat_12, K_hat_13, K_hat_14;
        K_hat_21, K_hat_22, K_hat_23, K_hat_24;
        K_hat_31, K_hat_32, K_hat_33, K_hat_34;
        K_hat_41, K_hat_42, K_hat_43, K_hat_44;
        K_hat_51, K_hat_52, K_hat_53, K_hat_54;
        K_hat_61, K_hat_62, K_hat_63, K_hat_64;
    ];
end
