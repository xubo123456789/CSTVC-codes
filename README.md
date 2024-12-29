# CSTVC-codes
This is the code for the constant-strength-thrust-vector-control framework  (CSTVC) of a flying humanoid robot, which can be run in GNU Octave.

The file "analytical_structure_of_K_hat" contains the code for solving the elements of the matrix K_hat in the appendix. It corresponds to equation (9) in the article and equation (25) in the appendix.

The file "thrust_define_optimization" contains the code for solving the constant thrust magnitude using a simplified model and optimization method. It corresponds to equation (4) in the article.

The file "LQR_get_control_gain" contains the code for solving the CSTVC control gains using the Linear Quadratic Regulator (LQR) method. It corresponds to equations (17), (18), and (19) in the article.

The files "compute_K_hat", "compute_forces_and_torques", "rigid_body_dynamics", and "system_state_change" are numerical simulation programs corresponding to Section 4.1 "Simulation Experiment" in the article.

The "compute_K_hat" function is responsible for filling the elements of the K_hat matrix, corresponding to equation (25) in the article.

The "compute_forces_and_torques" function is the code for calculating the forces and torques acting on the robot's center of mass during flight, corresponding to equation (9) in the article.

The "rigid_body_dynamics" function contains the code implementation of the robot's flight controller, corresponding to equations (1), (5), (6), (12), (13), (14), (15), (16), and (20) in the article.

The "system_state_change" file contains the code implementation for the dynamic evolution of the robot's rigid body model, corresponding to equations (5) and (6) in the article.
