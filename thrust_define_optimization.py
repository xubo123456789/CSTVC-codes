import numpy as np
from scipy.optimize import minimize

# Parameters
m = 30  # mass in kg
g = 9.81  # gravitational acceleration in m/s^2
L1 = 0.17166
L2 = 0.30703
L3 = 0.01758

# Objective function
def objective(x):
    Tl, Tr, Tb = x[0], x[1], x[2]
    return Tl**2 + Tr**2 + 0.6 * Tb**2

# Constraints
def constraints(x):
    Tl, Tr, Tb, theta_ref, alpha_ref, beta_ref = x
    Ta = (Tl + Tr) * np.cos(beta_ref)
    c1 = Tb * np.sin(theta_ref) + Ta * np.sin(theta_ref + alpha_ref)
    c2 = Tb * np.cos(theta_ref) + Ta * np.cos(theta_ref + alpha_ref) - m * g
    c3 = Tb * L1 + Ta * np.sin(alpha_ref) * L2 - Ta * np.cos(alpha_ref) * L3
    c4 = Tl - Tr  # Tl = Tr
    return [c1, c2, c3, c4]

# Bounds and constraints
bounds = [
    (0, 14.8 * g),  # Tl
    (0, 14.8 * g),  # Tr
    (0, 19.6 * g),  # Tb
    (0, np.pi / 2),  # theta_ref
    (-np.pi / 2, 0),  # alpha_ref
    (0, np.pi / 2),  # beta_ref
]

# Initial guess
x0 = [10, 10, 10, np.pi / 4, -np.pi / 4, np.pi / 4]

# Solve using COBYLA
result = minimize(
    objective,
    x0,
    method="SLSQP",
    constraints={"type": "eq", "fun": lambda x: constraints(x)},
    bounds=bounds,
    options={"disp": True},
)

# Display the results
if result.success:
    print("Optimized values:")
    print(f"Tl = {result.x[0]:.4f}")
    print(f"Tr = {result.x[1]:.4f}")
    print(f"Tb = {result.x[2]:.4f}")
    print(f"theta_ref = {result.x[3]:.4f}")
    print(f"alpha_ref = {result.x[4]:.4f}")
    print(f"beta_ref = {result.x[5]:.4f}")
    print(f"Objective function value = {result.fun:.4f}")
else:
    print("Optimization failed:", result.message)
