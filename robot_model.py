#!/usr/bin/env python3

"""
robot_model.py

Nonholonomic Wheeled Mobile Robot Kinematic Model

Author : Majdi Hassan et al.

Description
-----------
Provides continuous-time dynamics and forward Euler discrete-time
state propagation for a nonholonomic unicycle mobile robot platform
(e.g., TurtleBot3 Burger) operating in a 2D global frame.

State:
    x = [x_r, y_r, theta_r]^T  (Position & absolute heading)

Control:
    u = [v, omega]^T           (Linear & angular velocity)
"""

import casadi as ca


class DifferentialDriveModel:
    """
    Nonholonomic Mobile Robot Kinematic Model (Manuscript Section IV.A)
    """

    def __init__(self, dt=0.10):
        """
        Parameters
        ----------
        dt : float
            Sampling period Ts in seconds (Table I Alignment: Ts = 0.10 s)
        """
        self.dt = dt

        self.nx = 3  # State dimension: [x, y, theta]
        self.nu = 2  # Control dimension: [v, omega]

    def dynamics(self, x, u):
        """
        Continuous-Time Kinematic Equations (Equations 3-5)

        dx/dt = v * cos(theta)
        dy/dt = v * sin(theta)
        dtheta/dt = omega
        """
        theta = x[2]
        v = u[0]
        omega = u[1]

        dx = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            omega
        )

        return dx

    def discrete_model(self, x, u):
        """
        Discrete-Time Kinematic Forward Integration (Equations 6-8)

        Applies Forward Euler Discretization with sampling period Ts.

        x(k+1)     = x(k) + v(k) * cos(theta(k)) * Ts
        y(k+1)     = y(k) + v(k) * sin(theta(k)) * Ts
        theta(k+1) = theta(k) + omega(k) * Ts
        """
        return x + self.dt * self.dynamics(x, u)

# Standalone Integration Verification
def main():
    print("\n==============================================")
    print("  Mobile Robot Kinematic Model Verification")
    print("==============================================\n")

    model = DifferentialDriveModel(dt=0.10)

    print(f"Sampling Period (Ts) : {model.dt} s")
    print(f"State Vector (nx)   : {model.nx} [x, y, theta]")
    print(f"Control Vector (nu) : {model.nu} [v, omega]")

    # Example Initial State and Test Input
    x0 = ca.DM([0.0, 2.0, 0.0])  # Start pose (0, 2, 0)
    u0 = ca.DM([1.0, 0.5])  # Test input: v = 1.0 m/s, omega = 0.5 rad/s

    x_next = model.discrete_model(x0, u0)

    print("\nInitial State  :", x0)
    print("Applied Input  :", u0)
    print("Next State     :", x_next)
    print("\n==============================================\n")


if __name__ == "__main__":
    main()