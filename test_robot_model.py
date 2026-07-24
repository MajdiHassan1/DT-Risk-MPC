#!/usr/bin/env python3

"""
test_robot_model.py

Mobile Robot Kinematic Model Integration Test

"""

import casadi as ca

try:
    from .robot_model import DifferentialDriveModel
except ImportError:
    from robot_model import DifferentialDriveModel


def main():
    # Instantiate Differential Drive Model with Ts = 0.10 s (Table I Alignment)
    model = DifferentialDriveModel(dt=0.10)

    # Initial Robot State: x = [x, y, theta]
    x = ca.DM([0.0, 0.0, 0.0])

    # Control Input: u = [v, omega] (e.g., v = 0.2 m/s, omega = 0.5 rad/s)
    u = ca.DM([0.2, 0.5])

    # Compute Next State via Discrete Kinematic Propagation (Eqs. 6-8)
    x_next = model.discrete_model(x, u)


    print(" Differential Drive Kinematic Forward Integration")

    print(f"Sampling Period (Ts) : {model.dt} s")
    print(f"Initial State (x0)   : {x.T}")
    print(f"Control Input (u)    : {u.T}")
    print("----------------------------------------------")
    print("Next State Output (x_next):")
    print(x_next)



if __name__ == "__main__":
    main()