#!/usr/bin/env python3

"""
mpc_solver.py

Risk-Aware Nonlinear Model Predictive Controller (NMPC)


Author : Majdi Hassan

Robot:
    TurtleBot3 Mobile Robot

State:
    x = [x, y, theta]

Control:
    u = [v, omega]
======================================================================
"""

import casadi as ca
import numpy as np

try:
    from .robot_model import DifferentialDriveModel
except ImportError:
    from robot_model import DifferentialDriveModel


class MPCSolver:


    # Constructor

    def __init__(self, horizon=20, dt=0.10):


        # Prediction Parameters (Table I)
        self.N = horizon  # Np = 20
        self.dt = dt      # Ts = 0.10 s


        # Robot Kinematic Model
        self.model = DifferentialDriveModel(dt)
        self.nx = self.model.nx
        self.nu = self.model.nu

        # State & Control Weighting Matrices (Table I Alignment)
        # Q = diag([10, 10, 1])
        self.Q = ca.diagcat(10.0, 10.0, 1.0)

        # Qf = Terminal State Weight Matrix
        self.Qf = ca.diagcat(20.0, 20.0, 2.0)

        # R = diag([0.1, 0.05])
        self.R = ca.diagcat(0.1, 0.05)

        # Rd = Rate of change control penalty
        self.Rd = ca.diagcat(0.5, 0.1)

        # Risk & Safety Penalties (Table I Alignment)
        self.lambda_risk = 15.5   # Collision-Risk Scaling Factor (\lambda)
        self.rho_slack = 500.0     # Slack Variable Quadratic Penalty Weight (\rho)
        self.sigma = 0.35          # Risk-Sensitivity Parameter (\sigma)
        self.safe_distance = 0.250 # Safety Distance Threshold (d_safe = 0.250 m)

        # Actuator Velocity Bounds (Table I)
        self.v_min = 0.0
        self.v_max = 1.0   # Maximum Linear Velocity Limit (1.0 m/s)

        self.w_min = -1.5  # Minimum Angular Velocity Limit (-1.5 rad/s)
        self.w_max = 1.5   # Maximum Angular Velocity Limit (1.5 rad/s)

        self.max_obstacles = 10

        # Optimization Variables
        self.X = ca.SX.sym("X", self.nx, self.N + 1)
        self.U = ca.SX.sym("U", self.nu, self.N)
        self.Eps = ca.SX.sym("Eps", self.N)  # Soft-constraint slack variables

        # Parameter Vector Inputs
        self.X0 = ca.SX.sym("X0", self.nx)
        self.Xref = ca.SX.sym("Xref", self.nx)
        self.Uprev = ca.SX.sym("Uprev", self.nu)
        self.risk = ca.SX.sym("risk")
        self.obstacles = ca.SX.sym("obstacles", 2, self.max_obstacles)

        # Optimization Containers
        self.cost = 0.0
        self.constraints = []

        self.lbg = []
        self.ubg = []

        self.lbx = []
        self.ubx = []

        # Decision Variables Vector
        self.variables = ca.vertcat(
            ca.reshape(self.X, -1, 1),
            ca.reshape(self.U, -1, 1),
            ca.reshape(self.Eps, -1, 1)
        )

        # Parameter Vector Assembly
        self.parameters = ca.vertcat(
            self.X0,
            self.Xref,
            self.Uprev,
            self.risk,
            ca.reshape(self.obstacles, -1, 1)
        )

        # Warm Start & Memory Buffers
        self.initial_guess = np.zeros(self.variables.numel())
        self.previous_control = np.zeros(self.nu)
        self.predicted_path = []
        self.solver = None

        # Build NLP Optimizer
        self.build_optimizer()

    # Build Optimization Problem (Manuscript Section IV.E)

    def build_optimizer(self):

        self.cost = 0.0
        self.constraints = []

        self.lbg = []
        self.ubg = []

        self.lbx = []
        self.ubx = []

        # Initial State Equality Constraint: X(:, 0) == X0
        self.constraints.append(self.X[:, 0] - self.X0)
        self.lbg.extend([0.0] * self.nx)
        self.ubg.extend([0.0] * self.nx)

        previous_u = self.Uprev

        # Loop over prediction horizon N
        for k in range(self.N):

            xk = self.X[:, k]
            uk = self.U[:, k]
            eps_k = self.Eps[k]
            x_next = self.X[:, k + 1]

            # Dynamic System Model Forward Propagation (Eqs. 6-8)
            x_predict = self.model.discrete_model(xk, uk)

            # Kinematic Continuity Constraint
            self.constraints.append(x_next - x_predict)
            self.lbg.extend([0.0] * self.nx)
            self.ubg.extend([0.0] * self.nx)

            # Desired Heading Angle calculation
            theta_ref = ca.atan2(
                self.Xref[1] - xk[1],
                self.Xref[0] - xk[0]
            )

            # State Tracking Error Vector e(k+i|k) (Eq. 20)
            error = ca.vertcat(
                xk[0] - self.Xref[0],
                xk[1] - self.Xref[1],
                xk[2] - theta_ref
            )

            # 1. State Tracking Cost: e' * Q * e
            self.cost += ca.mtimes([error.T, self.Q, error])

            # 2. Control Effort Cost: u' * R * u
            self.cost += ca.mtimes([uk.T, self.R, uk])

            # 3. Control Smoothness Increment Cost: delta_u' * Rd * delta_u
            delta_u = uk - previous_u
            self.cost += ca.mtimes([delta_u.T, self.Rd, delta_u])

            previous_u = uk

            # 4. Collision Risk & Soft Constraint Penalty Cost (Eq. 21)
            self.cost += self.lambda_risk * self.risk * (uk[0]**2 + 0.5 * uk[1]**2)
            self.cost += self.rho_slack * (eps_k**2)

            # 5. Obstacle Avoidance Soft Constraints (Eq. 24): d_i >= d_safe - eps_i
            for i in range(self.max_obstacles):
                ox = self.obstacles[0, i]
                oy = self.obstacles[1, i]

                obstacle_exists = ca.if_else(
                    ca.fabs(ox) + ca.fabs(oy) < 1e-6,
                    0,
                    1
                )

                dist = ca.sqrt((xk[0] - ox)**2 + (xk[1] - oy)**2 + 1e-6)

                # Relaxed Safety Constraint
                self.constraints.append(
                    obstacle_exists * (dist + eps_k) + (1 - obstacle_exists) * (self.safe_distance + 1.0)
                )
                self.lbg.append(self.safe_distance)
                self.ubg.append(ca.inf)

        # Terminal Error and Cost
        theta_terminal = ca.atan2(
            self.Xref[1] - self.X[1, self.N],
            self.Xref[0] - self.X[0, self.N]
        )

        terminal_error = ca.vertcat(
            self.X[0, self.N] - self.Xref[0],
            self.X[1, self.N] - self.Xref[1],
            self.X[2, self.N] - theta_terminal
        )

        self.cost += ca.mtimes([terminal_error.T, self.Qf, terminal_error])

        # State Variable Bounds (Unconstrained position/heading)
        INF = 1e10
        for _ in range(self.N + 1):
            self.lbx.extend([-INF, -INF, -INF])
            self.ubx.extend([INF, INF, INF])

        # Control Input Bounds (v, w Limits from Table I)
        for _ in range(self.N):
            self.lbx.extend([self.v_min, self.w_min])
            self.ubx.extend([self.v_max, self.w_max])

        # Slack Variable Non-negativity Bounds (eps_i >= 0, Eq. 25)
        for _ in range(self.N):
            self.lbx.append(0.0)
            self.ubx.append(INF)

        # Nonlinear Programming Struct
        nlp = {
            "x": self.variables,
            "f": self.cost,
            "g": ca.vertcat(*self.constraints),
            "p": self.parameters
        }

        # IPOPT Solver Options
        options = {
            "ipopt.print_level": 0,
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-6,
            "ipopt.acceptable_tol": 1e-5,
            "ipopt.linear_solver": "mumps",
            "ipopt.sb": "yes",
            "print_time": False,
            "expand": True
        }

        self.solver = ca.nlpsol("solver", "ipopt", nlp, options)

    # Solve NMPC Execution Step

    def solve(self, current_state, goal_state, collision_risk, predicted_obstacles):

        obstacle_matrix = np.zeros((2, self.max_obstacles))
        number_of_obstacles = min(len(predicted_obstacles), self.max_obstacles)

        for i in range(number_of_obstacles):
            obstacle_matrix[0, i] = predicted_obstacles[i][0]
            obstacle_matrix[1, i] = predicted_obstacles[i][1]

        parameters = np.concatenate([
            np.asarray(current_state, dtype=float),
            np.asarray(goal_state, dtype=float),
            np.asarray(self.previous_control, dtype=float),
            np.array([collision_risk], dtype=float),
            obstacle_matrix.reshape(-1)
        ])

        solution = self.solver(
            x0=self.initial_guess,
            lbx=self.lbx,
            ubx=self.ubx,
            lbg=self.lbg,
            ubg=self.ubg,
            p=parameters
        )

        optimum = np.array(solution["x"]).flatten()
        self.initial_guess = optimum.copy()

        # Extract Predicted States
        number_of_states = self.nx * (self.N + 1)
        state_vector = optimum[:number_of_states]
        X_prediction = state_vector.reshape((self.N + 1, self.nx))
        self.predicted_path = X_prediction.copy()

        # Extract Predicted Controls
        number_of_controls = self.nu * self.N
        control_vector = optimum[number_of_states : number_of_states + number_of_controls]
        U_prediction = control_vector.reshape((self.N, self.nu))

        # First Receding Control Command
        control = U_prediction[0]
        linear_velocity = float(control[0])
        angular_velocity = float(control[1])

        self.previous_control = np.array([linear_velocity, angular_velocity])

        return linear_velocity, angular_velocity

    # Helper Methods

    def reset(self):
        self.initial_guess = np.zeros(self.variables.numel())
        self.previous_control = np.zeros(self.nu)
        self.predicted_path = []

    def get_predicted_path(self):
        return self.predicted_path

    def get_prediction_horizon(self):
        return self.N

    def get_sampling_time(self):
        return self.dt

    def print_solver_information(self):
        print("\n==========================================")
        print(" Risk-Aware Nonlinear MPC Solver ")
        print("==========================================")
        print(f"Horizon (Np)       : {self.N}")
        print(f"Sampling Time (Ts) : {self.dt:.3f} s")
        print(f"Safe Distance      : {self.safe_distance:.3f} m")
        print(f"Max Linear Vel     : {self.v_max:.2f} m/s")
        print(f"Max Angular Vel    : {self.w_max:.2f} rad/s")
        print(f"Risk Weight        : {self.lambda_risk:.2f}")
        print(f"Slack Penalty (rho): {self.rho_slack:.1f}")
        print("==========================================\n")

# Standalone Test Entry Point

def main():
    print("\n==============================================")
    print("     Risk-Aware NMPC Solver Standalone Test")
    print("==============================================\n")

    solver = MPCSolver(horizon=20, dt=0.10)
    solver.print_solver_information()

    current_state = np.array([0.0, 0.0, 0.0])
    goal_state = np.array([2.0, 2.0, 0.0])
    collision_risk = 0.20
    predicted_obstacles = [(1.20, 1.00), (1.50, 1.60)]

    linear_velocity, angular_velocity = solver.solve(
        current_state,
        goal_state,
        collision_risk,
        predicted_obstacles
    )

    print("==============================================")
    print("Optimal Control Output")
    print("==============================================")
    print(f"Linear Velocity  : {linear_velocity:.4f} m/s")
    print(f"Angular Velocity : {angular_velocity:.4f} rad/s")

    prediction = solver.get_predicted_path()
    if len(prediction) > 0:
        print("\n==============================================")
        print("Predicted Horizon Path (First 5 steps)")
        print("==============================================")
        for k in range(min(5, len(prediction))):
            print(f"Step {k:02d} : x={prediction[k][0]:.3f}, y={prediction[k][1]:.3f}, theta={prediction[k][2]:.3f}")


if __name__ == "__main__":
    main()