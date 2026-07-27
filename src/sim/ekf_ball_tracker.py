#!/usr/bin/env python3
"""
Extended Kalman Filter (EKF) Ball Trajectory Predictor for Dynamic Catching
==========================================================================
This module models 3D ball dynamics under gravity and aerodynamic drag:
- State vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]^T
- Physics Process Model: \ddot{x} = - (1/2m) * rho * Cd * A * |v| * v + g
- EKF Predict & Measurement Update loop to filter noisy visual/sensor detections.
- Predicts future trajectory points, time-to-impact (T_catch), and Intercept Point (P_int).

Usage:
    from src.sim.ekf_ball_tracker import EKFBallTracker
    ekf = EKFBallTracker(dt=0.0166)
    ekf.update(np.array([1.1, 0.05, 0.78]))
    p_int, t_catch = ekf.predict_intercept_point(workspace_z=0.6)
"""

import math
import numpy as np
from typing import Tuple, Optional, List

class EKFBallTracker:
    """Extended Kalman Filter for 3D Ball Trajectory Estimation & Intercept Prediction."""

    def __init__(self, dt: float = 1.0 / 60.0, mass: float = 0.15, radius: float = 0.035, 
                 drag_coeff: float = 0.47, air_density: float = 1.225):
        self.dt = dt
        self.m = mass
        self.r = radius
        self.Cd = drag_coeff
        self.rho = air_density
        self.A = math.pi * (radius ** 2)
        self.g = np.array([0.0, 0.0, -9.81])

        # State vector: [x, y, z, vx, vy, vz]^T
        self.x = np.zeros(6)
        
        # State Covariance Matrix P
        self.P = np.eye(6) * 0.1
        
        # Process Noise Covariance Q
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2])
        
        # Measurement Noise Covariance R (3D position observation)
        self.R = np.eye(3) * 1e-3
        
        # Measurement Matrix H (observes [x, y, z])
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.initialized = False

    def reset(self):
        """Resets filter state."""
        self.x = np.zeros(6)
        self.P = np.eye(6) * 0.1
        self.initialized = False

    def _drag_accel(self, vel: np.ndarray) -> np.ndarray:
        """Computes drag acceleration vector: a_drag = - (1 / 2m) * rho * Cd * A * |v| * v"""
        v_norm = np.linalg.norm(vel)
        if v_norm < 1e-6:
            return np.zeros(3)
        k = 0.5 * self.rho * self.Cd * self.A / self.m
        return -k * v_norm * vel

    def predict(self):
        """EKF Time Update (Prediction Step)."""
        pos = self.x[0:3]
        vel = self.x[3:6]

        a_drag = self._drag_accel(vel)
        accel = self.g + a_drag

        # State Integration
        new_pos = pos + vel * self.dt + 0.5 * accel * (self.dt ** 2)
        new_vel = vel + accel * self.dt

        self.x[0:3] = new_pos
        self.x[3:6] = new_vel

        # Jacobian F_k
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * self.dt
        
        v_norm = np.linalg.norm(vel)
        if v_norm > 1e-6:
            k = 0.5 * self.rho * self.Cd * self.A / self.m
            # Derivative of drag w.r.t velocity components
            df_dv = -k * (v_norm * np.eye(3) + np.outer(vel, vel) / v_norm)
            F[3:6, 3:6] += df_dv * self.dt

        # Covariance Prediction
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_meas: np.ndarray):
        """EKF Measurement Update with 3D observed ball position."""
        if not self.initialized:
            self.x[0:3] = z_meas
            self.x[3:6] = np.zeros(3)
            self.initialized = True
            return

        # Step 1: Predict
        self.predict()

        # Step 2: Measurement Update
        y = z_meas - self.H @ self.x  # Innovation residual
        S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman Gain

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def predict_intercept_point(self, workspace_z: float = 0.6, max_horizon: float = 2.0) -> Tuple[np.ndarray, float]:
        """
        Projects future ball motion to find intersection with hand workspace height (z = workspace_z).
        
        Returns:
            P_int: (3,) numpy array [x_int, y_int, z_int]
            t_catch: estimated time in seconds until arrival
        """
        sim_x = self.x.copy()
        t = 0.0
        dt_sim = 0.005  # Fine simulation step

        while t < max_horizon:
            pos = sim_x[0:3]
            vel = sim_x[3:6]

            # Check if ball has descended to target workspace height
            if pos[2] <= workspace_z or vel[2] < -10.0:
                return pos.copy(), t

            a_drag = self._drag_accel(vel)
            accel = self.g + a_drag
            
            sim_x[0:3] += vel * dt_sim + 0.5 * accel * (dt_sim ** 2)
            sim_x[3:6] += accel * dt_sim
            t += dt_sim

        return sim_x[0:3].copy(), t

if __name__ == "__main__":
    # Test EKF Filter
    tracker = EKFBallTracker(dt=0.0166)
    
    # Simulated parabolic arc with noise
    pos_true = np.array([1.2, 0.0, 0.9])
    vel_true = np.array([-1.8, 0.1, 0.4])

    print("--- EKF Ball Tracker Test ---")
    for i in range(10):
        pos_noisy = pos_true + np.random.normal(0, 0.005, size=3)
        tracker.update(pos_noisy)
        pos_est = tracker.x[0:3]
        vel_est = tracker.x[3:6]
        
        # Advance ground truth
        pos_true += vel_true * 0.0166 + 0.5 * np.array([0, 0, -9.81]) * (0.0166**2)
        vel_true += np.array([0, 0, -9.81]) * 0.0166

    p_int, t_catch = tracker.predict_intercept_point(workspace_z=0.5)
    print(f"Estimated Intercept Point P_int: {np.round(p_int, 3)}")
    print(f"Estimated Time-to-Impact T_catch: {t_catch:.3f} seconds")
