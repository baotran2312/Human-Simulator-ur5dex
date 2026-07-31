#!/usr/bin/env python3
"""
Soft Compliance Grasping Controller for DH Dexterous Hand (Isaac Sim Physics)
=============================================================================
Implements a 2-Phase Dynamic Catching Policy:
1. Phase 1 (Pre-shaping): Pre-opens fingers (Thumb, Index, Middle) aligned with ball trajectory.
2. Phase 2 (Soft Compliance Enclosure): Triggered upon contact force detection (F > F_thresh).
   Dynamically ramps up finger torque & position closing velocities to absorb kinetic energy
   and prevent ball bounce-out.

Usage:
    from src.sim.compliance_grasp_controller import SoftComplianceGraspController
    controller = SoftComplianceGraspController()
    finger_cmds = controller.update(contact_force=2.5, dt=0.0166)
"""

import numpy as np
from typing import List, Dict, Tuple

class SoftComplianceGraspController:
    """Soft Compliance Grasping Controller for 5-finger DH Dexterous Hand."""

    STATE_IDLE = "IDLE"
    STATE_PRESHAPING = "PRESHAPING"
    STATE_COMPLIANT_CLOSING = "COMPLIANT_CLOSING"
    STATE_LOCKED = "LOCKED"

    def __init__(self, contact_threshold_N: float = 0.8, ramp_time_sec: float = 0.05):
        self.F_thresh = contact_threshold_N
        self.ramp_time = ramp_time_sec
        self.state = self.STATE_IDLE
        
        # 5 Finger Target Positions (0 = open, 1000 = fully closed)
        self.current_pos = np.array([100, 100, 100, 0, 0], dtype=float)
        self.target_pos = np.array([850, 850, 850, 600, 600], dtype=float) # Pinch + Power grasp
        
        self.closing_time = 0.0

    def reset(self):
        """Resets controller state to idle pre-shape."""
        self.state = self.STATE_IDLE
        self.current_pos = np.array([100, 100, 100, 0, 0], dtype=float)
        self.closing_time = 0.0

    def trigger_preshaping(self, is_ball_launched: bool = True):
        # Trigger preshaping as soon as the ball is launched
        if self.state == self.STATE_IDLE and is_ball_launched:
            self.state = self.STATE_PRESHAPING
            # Basket preshape: Thumb spread out, other fingers slightly cupped
            self.current_pos = np.array([300, 400, 400, 400, 400], dtype=float)

    def update(self, max_contact_force_N: float, t_catch: float = 999.0, dt: float = 1.0 / 60.0) -> Tuple[List[int], str]:
        """
        Updates controller state based on real-time contact force and EKF time-to-contact.

        Args:
            max_contact_force_N: Maximum impact force reading from fingertip/palm sensors (Newtons)
            t_catch: Predicted time until the ball reaches the hand (seconds).
            dt: Simulation timestep in seconds

        Returns:
            finger_positions: List of 5 target integer positions [0, 1000]
            current_state: String name of active controller state
        """
        # Transition from PRESHAPING to COMPLIANT_CLOSING upon contact OR pre-trigger
        if self.state in [self.STATE_IDLE, self.STATE_PRESHAPING]:
            if max_contact_force_N >= self.F_thresh or t_catch <= 0.05:
                self.state = self.STATE_COMPLIANT_CLOSING
                self.closing_time = 0.0

        if self.state == self.STATE_COMPLIANT_CLOSING:
            self.closing_time += dt
            alpha = min(1.0, self.closing_time / self.ramp_time)
            
            # Smooth S-curve interpolation for soft kinetic energy absorption
            s_alpha = 3 * (alpha ** 2) - 2 * (alpha ** 3)
            
            open_pos = np.array([200, 200, 200, 100, 100], dtype=float)
            self.current_pos = open_pos + s_alpha * (self.target_pos - open_pos)

            if alpha >= 1.0:
                self.state = self.STATE_LOCKED

        cmd_positions = [int(p) for p in np.clip(self.current_pos, 0, 1000)]
        return cmd_positions, self.state

if __name__ == "__main__":
    controller = SoftComplianceGraspController()
    controller.trigger_preshaping()
    
    print("--- Soft Compliance Controller Test ---")
    print(f"Initial State: {controller.state}")
    
    # Simulate contact at step 3
    for step in range(10):
        force = 0.0 if step < 3 else 3.5
        cmds, state = controller.update(force, dt=0.0166)
        print(f"Step {step:02d} | Force: {force:.1f}N | State: {state:17s} | Cmds: {cmds}")
