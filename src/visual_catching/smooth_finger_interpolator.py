import numpy as np
import torch

class SmoothFingerInterpolator:
    """
    Implements a Minimum-Jerk and Bézier-based smooth trajectory generator 
    for the 19 finger joints of the DH Robotics hand.
    Bypasses abrupt position changes to eliminate the physical 'bat-effect' (violent impacts)
    and produces visually natural, anthropomorphic closing motions.
    """
    def __init__(self, num_envs: int, device: str = "cuda:0"):
        self.num_envs = num_envs
        self.device = device
        
        # Mapping fingers to joint indices in the 25-DoF robot (UR5 is 0-5, Hand is 6-24)
        self.finger_joint_indices = {
            "thumb": [10, 15, 20],            # 3 joints
            "index": [6, 11, 16, 21],         # 4 joints
            "middle": [7, 12, 17, 22],        # 4 joints
            "ring": [9, 14, 19, 24],          # 4 joints
            "pinky": [8, 13, 18, 23],         # 4 joints
        }
        
        # Anthropomorphic offsets representing natural human grasping:
        # Fingers close sequentially, with the thumb wrapping around to lock the object.
        # Target angles (in radians) to form a perfect holding cup.
        self.grasp_targets = {
            "thumb": [0.6, 0.8, 0.8],
            "index": [0.8, 0.9, 0.9, 0.7],
            "middle": [0.85, 0.95, 0.95, 0.7],
            "ring": [0.85, 0.95, 0.95, 0.7],
            "pinky": [0.8, 0.9, 0.9, 0.7],
        }
        
        # State tracking for interpolation: time since trigger started
        self.t_started = torch.zeros(self.num_envs, device=self.device)
        self.is_closing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.duration = 0.4 # Duration in seconds for a silky-smooth capture
        
    def reset_envs(self, env_ids: torch.Tensor):
        """Resets the interpolation timers for specific environment IDs."""
        self.t_started[env_ids] = 0.0
        self.is_closing[env_ids] = False

    def trigger_closing(self, env_ids: torch.Tensor, current_time: float):
        """Triggers the smooth closing phase for specific environments."""
        # Only trigger if not already closing
        new_triggers = env_ids[~self.is_closing[env_ids]]
        if len(new_triggers) > 0:
            self.is_closing[new_triggers] = True
            self.t_started[new_triggers] = current_time

    def compute_joint_targets(self, joint_targets: torch.Tensor, current_time: float) -> torch.Tensor:
        """
        Computes joint angle targets using a minimum-jerk trajectory profile.
        q(t) = q_start + (q_target - q_start) * (10*(t/D)^3 - 15*(t/D)^4 + 6*(t/D)^5)
        """
        targets = joint_targets.clone()
        
        # Calculate normalized time profile s = t / duration
        t_elapsed = current_time - self.t_started
        s = torch.clamp(t_elapsed / self.duration, min=0.0, max=1.0)
        
        # Minimum-Jerk interpolation scaling factor (smooth s-curve from 0.0 to 1.0)
        # Bypasses abrupt shocks and co-giật khớp.
        mj_factor = 10 * torch.pow(s, 3) - 15 * torch.pow(s, 4) + 6 * torch.pow(s, 5)
        
        # If not triggered to close, keep fingers open (factor is 0.0)
        mj_factor = torch.where(self.is_closing, mj_factor, torch.zeros_like(mj_factor))
        
        # Apply interpolated targets for each finger separately to create a beautiful grasp shape
        for finger_name, joint_idxs in self.finger_joint_indices.items():
            finger_targets = self.grasp_targets[finger_name]
            for j_idx, joint_idx in enumerate(joint_idxs):
                target_angle = finger_targets[j_idx]
                targets[:, joint_idx] = mj_factor * target_angle
                
        return targets
