import numpy as np
import torch
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.types import ArticulationAction
from src.sim.curobo_ik_solver import UR5IKSolver
from src.sim.ekf_ball_tracker import EKFBallTracker

from omni.isaac.core.utils.prims import create_prim, define_prim
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.cloner import GridCloner
from src.sim.physics_config import UR5DEXConfig, DynamicBallConfig

class DHHandCatchTask(BaseTask):
    """
    Hierarchical RL Task for Dynamic Catching (Macro-Micro Framework).
    - UR5 Arm (Macro): Controlled via EKF + cuRobo IK (Analytical/Heuristic).
    - DH Hand (Micro): Controlled via RL Policy (PPO) for impedance/stiffness adaptation.
    """
    def __init__(self, name="DHHandCatchTask", num_envs=256, env_spacing=3.0):
        super().__init__(name=name, offset=None)
        
        self._num_envs = num_envs
        self._env_spacing = env_spacing
        self._cloner = GridCloner(spacing=self._env_spacing)
        self._cloner.define_base_env("/World/Envs")
        define_prim("/World/Envs/Env_0")
        
        # RL Spaces based on Manuscript Section III.A
        self.observation_space = 41
        self.action_space = 5
        
        # Tracking & IK
        self.ekf = EKFBallTracker()
        self.ik_solver = UR5IKSolver(use_curobo=True)
        
        # Hand Base Parameters
        self.base_stiffness = 50.0
        self.base_damping = 5.0

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        
        # 1. Spawn Robot in the base environment
        create_prim(
            prim_path="/World/Envs/Env_0/UR5DEX",
            prim_type="Xform",
            position=np.array([0.0, 0.0, 0.0]),
            usd_path=UR5DEXConfig.usd_asset_path,
        )
        
        # 2. Spawn Ball in the base environment
        DynamicSphere(
            prim_path="/World/Envs/Env_0/DynamicBall",
            name="ball_0",
            position=np.array(DynamicBallConfig.initial_position),
            radius=DynamicBallConfig.radius,
            mass=DynamicBallConfig.mass,
            color=np.array([1.0, 0.2, 0.2]),
        )
        
        # 3. Clone Environments
        prim_paths = self._cloner.generate_paths("/World/Envs/Env", self._num_envs)
        self._cloner.clone(source_prim_path="/World/Envs/Env_0", prim_paths=prim_paths)
        
        # 4. Create Tensor Views for RL
        self.robots = ArticulationView(prim_paths_expr="/World/Envs/.*/UR5DEX", name="ur5dex_view")
        self.balls = RigidPrimView(prim_paths_expr="/World/Envs/.*/DynamicBall", name="ball_view")
        
        scene.add(self.robots)
        scene.add(self.balls)

    def get_observations(self) -> dict:
        """
        Extracts state vector s_t for the PPO agent.
        """
        # Fetch data from simulation (pseudo-code for structure)
        # finger_pos = self.hand.get_joint_positions()
        # finger_vel = self.hand.get_joint_velocities()
        # ball_pos, ball_vel = self.ball.get_state()
        # f_ext = self.hand.get_measured_contact_forces()
        
        # For demonstration of the API structure:
        obs = torch.zeros((self._num_envs, self.observation_space), device="cuda:0")
        
        # obs[:, 0:15] = finger_pos
        # obs[:, 15:30] = finger_vel
        # obs[:, 30:33] = ball_pos
        # obs[:, 33:36] = ball_vel
        # obs[:, 36:41] = f_ext
        
        return {"obs": obs}

    def pre_physics_step(self, actions: torch.Tensor):
        """
        Applies hierarchical control: 
        1. EKF + IK for the Arm.
        2. RL Action for the Hand Stiffness.
        """
        # 1. Macro Control (UR5 Arm) - Identical to offline script
        # ball_pos = get_ball_pos()
        # ball_vel = get_ball_vel()
        # self.ekf.update(ball_pos)
        # p_int, t_catch = self.ekf.predict_intercept_point()
        
        # q_target, success = self.ik_solver.solve_ik(p_int, ball_vel)
        # self.ur5_arm.apply_action(ArticulationAction(joint_positions=q_target))
        
        # 2. Micro Control (DH Hand DRL Adapter)
        # action scales delta K_a from [-1, 1] to physical stiffness limits e.g., [-20, 20]
        delta_k = actions.cpu().numpy() * 20.0
        
        new_stiffness = self.base_stiffness + delta_k
        
        # Apply modulated stiffness to hand joints (Impedance Control)
        # self.hand.set_stiffness(new_stiffness)
        # self.hand.apply_action(ArticulationAction(joint_positions=target_grasp_pos))

    def calculate_metrics(self) -> torch.Tensor:
        """
        Reward Function from Eq (17):
        r_t = w1 * r_grasp - w2 * ||f_ext||^2 - w3 * ||tau||^2
        """
        w1, w2, w3 = 100.0, 0.1, 0.01
        
        # Check grasp condition (ball inside hand bounding box and moving with hand)
        # r_grasp = check_grasp_success()
        r_grasp = torch.zeros(self._num_envs, device="cuda:0")
        
        # Contact forces and torques
        # f_ext = self.hand.get_measured_contact_forces()
        # tau = self.hand.get_applied_torques()
        f_ext = torch.zeros(self._num_envs, device="cuda:0")
        tau = torch.zeros(self._num_envs, device="cuda:0")
        
        reward = w1 * r_grasp - w2 * torch.norm(f_ext, dim=-1) - w3 * torch.norm(tau, dim=-1)
        return reward

    def is_done(self) -> torch.Tensor:
        """
        Terminal conditions:
        - Ball drops below Z=0.2 (Missed)
        - Catch time exceeded without contact
        """
        # ball_pos = self.ball.get_world_poses()[0]
        # dropped = ball_pos[:, 2] < 0.2
        dones = torch.zeros(self._num_envs, dtype=torch.bool, device="cuda:0")
        return dones
