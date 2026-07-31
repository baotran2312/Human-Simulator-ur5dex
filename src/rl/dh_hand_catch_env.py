import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, RigidObjectCfg

from isaaclab.actuators import ImplicitActuatorCfg

from src.sim.physics_config import UR5DEXConfig, DynamicBallConfig
from src.sim.curobo_ik_solver import UR5IKSolver
from src.sim.ekf_ball_tracker import EKFBallTracker

@configclass
class DHHandCatchSceneCfg(InteractiveSceneCfg):
    num_envs = 256
    env_spacing = 3.0

    # Robot Asset Setup
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/UR5DEX",
        spawn=sim_utils.UsdFileCfg(
            usd_path=UR5DEXConfig.usd_asset_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=10.0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                ".*": 0.0,  # Initialize all joints to 0.0 first
            },
        ),
        actuators={
            "all": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=400.0,
                damping=40.0,
            )
        }
    )

    # Ball Asset Setup
    ball: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/DynamicBall",
        spawn=sim_utils.SphereCfg(
            radius=DynamicBallConfig.radius,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=DynamicBallConfig.mass),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                restitution=0.0,  # Zero bounce
                static_friction=1.0, # High friction
                dynamic_friction=1.0
            )
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=DynamicBallConfig.initial_position,
            lin_vel=DynamicBallConfig.launch_velocity,
        ),
    )

@configclass
class DHHandCatchEnvCfg(DirectRLEnvCfg):
    # Basic Env Settings
    decimation = 2
    episode_length_s = 3.0
    action_space = 5
    observation_space = 41
    num_states = 0

    # Simulation Config
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=1.0 / 60.0,
        render_interval=2,
    )
    
    # Set up scene with 1024 parallel environments
    scene: DHHandCatchSceneCfg = DHHandCatchSceneCfg(num_envs=1024, env_spacing=3.0)


class DHHandCatchEnv(DirectRLEnv):
    cfg: DHHandCatchEnvCfg

    def __init__(self, cfg: DHHandCatchEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        # Tools
        self.ekf = EKFBallTracker()
        self.robot = self.scene["robot"]
        self.ball = self.scene["ball"]
        
        self.ik_solver = UR5IKSolver(use_curobo=True)
        self.ekf = EKFBallTracker()
        
        self.ur5_q_targets = torch.zeros((self.num_envs, 6), device=self.device)
        self.actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device)
        # Cache body indices to avoid massive performance bottlenecks during step()
        self.palm_link_idx = self.robot.find_bodies("DH_base_link")[0][0]
        self.fingertip_indices = [
            self.robot.find_bodies(name)[0][0]
            for name in ["thumb_Link3", "index_Link4", "middle_Link4", "ring_Link4", "pinky_Link4"]
        ]
        
        # HARDCODE base catch pose to the standard bent elbow pose.
        # This completely removes any IK solver instability or unreachable poses.
        # Palm will be at [0.487, 0.109, 0.431].
        self.base_catch_pose = torch.tensor([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], device=self.device, dtype=torch.float32)

    def _setup_scene(self):
        pass

    def _pre_physics_step(self, actions: torch.Tensor):
        # Cache actions to be applied in _apply_action
        self.actions = actions.clone()

    def _apply_action(self):
        # UR5 (joints 0-5) uses precomputed self.ur5_q_targets (Macro Control)
        # DH Hand (joints 6-24) uses RL actions to close fingers (Micro Control)
        
        joint_targets = self.robot.data.default_joint_pos.clone()
        joint_targets[:, :6] = self.ur5_q_targets
        
        # Map 5 actions to 19 hand joints accurately based on Isaac Lab joint names
        clipped_actions = torch.clamp(self.actions, min=-1.0, max=1.0)
        finger_bend = (clipped_actions + 1.0) * 0.75 
        
        finger_joint_indices = [
            [10, 15, 20],            # Thumb (3 joints)
            [6, 11, 16, 21],         # Index (4 joints)
            [7, 12, 17, 22],         # Middle (4 joints)
            [9, 14, 19, 24],         # Ring (4 joints)
            [8, 13, 18, 23],         # Pinky (4 joints)
        ]
        
        for f, joint_idxs in enumerate(finger_joint_indices):
            for j in joint_idxs:
                joint_targets[:, j] = finger_bend[:, f]
                
        self.robot.set_joint_position_target(joint_targets)

    def _get_observations(self) -> dict:
        # Ball state
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        ball_vel = self.ball.data.root_lin_vel_w 
        
        # Palm state
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins 
        palm_quat = self.robot.data.body_quat_w[:, self.palm_link_idx] 
        
        # Joint state
        joint_pos = self.robot.data.joint_pos 
        
        # Relative error
        ball_pos_error = ball_pos - palm_pos 
        
        obs = torch.cat([
            ball_pos,        # 3
            ball_vel,        # 3
            palm_pos,        # 3
            palm_quat,       # 4
            joint_pos,       # 25
            ball_pos_error   # 3
        ], dim=-1)           # Total: 41
        
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # 1. Palm distance
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        palm_dist = torch.norm(ball_pos - palm_pos, dim=-1)
        
        # 2. Fingertip distances
        ft_dists = []
        for ft_idx in self.fingertip_indices:
            ft_pos = self.robot.data.body_pos_w[:, ft_idx] - self.scene.env_origins
            ft_dists.append(torch.norm(ball_pos - ft_pos, dim=-1))
            
        avg_ft_dist = sum(ft_dists) / len(ft_dists)
        
        # 3. Relative velocity (to ensure ball is trapped/held, not just bouncing off)
        palm_vel = self.robot.data.body_lin_vel_w[:, self.palm_link_idx]
        ball_vel = self.ball.data.root_lin_vel_w
        rel_vel = torch.norm(ball_vel - palm_vel, dim=-1)
        
        # Dense reward: guide ball to palm, and fingertips to ball
        r_grasp = torch.exp(-5.0 * palm_dist) + 0.5 * torch.exp(-10.0 * avg_ft_dist)
        
        # Sparse success bonus: Massive reward if ball is trapped inside hand
        is_caught = (palm_dist < 0.12) & (rel_vel < 0.5)
        catch_bonus = is_caught.float() * 5.0
        
        # Penalty for dropping
        dropped = (ball_pos[:, 2] < 0.2).float()
        
        reward = 1.0 * r_grasp + catch_bonus - 10.0 * dropped
        
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # Died (dropped below 0.2m) or Time out
        time_out = self.episode_length_buf >= self.max_episode_length
        ball_z = self.ball.data.root_pos_w[:, 2]
        died = ball_z < 0.2
        return died, time_out

    def _reset_idx(self, env_ids: torch.Tensor):
        super()._reset_idx(env_ids)
        
        # 1. Reset Ball with small random velocity noise
        default_ball_state = self.ball.data.default_root_state[env_ids].clone()
        # Reduce noise from 1.0 to 0.1 so the ball drifts by max 2.5cm. 
        # If noise is too large, it misses the static hand completely, breaking RL credit assignment.
        random_vel = (torch.rand((len(env_ids), 3), device=self.device) - 0.5) * 0.1
        default_ball_state[:, 7:10] += random_vel
        
        self.ball.write_root_state_to_sim(default_ball_state, env_ids)
        
        # 2. Vectorized UR5 pose setting (Massive speedup over IK loop)
        # Remove noise so the ball actually hits the hand. The RL agent only controls the fingers,
        # so if the hand is offset by noise, it is physically impossible to catch the ball!
        self.ur5_q_targets[env_ids] = self.base_catch_pose
        
        # TELEPORT UR5 to the intercept pose immediately!
        # Otherwise, the PD controller will take >1 second to swing the arm there,
        # but the ball drops in 0.3 seconds, meaning the hand will NEVER be there in time!
        default_joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        default_joint_pos[:, :6] = self.ur5_q_targets[env_ids]
        
        # Keep hand open initially
        default_joint_pos[:, 6:] = 0.0
        
        self.robot.write_joint_state_to_sim(
            position=default_joint_pos, 
            velocity=torch.zeros_like(default_joint_pos), 
            env_ids=env_ids
        )
