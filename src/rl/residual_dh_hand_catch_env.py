import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg

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
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=10.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                ".*shoulder_pan_joint": 0.0,
                ".*shoulder_lift_joint": -1.25,
                ".*elbow_joint": 1.66,
                ".*wrist_1_joint": -0.43,
                "wrist_2_joint": 1.55,
                "wrist_3_joint": -3.14,
                ".*_J.*": 0.0,
                ".*thumb_j.*": 0.0,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*shoulder.*", ".*elbow.*", ".*wrist.*"],
                effort_limit=150.0,
                velocity_limit=3.14,
                stiffness=400.0,
                damping=40.0,
            ),
            "hand": ImplicitActuatorCfg(
                joint_names_expr=[".*_J.*", ".*thumb_j.*"],
                effort_limit=30.0,
                velocity_limit=10.0,
                stiffness=5.0,
                damping=0.5,
            ),
        }
    )

    # Ball Asset Setup
    ball: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/DynamicBall",
        spawn=sim_utils.SphereCfg(
            radius=0.035,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=10.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.15),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.8, 0.1)),
            physics_material=sim_utils.RigidBodyMaterialCfg(restitution=0.0, static_friction=1.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.3037, 0.1097, 0.8),
            lin_vel=(0.0, 0.0, -0.5),
        ),
    )

    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/UR5DEX/DexterousHandBase/.*",
        update_period=0.0,
        history_length=1,
        debug_vis=False,
    )

@configclass
class DHHandCatchEnvCfg(DirectRLEnvCfg):
    # Basic Env Settings
    decimation = 2
    episode_length_s = 3.0
    action_space = 5
    # ball_pos(3) + ball_vel(3) + palm_pos(3) + joint_pos(25) + joint_vel(25) + net_force(3) = 62
    observation_space = 62
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
        
        self.base_catch_pose = torch.tensor([0.0, -1.25, 1.66, -0.43, 1.55, -3.14], device=self.device, dtype=torch.float32)

    def _setup_scene(self):
        pass

    def _pre_physics_step(self, actions: torch.Tensor):
        # Cache actions to be applied in _apply_action
        self.actions = actions.clone()

    def _apply_action(self):
        # UR5 (joints 0-5) uses precomputed self.ur5_q_targets (Macro Control)
        joint_targets = self.robot.data.default_joint_pos.clone()
        joint_targets[:, :6] = self.ur5_q_targets
        
        # 1. Macro Control (Heuristic Trigger)
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        palm_dist = torch.norm(ball_pos - palm_pos, dim=-1)
        
        # Shape: (num_envs, 1)
        # Trigger distance 0.15m so fingers start closing just before impact.
        # Set target to 1.5 rad (forms a tight fist to hold the ball).
        target_finger_joint_pos = torch.where(palm_dist.unsqueeze(-1) < 0.35, 1.5, 0.0)
        
        # 2. Micro Control (DRL Impedance)
        clipped_actions = torch.clamp(self.actions, min=-1.0, max=1.0)
        delta_K = clipped_actions * 20.0
        stiffness = 50.0 + delta_K
        damping = 5.0 + 0.1 * delta_K
        
        finger_joint_indices = [
            [10, 15, 20],            # Thumb (3 joints)
            [6, 11, 16, 21],         # Index (4 joints)
            [7, 12, 17, 22],         # Middle (4 joints)
            [9, 14, 19, 24],         # Ring (4 joints)
            [8, 13, 18, 23],         # Pinky (4 joints)
        ]
        
        # Initialize full tensor for stiffness/damping for all 25 joints
        joint_stiffness = torch.full((self.num_envs, 25), 400.0, device=self.device)
        joint_damping = torch.full((self.num_envs, 25), 40.0, device=self.device)
        
        for f, joint_idxs in enumerate(finger_joint_indices):
            for j in joint_idxs:
                joint_targets[:, j] = target_finger_joint_pos[:, 0]
                joint_stiffness[:, j] = stiffness[:, f]
                joint_damping[:, j] = damping[:, f]
                
        self.robot.set_joint_position_target(joint_targets)
        self.robot.write_joint_stiffness_to_sim(joint_stiffness)
        self.robot.write_joint_damping_to_sim(joint_damping)

    def _get_observations(self) -> dict:
        # Ball state
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        ball_vel = self.ball.data.root_lin_vel_w 
        
        # Palm state
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins 
        
        # Joint state (25 joints)
        joint_pos = self.robot.data.joint_pos 
        joint_vel = self.robot.data.joint_vel
        
        # Contact force (summed over all hand links)
        # contact_forces.data.net_forces_w is (num_envs, num_sensors, 3)
        net_force = torch.sum(self.scene.sensors["contact_forces"].data.net_forces_w, dim=1) # (num_envs, 3)
        
        obs = torch.cat([
            ball_pos,        # 3
            ball_vel,        # 3
            palm_pos,        # 3
            joint_pos,       # 25
            joint_vel,       # 25
            net_force        # 3
        ], dim=-1)           # Total: 62
        
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # 1. Penalty for Contact Forces (Milestone 4 compliance)
        net_force = torch.sum(self.scene.sensors["contact_forces"].data.net_forces_w, dim=1)
        force_penalty = torch.sum(torch.square(net_force), dim=-1)
        
        # 2. Penalty for Applied Torques
        joint_efforts = self.robot.data.computed_torque
        torque_penalty = torch.sum(torch.square(joint_efforts[:, 6:25]), dim=-1)
        
        # 3. Fingertip distance to ball (Dense Reward for closing fingers)
        fingertip_indices = [10, 15, 20, 25, 30] # Link indices for thumb_Link3, index_Link4, etc (approximation)
        # Actually, let's just use palm_dist for now to keep it simple, or calculate average dist
        palm_pos = self.robot.data.body_pos_w[:, self.palm_link_idx] - self.scene.env_origins
        ball_pos = self.ball.data.root_pos_w - self.scene.env_origins
        palm_dist = torch.norm(ball_pos - palm_pos, dim=-1)
        
        # Dense reward: negative distance
        dense_reward = torch.exp(-palm_dist / 0.1)
        
        # Sparse drop penalty
        dropped = (ball_pos[:, 2] < 0.2).float()
        is_caught = (palm_dist < 0.12).float()
        
        w_f = 0.0001
        w_t = 0.0001
        
        reward = dense_reward * 2.0 + is_caught * 5.0 - w_f * force_penalty - w_t * torque_penalty - 10.0 * dropped
        
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
        default_ball_state[:, 0:3] += self.scene.env_origins[env_ids]
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
