import torch
import numpy as np
from isaaclab.app import AppLauncher
app_launcher = AppLauncher({'headless': True})
app_launcher.app

from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
cfg = DHHandCatchEnvCfg()
cfg.scene.num_envs = 1
env = DHHandCatchEnv(cfg=cfg)
env.reset()

best_joints = None
best_z = -10.0
best_local_pos = None

from scipy.spatial.transform import Rotation as R

# We know shoulder_pan=0, shoulder_lift=-1.5708, elbow=1.5708
# We only need to tune wrist_1, wrist_2, wrist_3
angles = [-1.5708, 0.0, 1.5708]

print("Starting auto-tune sweep...")
for w1 in angles:
    for w2 in angles:
        for w3 in angles:
            joint_pos = env.robot.data.default_joint_pos.clone()
            joint_pos[:, 3] = w1
            joint_pos[:, 4] = w2
            joint_pos[:, 5] = w3
            
            env.robot.set_joint_position_target(joint_pos)
            env.robot.write_joint_state_to_sim(joint_pos, env.robot.data.default_joint_vel)
            env.scene.write_data_to_sim()
            env.sim.step()
            env.sim.step()
            
            palm_quat = env.robot.data.body_quat_w[0, env.palm_link_idx].cpu().numpy()
            r = R.from_quat([palm_quat[1], palm_quat[2], palm_quat[3], palm_quat[0]])
            z_axis = r.apply([0, 0, 1])
            
            palm_pos_local = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
            
            if z_axis[2] > best_z:
                best_z = z_axis[2]
                best_joints = [0.0, -1.5708, 1.5708, w1, w2, w3]
                best_local_pos = palm_pos_local

print("=== AUTO-TUNE RESULTS ===")
print("Best Z-axis upward component:", best_z)
print("Best Joints:", best_joints)
print("Best Local Pos:", best_local_pos)

env.close()
