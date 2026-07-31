import torch
from isaaclab.app import AppLauncher
app_launcher = AppLauncher({'headless': True})
app_launcher.app
from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
env = DHHandCatchEnv(cfg=DHHandCatchEnvCfg())
env.reset()

# Set joint pos
joint_pos = env.robot.data.default_joint_pos.clone()
joint_pos[:, 3] = 1.5708  # wrist_1_joint
joint_pos[:, 4] = 0.0     # wrist_2_joint
joint_pos[:, 5] = 0.0     # wrist_3_joint
env.robot.set_joint_position_target(joint_pos)
env.robot.write_joint_state_to_sim(joint_pos, env.robot.data.default_joint_vel)
env.scene.write_data_to_sim()
env.sim.step()
env.sim.step()

palm_pos_w = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy()
palm_pos_local = palm_pos_w - env.scene.env_origins[0].cpu().numpy()
palm_quat = env.robot.data.body_quat_w[0, env.palm_link_idx].cpu().numpy()

from scipy.spatial.transform import Rotation as R
r = R.from_quat([palm_quat[1], palm_quat[2], palm_quat[3], palm_quat[0]])
print("New Palm Z-axis:", r.apply([0, 0, 1]))
print("New Palm local pos:", palm_pos_local)

env.close()
