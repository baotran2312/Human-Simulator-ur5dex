import torch
from isaaclab.app import AppLauncher
app_launcher = AppLauncher({'headless': True})
app_launcher.app
from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
env = DHHandCatchEnv(cfg=DHHandCatchEnvCfg())
env.reset()
# Let's test a few poses
print("Default pose:", env.robot.data.joint_pos[0, :6].cpu().numpy())
palm_pos = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy()
palm_quat = env.robot.data.body_quat_w[0, env.palm_link_idx].cpu().numpy()
print("Palm pos:", palm_pos)
print("Palm quat (w,x,y,z):", palm_quat)

# Calculate UP vector (Z axis of the palm frame)
from scipy.spatial.transform import Rotation as R
r = R.from_quat([palm_quat[1], palm_quat[2], palm_quat[3], palm_quat[0]])
print("Palm Z-axis (should be ~[0,0,1] to face UP):", r.apply([0, 0, 1]))
print("Palm X-axis:", r.apply([1, 0, 0]))
print("Palm Y-axis:", r.apply([0, 1, 0]))

env.close()
