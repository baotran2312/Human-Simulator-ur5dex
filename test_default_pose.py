import sys
import os
import torch
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True})
app_launcher.app

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

env_cfg = DHHandCatchEnvCfg()
env_cfg.scene.num_envs = 1
env = DHHandCatchEnv(cfg=env_cfg)

obs, _ = env.reset()
palm_link_idx = env.robot.find_bodies("DH_base_link")[0][0]

default_q = env.robot.data.default_joint_pos[0, :6]
print("Default UR5 Joints:", default_q)

palm_pos = env.robot.data.body_pos_w[:, palm_link_idx] - env.scene.env_origins
print("Default Palm Pos:", palm_pos[0].cpu().numpy())

env.close()
app_launcher.app.close()
