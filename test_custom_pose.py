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

test_q = torch.zeros((1, 25), device=env.device)
test_q[0, :6] = torch.tensor([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], device=env.device)

env.robot.write_joint_state_to_sim(
    position=test_q, 
    velocity=torch.zeros_like(test_q), 
    env_ids=torch.tensor([0], device=env.device)
)
env.scene.write_data_to_sim()
env.scene.update(0.0)

palm_pos = env.robot.data.body_pos_w[:, palm_link_idx] - env.scene.env_origins
print("Custom Palm Pos:", palm_pos[0].cpu().numpy())

env.close()
app_launcher.app.close()
