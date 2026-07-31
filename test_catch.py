import argparse
import sys
import os
import torch
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
app_launcher.app

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

env_cfg = DHHandCatchEnvCfg()
env_cfg.scene.num_envs = 64  # Use 64 envs for smooth visual rendering
env = DHHandCatchEnv(cfg=env_cfg)

obs, _ = env.reset()
palm_link_idx = env.robot.find_bodies("DH_base_link")[0][0]

print("[INFO] Simulation is running. Press Ctrl+C in the terminal to stop.")
try:
    while app_launcher.app.is_running():
        # Apply zero action (fingers open) to observe pure physics
        actions = torch.zeros((env_cfg.scene.num_envs, 5), device=env.device)
        obs, rewards, dones, truncated, info = env.step(actions)
        
        # We don't print every step to avoid terminal spam, just let the visualizer run
except KeyboardInterrupt:
    print("[INFO] Stopping simulation...")

env.close()
app_launcher.app.close()
