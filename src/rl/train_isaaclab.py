import argparse
import sys
import os

# Ensure src module is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from isaaclab.app import AppLauncher

# Add argparse for Isaac Lab
parser = argparse.ArgumentParser("DH Hand PPO Training (Isaac Lab)")
parser.add_argument("--num_envs", type=int, default=256, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

# Launch Isaac Sim app via Isaac Lab AppLauncher
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gym
import torch

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

def main():
    print("[INFO] Setting up Isaac Lab DirectRLEnv...")
    env_cfg = DHHandCatchEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    
    # Create the environment
    env = DHHandCatchEnv(cfg=env_cfg)
    
    print("[INFO] Environment setup complete. Observation space:", env.observation_space)
    print("[INFO] Starting dummy rollout to test environment...")
    
    # Dummy training loop to verify environment steps without tying to a specific RL library version
    obs, _ = env.reset()
    for _ in range(100):
        # Generate random actions
        actions = torch.rand(env.num_envs, env.action_space.shape[0], device=env.device) * 2.0 - 1.0
        obs, rewards, dones, _, _ = env.step(actions)
    
    print("[INFO] Dummy rollout successful! Environment is fully functional.")
    
    # Close the simulator
    env.close()
    simulation_app.close()

if __name__ == '__main__':
    main()
