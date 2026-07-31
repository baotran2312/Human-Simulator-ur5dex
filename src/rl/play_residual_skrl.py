import argparse
import sys
import os
import torch
import torch.nn as nn

# Ensure src module is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from isaaclab.app import AppLauncher

# Parse arguments for Isaac Lab
parser = argparse.ArgumentParser("DH Hand PPO Playing with SKRL")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

# Force headless to False for visualization
args_cli.headless = False

# Launch Isaac Sim app before any physics or rendering imports
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Imports must happen after AppLauncher
import skrl
from skrl.envs.wrappers.torch import wrap_env
from skrl.memories.torch import RandomMemory
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model
from skrl.trainers.torch import SequentialTrainer
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG

from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

class Policy(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions)

        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU()
        )
        self.mean_layer = nn.Linear(64, self.num_actions)
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))
        
        self.to(self.device)

    def compute(self, inputs, role=""):
        x = self.net(inputs["states"])
        return self.mean_layer(x), self.log_std_parameter, {}

class Value(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions)

        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU()
        )
        self.value_layer = nn.Linear(64, 1)
        
        self.to(self.device)

    def compute(self, inputs, role=""):
        x = self.net(inputs["states"])
        return self.value_layer(x), {}

def get_latest_checkpoint(log_dir):
    checkpoints_dir = os.path.join(log_dir, "checkpoints")
    if not os.path.exists(checkpoints_dir):
        return None
    files = [f for f in os.listdir(checkpoints_dir) if f.endswith('.pt')]
    if not files:
        return None
    files.sort(key=lambda x: os.path.getmtime(os.path.join(checkpoints_dir, x)))
    return os.path.join(checkpoints_dir, files[-1])

def main():
    print("[INFO] Setting up Isaac Lab DirectRLEnv...")
    env_cfg = DHHandCatchEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = DHHandCatchEnv(cfg=env_cfg)
    
    print("[INFO] Wrapping environment for SKRL...")
    env_wrapped = wrap_env(env, wrapper="isaaclab")
    
    # Configure PPO Agent
    cfg = PPO_DEFAULT_CONFIG.copy()
    cfg["device"] = env.device
    
    # Instantiate Neural Networks
    models = {}
    models["policy"] = Policy(env_wrapped.observation_space, env_wrapped.action_space, env.device)
    models["value"] = Value(env_wrapped.observation_space, env_wrapped.action_space, env.device)
    
    # Create Memory
    memory = RandomMemory(memory_size=16, num_envs=env_wrapped.num_envs, device=env.device)
    
    # Create PPO Agent
    agent = PPO(models=models,
                memory=memory,
                cfg=cfg,
                observation_space=env_wrapped.observation_space,
                action_space=env_wrapped.action_space,
                device=env.device)
    
    log_dir = "logs/skrl/residual_dh_hand_catch/PPO_Residual_v10"
    latest_cp = get_latest_checkpoint(log_dir)
    if latest_cp:
        print(f"[INFO] Loading checkpoint: {latest_cp}")
        agent.load(latest_cp)
    else:
        print(f"[WARNING] No checkpoint found in {log_dir}!")
        return

    agent.set_mode("eval")
    
    print("[INFO] Starting Evaluation Loop...")
    obs, _ = env_wrapped.reset()
    
    for i in range(1000):
        # Act
        actions = agent.act(obs, timestep=i, timesteps=1000)[0]
        # Step
        obs, reward, terminated, truncated, info = env_wrapped.step(actions)
        
        # In các thông số quan sát
        ball_z = env.ball.data.root_pos_w[0, 2].item()
        palm_z = env.robot.data.body_pos_w[0, env.palm_link_idx, 2].item()
        action_mean = actions[0].mean().item()
        print(f"Step {i:03d} | Reward: {reward[0].item():+.4f} | Ball Z: {ball_z:.4f} | Palm Z: {palm_z:.4f} | Action Mean: {action_mean:.4f}")
        
    env.close()
    simulation_app.close()

if __name__ == '__main__':
    main()
