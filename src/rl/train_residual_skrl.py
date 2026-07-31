import argparse
import sys
import os
import torch
import torch.nn as nn

# Ensure src module is in path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from isaaclab.app import AppLauncher

# Parse arguments for Isaac Lab
parser = argparse.ArgumentParser("DH Hand PPO Training with SKRL")
parser.add_argument("--num_envs", type=int, default=1024, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

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

def main():
    print("[INFO] Setting up Isaac Lab DirectRLEnv...")
    env_cfg = DHHandCatchEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = DHHandCatchEnv(cfg=env_cfg)
    
    print("[INFO] Wrapping environment for SKRL...")
    # SKRL natively supports Isaac Lab environments. Just use wrap_env.
    env_wrapped = wrap_env(env, wrapper="isaaclab")
    
    # Configure PPO Agent
    cfg = PPO_DEFAULT_CONFIG.copy()
    cfg["rollouts"] = 128
    cfg["learning_epochs"] = 4
    cfg["mini_batches"] = 4
    cfg["discount_factor"] = 0.99
    cfg["lambda"] = 0.95
    cfg["learning_rate"] = 1e-3
    cfg["grad_norm_clip"] = 1.0
    cfg["ratio_clip"] = 0.2
    cfg["value_clip"] = 0.2
    cfg["clip_predicted_values"] = True
    cfg["entropy_loss_scale"] = 0.0
    cfg["value_loss_scale"] = 1.0
    cfg["kl_threshold"] = 0.01
    
    # Logging Configuration
    cfg["experiment"]["directory"] = "logs/skrl/residual_dh_hand_catch"
    cfg["experiment"]["experiment_name"] = "PPO_Residual_v11"
    cfg["experiment"]["write_interval"] = 100
    cfg["experiment"]["checkpoint_interval"] = 500
    
    # Ensure device is properly configured in PPO cfg
    cfg["device"] = env.device
    
    # Instantiate Neural Networks
    models = {}
    models["policy"] = Policy(env_wrapped.observation_space, env_wrapped.action_space, env.device)
    models["value"] = Value(env_wrapped.observation_space, env_wrapped.action_space, env.device)
    
    # Create Memory
    memory = RandomMemory(memory_size=cfg["rollouts"], num_envs=env_wrapped.num_envs, device=env.device)
    
    # Create PPO Agent
    agent = PPO(models=models,
                memory=memory,
                cfg=cfg,
                observation_space=env_wrapped.observation_space,
                action_space=env_wrapped.action_space,
                device=env.device)
    
    # Create Trainer
    cfg_trainer = {"timesteps": 5000000, "headless": True}
    trainer = SequentialTrainer(cfg=cfg_trainer, env=env_wrapped, agents=agent)
    
    print("[INFO] Starting SKRL Training Loop...")
    trainer.train()
    
    env.close()
    simulation_app.close()

if __name__ == '__main__':
    main()
