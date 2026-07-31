import time
import sys
import os
import torch
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True})
app_launcher.app

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

env_cfg = DHHandCatchEnvCfg()
env_cfg.scene.num_envs = 1024
env = DHHandCatchEnv(cfg=env_cfg)

obs, _ = env.reset()

print("Starting timing...")
start_time = time.time()
for i in range(100):
    actions = torch.zeros((1024, 5), device=env.device)
    env.step(actions)
end_time = time.time()

print(f"100 steps took {end_time - start_time:.4f} seconds.")
print(f"FPS: {100 / (end_time - start_time):.2f} env steps / sec")
print(f"Timesteps/sec: {100 * 1024 / (end_time - start_time):.2f}")

env.close()
app_launcher.app.close()
