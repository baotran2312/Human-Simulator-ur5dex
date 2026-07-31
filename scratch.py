import argparse
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True})
app_launcher.app

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

env_cfg = DHHandCatchEnvCfg()
env_cfg.scene.num_envs = 2
env = DHHandCatchEnv(cfg=env_cfg)

print("Robot joints:", env.robot.num_joints)
env.close()
app_launcher.app.close()
