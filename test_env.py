import sys
import os
import torch
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True})
app_launcher.app

from src.rl.dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg

env_cfg = DHHandCatchEnvCfg()
env_cfg.scene.num_envs = 1
env = DHHandCatchEnv(cfg=env_cfg)

bp = np.array([1.2, 0.0, 0.8])
bv = np.array([-1.5, 0.0, 0.3])
p_int = bp + bv * 0.3

q_curr = env.robot.data.default_joint_pos[0, :6].cpu().numpy()
q_target, _ = env.ik_solver.solve_ik(p_int, bv, q_current=q_curr)

print(f"Base q_target from IK: {q_target}")

env.close()
app_launcher.app.close()
