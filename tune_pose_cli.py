import torch
import numpy as np
import threading
import queue
import sys
from isaaclab.app import AppLauncher

# Launch in Windowed mode so user can see the robot
app_launcher = AppLauncher({'headless': False})
simulation_app = app_launcher.app

from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
cfg = DHHandCatchEnvCfg()
cfg.scene.num_envs = 1
env = DHHandCatchEnv(cfg=cfg)
env.reset()

print("=====================================================")
print("Terminal Pose Tuner!")
print("Commands:")
print("  s 0 1.5  -> Sets joint 0 (shoulder_pan) to 1.5 rad")
print("  s 3 -1.5 -> Sets joint 3 (wrist_1) to -1.5 rad")
print("  p        -> Print current pose details")
print("  exit     -> Quit")
print("Joint Indices:")
print("0: shoulder_pan, 1: shoulder_lift, 2: elbow")
print("3: wrist_1,      4: wrist_2,       5: wrist_3")
print("=====================================================")

q = queue.Queue()

def input_thread():
    while True:
        try:
            cmd = input()
            q.put(cmd)
            if cmd.strip().lower() == "exit":
                break
        except Exception:
            break

t = threading.Thread(target=input_thread, daemon=True)
t.start()

# Initial joint positions
current_joints = env.robot.data.default_joint_pos[0, :6].cpu().numpy().tolist()

running = True
while running and simulation_app.is_running():
    # Process commands
    while not q.empty():
        cmd = q.get()
        cmd = cmd.strip().lower()
        if cmd == "exit":
            running = False
        elif cmd == "p":
            palm_quat = env.robot.data.body_quat_w[0, env.palm_link_idx].cpu().numpy()
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([palm_quat[1], palm_quat[2], palm_quat[3], palm_quat[0]])
            z_axis = r.apply([0, 0, 1])
            palm_pos_local = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
            print("\n--- CURRENT POSE ---")
            print(f"Joints array: [{', '.join([str(round(x, 4)) for x in current_joints])}]")
            print(f"Palm Z-axis (Aim for [0, 0, 1]): [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]")
            print(f"Palm Local Pos (Ball Drop X,Y): [{palm_pos_local[0]:.4f}, {palm_pos_local[1]:.4f}, {palm_pos_local[2]:.4f}]\n")
        elif cmd.startswith("s "):
            parts = cmd.split()
            if len(parts) == 3:
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                    if 0 <= idx <= 5:
                        current_joints[idx] = val
                        print(f"Set joint {idx} to {val}")
                    else:
                        print("Invalid joint index (0-5)")
                except ValueError:
                    print("Invalid number format")
            else:
                print("Usage: s <idx> <val>")

    if not running:
        break

    # Apply to simulation
    joint_pos = env.robot.data.default_joint_pos.clone()
    for i in range(6):
        joint_pos[:, i] = current_joints[i]
        
    env.robot.set_joint_position_target(joint_pos)
    env.robot.write_joint_state_to_sim(joint_pos, env.robot.data.default_joint_vel)
    env.scene.write_data_to_sim()
    env.sim.step()

env.close()
