import torch
import numpy as np
import threading
from isaaclab.app import AppLauncher

# Launch in Windowed mode so user can see the robot
app_launcher = AppLauncher({'headless': False})
simulation_app = app_launcher.app

from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
cfg = DHHandCatchEnvCfg()
cfg.scene.num_envs = 1 # VERY IMPORTANT: PREVENT CRASH
env = DHHandCatchEnv(cfg=cfg)
env.reset()

import tkinter as tk

try:
    root = tk.Tk()
    root.title("UR5 Pose Tuner")
    root.geometry("500x450")
    
    # Force window to stay on top
    root.attributes('-topmost', True)

    joint_names = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    ]
    default_pos = env.robot.data.default_joint_pos[0, :6].cpu().numpy()

    sliders = []
    for i, name in enumerate(joint_names):
        w = tk.Scale(root, from_=-3.14159, to=3.14159, resolution=0.01, orient=tk.HORIZONTAL, length=400, label=name)
        w.set(default_pos[i])
        w.pack(pady=5)
        sliders.append(w)

    def on_closing():
        global running
        running = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    running = True
    print("=====================================================")
    print("GUI has been launched successfully!")
    print("Adjust the sliders in the Tkinter window.")
    print("=====================================================")

    counter = 0
    while running and simulation_app.is_running():
        try:
            root.update()
        except tk.TclError:
            break # Window closed
        
        # Read sliders
        targets = [s.get() for s in sliders]
        
        joint_pos = env.robot.data.default_joint_pos.clone()
        for i in range(6):
            joint_pos[:, i] = targets[i]
            
        env.robot.set_joint_position_target(joint_pos)
        # Force the robot to snap to the target instantly for easy tuning
        env.robot.write_joint_state_to_sim(joint_pos, env.robot.data.default_joint_vel)
        env.scene.write_data_to_sim()
        env.sim.step()
        
        if counter % 60 == 0:
            palm_pos_local = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
            print(f"\rCurrent Local Palm Pos: [{palm_pos_local[0]:.4f}, {palm_pos_local[1]:.4f}, {palm_pos_local[2]:.4f}]   ", end="")
            
        counter += 1

    print("\n\n=== FINAL TUNING RESULTS ===")
    print(f"Joints array: [{', '.join([str(round(x, 4)) for x in targets])}]")
    print(f"Palm Local Pos: [{palm_pos_local[0]:.4f}, {palm_pos_local[1]:.4f}, {palm_pos_local[2]:.4f}]")

except Exception as e:
    print(f"Tkinter failed to launch: {e}")
    print("Falling back to CLI mode...")

env.close()
