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
    root.title("UR5 & Ball Tuner")
    root.geometry("600x550")
    
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
        
    # Thêm slider cho quả bóng
    w_ball_x = tk.Scale(root, from_=0.0, to=1.0, resolution=0.001, orient=tk.HORIZONTAL, length=400, label="Ball X")
    w_ball_x.set(env.ball.data.root_pos_w[0, 0].item())
    w_ball_x.pack(pady=5)
    
    w_ball_y = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.001, orient=tk.HORIZONTAL, length=400, label="Ball Y")
    w_ball_y.set(env.ball.data.root_pos_w[0, 1].item())
    w_ball_y.pack(pady=5)

    def on_closing():
        global running
        running = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    running = True
    print("=====================================================")
    print("GUI has been launched successfully!")
    print("Adjust the sliders in the Tkinter window to ALIGN the Ball and Palm.")
    print("=====================================================")

    counter = 0
    while running and simulation_app.is_running():
        try:
            root.update()
        except tk.TclError:
            break # Window closed
        
        # Read sliders
        targets = [s.get() for s in sliders]
        ball_x = w_ball_x.get()
        ball_y = w_ball_y.get()
        
        # Cập nhật Robot
        joint_pos = env.robot.data.default_joint_pos.clone()
        for i in range(6):
            joint_pos[:, i] = targets[i]
        env.robot.set_joint_position_target(joint_pos)
        env.robot.write_joint_state_to_sim(joint_pos, env.robot.data.default_joint_vel)
        
        # Cập nhật Quả bóng lơ lửng
        ball_state = env.ball.data.default_root_state.clone()
        ball_state[:, 0] = ball_x
        ball_state[:, 1] = ball_y
        ball_state[:, 2] = 0.8  # Giữ cố định độ cao 0.8 để dễ ngắm
        ball_state[:, 7:] = 0.0 # Reset velocity
        env.ball.write_root_state_to_sim(ball_state)
        
        env.scene.write_data_to_sim()
        env.sim.step()
        
        if counter % 60 == 0:
            palm_pos_local = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
            print(f"\rWrist Pos: [{palm_pos_local[0]:.4f}, {palm_pos_local[1]:.4f}, {palm_pos_local[2]:.4f}] | Ball Pos: [{ball_x:.4f}, {ball_y:.4f}, 0.8000]   ", end="")
            
        counter += 1

    print("\n\n=== FINAL TUNING RESULTS ===")
    print(f"Joints array: [{', '.join([str(round(x, 4)) for x in targets])}]")
    print(f"Ball Target Pos: [{ball_x:.4f}, {ball_y:.4f}, 0.8]")

except Exception as e:
    print(f"Tkinter failed to launch: {e}")
    print("Falling back to CLI mode...")

env.close()
