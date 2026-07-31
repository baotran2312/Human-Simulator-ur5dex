import torch
import numpy as np
from isaaclab.app import AppLauncher

# Launch in Windowed mode so user can see the robot
app_launcher = AppLauncher({'headless': False})
simulation_app = app_launcher.app

from src.rl.residual_dh_hand_catch_env import DHHandCatchEnv, DHHandCatchEnvCfg
env = DHHandCatchEnv(cfg=DHHandCatchEnvCfg())
env.reset()

import tkinter as tk

root = tk.Tk()
root.title("UR5 Pose Tuner")
root.geometry("500x500")

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
print("GUI has been launched!")
print("Adjust the sliders in the Tkinter window.")
print("The robot in Isaac Sim will move in real-time.")
print("When you are happy with the pose, close the Tkinter window.")
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
        palm_quat = env.robot.data.body_quat_w[0, env.palm_link_idx].cpu().numpy()
        from scipy.spatial.transform import Rotation as R
        r = R.from_quat([palm_quat[1], palm_quat[2], palm_quat[3], palm_quat[0]])
        z_axis = r.apply([0, 0, 1])
        palm_pos_local = env.robot.data.body_pos_w[0, env.palm_link_idx].cpu().numpy() - env.scene.env_origins[0].cpu().numpy()
        
        print("\n--- CURRENT POSE ---")
        print(f"Joints array: [{', '.join([str(round(x, 4)) for x in targets])}]")
        print(f"Palm Z-axis (Aim for [0, 0, 1]): [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]")
        print(f"Palm Local Pos (Ball Drop X,Y): [{palm_pos_local[0]:.4f}, {palm_pos_local[1]:.4f}, {palm_pos_local[2]:.4f}]")
        
    counter += 1

print("\nFinal tuning completed! Please copy the final 'Joints array' and 'Palm Local Pos' and paste them to me so I can build v11.")
env.close()
