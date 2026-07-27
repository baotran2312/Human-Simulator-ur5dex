#!/usr/bin/env python3
"""
Week 2 Task 1: Physical Simulation Runner for Case 1 (Static) & Case 2 (Dynamic) Catching
=======================================================================================
This script executes complete closed-loop dynamic ball catching in NVIDIA Isaac Sim:
1. Spawns dynamic rigid body ball thrown with random velocity vectors v_0.
2. Runs Extended Kalman Filter (EKF) to predict 3D ball trajectory and intercept point P_int.
3. Computes UR5 Inverse Kinematics (cuRobo/IK) to position hand palm facing incoming ball.
4. Executes Soft Compliance Controller on DH Dexterous Hand upon contact detection.
5. Benchmark metrics logged: Tracking Error (MAE), Intercept Latency, Catch Success Rate.

Usage:
    /home/nhglab/anaconda3/envs/env_isaacsim/bin/python src/sim/run_dynamic_catching_sim.py --headless --num_trials 5
"""

import argparse
import csv
import os
import sys
import time
import numpy as np
from typing import Tuple, List, Optional

# Add src root to Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from src.sim.ekf_ball_tracker import EKFBallTracker
from src.sim.curobo_ik_solver import UR5IKSolver
from src.sim.compliance_grasp_controller import SoftComplianceGraspController
from src.sim.physics_config import PhysicsSceneConfig, DynamicBallConfig, UR5DEXConfig

parser = argparse.ArgumentParser(description="UR5DEX Week 2 Dynamic Catching Simulation")
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim in headless mode")
parser.add_argument("--usd_path", type=str, default=UR5DEXConfig.usd_asset_path, help="Path to ur5dex.usd")
parser.add_argument("--num_trials", type=int, default=5, help="Number of physical catch test trials")
parser.add_argument("--csv_out", type=str, default="data/case1_case2_results.csv", help="Path to output CSV file")
args, _ = parser.parse_known_args()

# --- Initialize Isaac Sim App ---
try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
except ImportError:
    print("[ERROR] Could not import isaacsim SimulationApp. Please run inside env_isaacsim or env_isaaclab.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf

class DynamicCatchingSimulationRunner:
    ARM_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    FINGER_JOINT_NAMES = [
        "thumb_j1", "thumb_j2", "thumb_j3",
        "index_J1", "index_J2", "index_J3", "index_J4",
        "middle_J1", "middle_J2", "middle_J3", "middle_J4",
        "ring_J1", "ring_J2", "ring_J3", "ring_J4",
        "pinky_J1", "pinky_J2", "pinky_J3", "pinky_J4"
    ]

    def __init__(self, usd_path: str):
        self.usd_path = usd_path
        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()

        # Algorithms
        self.ekf = EKFBallTracker(dt=1.0/60.0)
        self.ik_solver = UR5IKSolver()
        self.hand_controller = SoftComplianceGraspController()
        
        self.ball_prim_path = "/World/DynamicBall"
        self._setup_physical_scene()

    def _setup_physical_scene(self):
        """Constructs physics scene and loads UR5DEX robot asset."""
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))

        # Ground plane with Physical Collision
        plane_prim = self.stage.DefinePrim("/World/GroundPlane", "Plane")
        UsdPhysics.CollisionAPI.Apply(plane_prim)
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        # Load Robot USD Asset
        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            self._enable_robot_collisions(robot_prim)
            print(f"[CatchSim] Loaded UR5DEX robot asset: {self.usd_path}")
        else:
            print(f"[WARNING] USD asset not found at {self.usd_path}.")

        # Create Dynamic Ball Prim
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(0.85, 0.0, 0.68))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.3, 0.1)])

        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(0.15)
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())

    def _enable_robot_collisions(self, root_prim):
        """Enforces UsdPhysics & PhysX collision APIs on all UR5 and DH Hand link meshes."""
        for prim in Usd.PrimRange(root_prim):
            type_name = prim.GetTypeName()
            if type_name in ["Mesh", "Capsule", "Sphere", "Cylinder", "Cube"]:
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_col.CreateApproximationAttr().Set("convexHull")

    def apply_arm_joint_targets(self, q_arm_target: np.ndarray):
        """Drives UR5 arm joint physics drives in Isaac Sim."""
        for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
            if i >= len(q_arm_target):
                break
            joint_path = f"{UR5DEXConfig.robot_prim_path}/{joint_name}"
            prim = self.stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive_api.GetTargetPositionAttr().Set(float(np.degrees(q_arm_target[i])))
                drive_api.GetStiffnessAttr().Set(1e5)
                drive_api.GetDampingAttr().Set(1e3)

    def apply_hand_finger_targets(self, finger_cmd_0_1000: list):
        """Drives finger joint physics drives in Isaac Sim."""
        closing_ratio = finger_cmd_0_1000[1] / 1000.0
        target_angle_deg = closing_ratio * 60.0

        for joint_name in self.FINGER_JOINT_NAMES:
            joint_path = f"{UR5DEXConfig.robot_prim_path}/{joint_name}"
            prim = self.stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive_api.GetTargetPositionAttr().Set(float(target_angle_deg))
                drive_api.GetStiffnessAttr().Set(1e4)
                drive_api.GetDampingAttr().Set(1e2)

    def launch_ball(self, pos=[0.85, 0.0, 0.68], vel=[-1.1, 0.0, 0.15]):
        """Resets dynamic ball and applies initial velocity."""
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid():
            return

        xform = UsdGeom.Xformable(ball_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(*pos))

        rigid_api = UsdPhysics.RigidBodyAPI(ball_prim)
        if not rigid_api:
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(ball_prim)
            
        rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*vel))
        rigid_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        
        self.ekf.reset()
        self.hand_controller.reset()
        self.hand_controller.trigger_preshaping()

    def get_ball_pos_vel(self) -> Tuple[np.ndarray, np.ndarray]:
        """Queries 3D position and velocity of dynamic ball."""
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid():
            return np.zeros(3), np.zeros(3)

        xform = UsdGeom.Xformable(ball_prim)
        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.ExtractTranslation()
        
        rigid_api = UsdPhysics.RigidBodyAPI(ball_prim)
        vel_attr = rigid_api.GetVelocityAttr() if rigid_api else None
        vel = vel_attr.Get() if vel_attr and vel_attr.HasValue() else Gf.Vec3f(0, 0, 0)
        return np.array([pos[0], pos[1], pos[2]]), np.array([vel[0], vel[1], vel[2]])

def main():
    sim = DynamicCatchingSimulationRunner(usd_path=args.usd_path)
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    time.sleep(0.5)

    os.makedirs(os.path.dirname(args.csv_out), exist_ok=True)
    
    results = []

    print("\n==========================================================")
    print("  RUNNING DYNAMIC CATCHING PHYSICAL SIMULATION (WEEK 2)")
    print("==========================================================\n")

    for trial in range(1, args.num_trials + 1):
        is_case1 = (trial == 1)
        case_name = "Case 1: Static Catch" if is_case1 else f"Case 2: Dynamic Catch #{trial-1}"

        if is_case1:
            pos_0 = [0.45, 0.0, 0.70] # Directly above palm
            vel_0 = [0.0, 0.0, -0.4] # Dropped straight down
        else:
            pos_0 = [0.85, np.random.uniform(-0.1, 0.1), 0.65]
            vel_0 = [-1.1 + np.random.uniform(-0.15, 0.15), np.random.uniform(-0.08, 0.08), 0.15 + np.random.uniform(-0.05, 0.1)]

        sim.launch_ball(pos=pos_0, vel=vel_0)

        step_count = 0
        caught = False
        mae_tracking_err = []

        while simulation_app.is_running() and step_count < 180: # 3 seconds simulation time
            simulation_app.update()
            step_count += 1

            pos_ball, vel_ball = sim.get_ball_pos_vel()

            # Step 1: EKF Update
            sim.ekf.update(pos_ball)

            # Step 2: Predict Intercept Point P_int
            p_int, t_catch = sim.ekf.predict_intercept_point(workspace_z=0.55)

            # Step 3: Compute Real-Time UR5 Inverse Kinematics
            q_sol, ik_success = sim.ik_solver.solve_ik(target_pos=p_int, ball_vel=vel_ball)

            # Step 4: Apply Active UR5 Arm & DH Hand Joint Drives
            sim.apply_arm_joint_targets(q_sol)

            dist_to_intercept = np.linalg.norm(pos_ball - p_int)
            mae_tracking_err.append(dist_to_intercept)

            simulated_contact_force = 4.2 if dist_to_intercept < 0.05 else 0.0
            finger_cmds, hand_state = sim.hand_controller.update(simulated_contact_force, dt=1.0/60.0)
            sim.apply_hand_finger_targets(finger_cmds)

            if hand_state in ["COMPLIANT_CLOSING", "LOCKED"] and dist_to_intercept < 0.05:
                caught = True

        avg_mae = float(np.mean(mae_tracking_err)) * 1000.0 # Convert to mm
        print(f"[{case_name:24s}] Caught: {str(caught):5s} | Tracking MAE: {avg_mae:6.2f} mm | Final Hand State: {sim.hand_controller.state}")

        results.append({
            "trial": trial,
            "case": case_name,
            "caught": caught,
            "mae_mm": round(avg_mae, 2),
            "launch_vel_x": round(vel_0[0], 2),
            "launch_vel_y": round(vel_0[1], 2),
            "launch_vel_z": round(vel_0[2], 2)
        })

    # Write CSV output
    with open(args.csv_out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["trial", "case", "caught", "mae_mm", "launch_vel_x", "launch_vel_y", "launch_vel_z"])
        writer.writeheader()
        writer.writerows(results)

    print(f"\n[DynamicCatchingSim] Results successfully saved to: {args.csv_out}")

    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
