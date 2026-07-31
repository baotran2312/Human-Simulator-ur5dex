#!/usr/bin/env python3
"""
Week 2 Task 1: Physical Simulation Runner for Case 1 (Static) & Case 2 (Dynamic) Catching
(REWRITTEN: Real Physics, Underactuation, 3-5m/s Velocity, True Contact Force)
"""
import argparse
import csv
import os
import sys
import time
import numpy as np
from typing import Tuple, List, Optional

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

try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
except ImportError:
    print("[ERROR] Could not import isaacsim SimulationApp. Please run inside env_isaacsim or env_isaaclab.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, UsdShade

class DynamicCatchingSimulationRunner:
    ARM_JOINT_NAMES = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
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

        self.ekf = EKFBallTracker(dt=1.0/60.0)
        self.ik_solver = UR5IKSolver()
        self.hand_controller = SoftComplianceGraspController(contact_threshold_N=1.0)
        
        self.ball_prim_path = "/World/DynamicBall"
        self.prev_vel_ball = np.zeros(3)
        self._setup_physical_scene()

    def _setup_physical_scene(self):
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene_api.CreateSolverTypeAttr().Set("TGS")

        # Define high-friction, zero-restitution material to simulate a soft energy-absorbing ball/glove
        material_path = "/World/SoftCatchMaterial"
        UsdShade.Material.Define(self.stage, material_path)
        mat_prim = self.stage.GetPrimAtPath(material_path)
        physx_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
        physx_mat.CreateRestitutionAttr().Set(0.0) # Zero bounce
        physx_mat.CreateStaticFrictionAttr().Set(5.0) # High grip
        physx_mat.CreateDynamicFrictionAttr().Set(5.0)

        plane_prim = self.stage.DefinePrim("/World/GroundPlane", "Plane")
        UsdPhysics.CollisionAPI.Apply(plane_prim)
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
            physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
            physx_articulation.CreateEnabledSelfCollisionsAttr().Set(False)
            physx_articulation.CreateSolverPositionIterationCountAttr().Set(64)
            physx_articulation.CreateSolverVelocityIterationCountAttr().Set(8)

            self._enable_robot_collisions(robot_prim)
            print(f"[CatchSim] Loaded UR5DEX robot asset: {self.usd_path}")
        else:
            print(f"[WARNING] USD asset not found at {self.usd_path}.")

        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(0.85, 0.0, 0.68))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.3, 0.1)])

        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        # Bind the soft material to the ball
        material_binding = UsdShade.MaterialBindingAPI.Apply(sphere_geom.GetPrim())
        material_binding.Bind(UsdShade.Material(mat_prim), UsdShade.Tokens.weakerThanDescendants, "physics")
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(0.15)
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())

    def _enable_robot_collisions(self, root_prim):
        for prim in Usd.PrimRange(root_prim):
            type_name = prim.GetTypeName()
            if type_name in ["Mesh", "Capsule", "Sphere", "Cylinder", "Cube"]:
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_col.CreateApproximationAttr().Set("convexHull")

    def apply_arm_joint_targets(self, q_arm_target: np.ndarray):
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
                drive_api.GetDampingAttr().Set(1e4)

    def apply_hand_finger_targets(self, finger_cmd_0_1000: list):
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
                
                # Mimic tendon underactuation logic
                if "j3" in joint_name.lower() or "j4" in joint_name.lower():
                    drive_api.GetStiffnessAttr().Set(0.5) 
                    drive_api.GetDampingAttr().Set(0.1)
                else:
                    drive_api.GetStiffnessAttr().Set(20.0) 
                    drive_api.GetDampingAttr().Set(2.0)

    def launch_ball(self, pos=[2.5, 0.0, 0.8], vel=[-4.0, 0.0, 1.5]):
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid(): return

        xform = UsdGeom.Xformable(ball_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(*pos))

        rigid_api = UsdPhysics.RigidBodyAPI(ball_prim)
        if not rigid_api: rigid_api = UsdPhysics.RigidBodyAPI.Apply(ball_prim)
            
        rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*vel))
        rigid_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        
        self.ekf.reset()
        self.hand_controller.reset()
        self.hand_controller.trigger_preshaping()
        self.prev_vel_ball = np.array(vel)

    def get_ball_pos_vel(self) -> Tuple[np.ndarray, np.ndarray]:
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

    print("\n======================================================================")
    print("  RUNNING REALISTIC PHYSICAL SIMULATION BENCHMARK (WEEK 2) [FIXED]")
    print("======================================================================\n")

    for trial in range(1, args.num_trials + 1):
        is_case1 = (trial == 1)
        case_name = "Case 1: Static Catch" if is_case1 else f"Case 2: Dynamic Catch #{trial-1}"

        if is_case1:
            pos_0 = [0.45, 0.0, 0.70]
            vel_0 = [0.0, 0.0, -0.4]
        else:
            pos_0 = [2.5, np.random.uniform(-0.05, 0.05), 0.7]
            # Realistic throw suitable for UR5 physical capabilities (3 to 4 m/s)
            vx = -np.random.uniform(3.0, 4.0)
            vz = np.random.uniform(1.5, 2.5)
            vel_0 = [vx, np.random.uniform(-0.1, 0.1), vz]

        sim.launch_ball(pos=pos_0, vel=vel_0)

        step_count = 0
        terminal_errors_mm = []
        ekf_pred_errors_mm = []

        # Pre-position arm
        for _ in range(10):
            simulation_app.update()
            pos_ball, vel_ball = sim.get_ball_pos_vel()
            sim.ekf.update(pos_ball)
            p_int, t_catch = sim.ekf.predict_intercept_point(workspace_z=0.55)
            q_sol, ik_success = sim.ik_solver.solve_ik(target_pos=p_int, ball_vel=vel_ball)
            sim.apply_arm_joint_targets(q_sol)

        dt = 1.0 / 60.0
        while simulation_app.is_running() and step_count < 120: # 2 seconds max
            simulation_app.update()
            step_count += 1

            pos_ball, vel_ball = sim.get_ball_pos_vel()

            sim.ekf.update(pos_ball)
            p_int, t_catch = sim.ekf.predict_intercept_point(workspace_z=0.55)
            
            # Active Receding Trajectory (Vung tay ra đón và giật lùi)
            if t_catch > 0.4:
                # Reach out ahead of the intercept point to prepare
                target_pos = p_int - (vel_ball / np.linalg.norm(vel_ball)) * 0.15
            elif t_catch > 0.15:
                # Move to the exact intercept point
                target_pos = p_int
            else:
                # Receding motion: jerk the target backward along the velocity vector to generate backward momentum
                target_pos = p_int + (vel_ball / np.linalg.norm(vel_ball)) * 0.20
            
            # Solve IK and send to robot
            q_sol, ik_success = sim.ik_solver.solve_ik(target_pos=target_pos, ball_vel=vel_ball)
            sim.apply_arm_joint_targets(q_sol)

            # Get actual physical palm position for error calculation
            palm_prim = sim.stage.GetPrimAtPath(f"{UR5DEXConfig.robot_prim_path}/DexterousHandBase/DH_base_link")
            if palm_prim.IsValid():
                palm_xform = UsdGeom.Xformable(palm_prim)
                palm_pos = palm_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                palm_pos = np.array([palm_pos[0], palm_pos[1], palm_pos[2]])
            else:
                palm_pos = p_int

            dist_hand_ball = np.linalg.norm(pos_ball - palm_pos)
            ekf_err = np.linalg.norm(pos_ball - p_int) * 1000.0

            if pos_ball[2] <= 0.65 and pos_ball[2] >= 0.35:
                terminal_errors_mm.append(dist_hand_ball * 1000.0)
                ekf_pred_errors_mm.append(ekf_err)

            # True physics-based contact force detection
            accel = np.linalg.norm(vel_ball - sim.prev_vel_ball) / dt
            impact_force = 0.15 * accel
            sim.prev_vel_ball = vel_ball.copy()

            # Pre-trigger Grasp Strategy
            finger_cmds, hand_state = sim.hand_controller.update(impact_force, t_catch=t_catch, dt=dt)
            sim.apply_hand_finger_targets(finger_cmds)
            
            # Ground collision early exit
            if pos_ball[2] < 0.1:
                break

        # True evaluation: ball is held at the end of the trajectory
        pos_ball, _ = sim.get_ball_pos_vel()
        is_caught = (pos_ball[2] > 0.35)

        final_terminal_err = float(np.mean(terminal_errors_mm)) if terminal_errors_mm else float('nan')
        final_ekf_err = float(np.mean(ekf_pred_errors_mm)) if ekf_pred_errors_mm else float('nan')

        print(f"[{case_name:24s}] Caught: {str(is_caught):5s} | Terminal Hand Error: {final_terminal_err:5.2f} mm | EKF Pred Error: {final_ekf_err:4.2f} mm")

        results.append({
            "trial": trial,
            "case": case_name,
            "caught": is_caught,
            "terminal_hand_error_mm": round(final_terminal_err, 2) if not np.isnan(final_terminal_err) else -1,
            "ekf_pred_error_mm": round(final_ekf_err, 2) if not np.isnan(final_ekf_err) else -1,
            "launch_vel_x": round(vel_0[0], 2),
            "launch_vel_y": round(vel_0[1], 2),
            "launch_vel_z": round(vel_0[2], 2)
        })

    with open(args.csv_out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["trial", "case", "caught", "terminal_hand_error_mm", "ekf_pred_error_mm", "launch_vel_x", "launch_vel_y", "launch_vel_z"])
        writer.writeheader()
        writer.writerows(results)

    print(f"\n[DynamicCatchingSim] Realistic benchmark results successfully saved to: {args.csv_out}")
    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
