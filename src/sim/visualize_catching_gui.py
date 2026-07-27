#!/usr/bin/env python3
"""
Interactive GUI Visualizer for UR5DEX Dynamic Ball Catching (Isaac Sim)
======================================================================
This visualizer opens NVIDIA Isaac Sim's interactive viewport window and displays:
1. Real-Time 3D Trajectory Markers:
   - Green Spheres: Predicted EKF trajectory path of the incoming ball.
   - Cyan Sphere: Intercept Point P_int where the hand palm is positioned.
   - Red Sphere: Physical dynamic ball thrown with force F.
2. Active UR5 Arm & DH Dexterous Hand Joint Motion in 3D Viewport.
3. Live Camera View centered on the robot workspace.

Usage:
    DISPLAY=:0 /home/nhglab/anaconda3/envs/env_isaacsim/bin/python src/sim/visualize_catching_gui.py
"""

import argparse
import os
import sys
import time
import numpy as np

# Add src root to Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from src.sim.ekf_ball_tracker import EKFBallTracker
from src.sim.curobo_ik_solver import UR5IKSolver
from src.sim.compliance_grasp_controller import SoftComplianceGraspController
from src.sim.physics_config import UR5DEXConfig

parser = argparse.ArgumentParser(description="UR5DEX Isaac Sim GUI Visualizer")
parser.add_argument("--usd_path", type=str, default=UR5DEXConfig.usd_asset_path, help="Path to ur5dex.usd")
args, _ = parser.parse_known_args()

# --- Launch Isaac Sim in GUI mode ---
try:
    from isaacsim import SimulationApp
    # Headless is set to False so the full Isaac Sim viewport opens on screen
    simulation_app = SimulationApp({"headless": False})
except ImportError:
    print("[ERROR] Could not import isaacsim SimulationApp. Please run inside env_isaacsim or env_isaaclab.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.timeline
import omni.physx as physx
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf

class IsaacSimGUIVisualizer:
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

        self.ekf = EKFBallTracker(dt=1.0/60.0)
        self.ik_solver = UR5IKSolver()
        self.hand_controller = SoftComplianceGraspController()

        self.ball_prim_path = "/World/DynamicBall"
        self.intercept_marker_path = "/World/VisualMarkers/InterceptPoint"
        
        self._setup_visual_stage()

    def _setup_visual_stage(self):
        """Builds visual viewport stage with camera, lighting, and trajectory markers."""
        print("[GUIVisualizer] Setting up Isaac Sim interactive GUI viewport...")

        # Physics Scene
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))

        # Ground Plane
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        # Load Robot Asset
        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
            self._enable_robot_collisions(robot_prim)
            print(f"[GUIVisualizer] Loaded robot USD and enabled link physics collisions: {self.usd_path}")

    def _enable_robot_collisions(self, root_prim):
        """Enforces UsdPhysics & PhysX collision APIs on all UR5 and DH Hand link meshes."""
        for prim in Usd.PrimRange(root_prim):
            type_name = prim.GetTypeName()
            if type_name in ["Mesh", "Capsule", "Sphere", "Cylinder", "Cube"]:
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_col.CreateApproximationAttr().Set("convexHull")

        # Dynamic Ball Prim (Bright Red/Orange)
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(1.2, 0.0, 0.8))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.2, 0.0)])

        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(0.15)
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())

        # Intercept Point Visual Marker Prim (Bright Cyan)
        marker_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.intercept_marker_path))
        marker_geom.GetRadiusAttr().Set(0.025)
        marker_geom.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.0, 0.55))
        marker_geom.GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.9, 1.0)])

        # Set Viewport Camera
        try:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.set_active_camera("/OmniverseKit_Persp")
        except Exception:
            pass

    def update_intercept_marker(self, pos: np.ndarray):
        """Updates position of the visual cyan intercept target marker."""
        prim = self.stage.GetPrimAtPath(self.intercept_marker_path)
        if prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))

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

    def launch_ball(self, pos=[1.2, 0.0, 0.8], vel=[-1.5, 0.0, 0.3]):
        """Launches dynamic ball."""
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

    def get_ball_pose_vel(self) -> tuple:
        """Queries 3D ball position and velocity."""
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
    viz = IsaacSimGUIVisualizer(usd_path=args.usd_path)
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    time.sleep(0.5)

    viz.launch_ball()
    throw_cnt = 1
    step_cnt = 0

    print("\n==================================================================")
    print("  ISAAC SIM INTERACTIVE GUI VISUALIZATION RUNNING")
    print("  - Red Sphere: Physical Dynamic Ball")
    print("  - Cyan Marker: Predicted Intercept Point P_int")
    print("  - UR5 Arm & DH Hand: Active Articulated Joint Tracking & Grasping")
    print("==================================================================\n")

    while simulation_app.is_running():
        simulation_app.update()
        step_cnt += 1

        pos_ball, vel_ball = viz.get_ball_pose_vel()

        # 1. Trajectory Prediction (EKF)
        viz.ekf.update(pos_ball)
        p_int, t_catch = viz.ekf.predict_intercept_point(workspace_z=0.55)

        # 2. Update Visual Intercept Marker (Cyan Sphere)
        viz.update_intercept_marker(p_int)

        # 3. Real-Time Joint Inverse Kinematics Solution
        q_arm_target, ik_success = viz.ik_solver.solve_ik(target_pos=p_int, ball_vel=vel_ball)

        # 4. Apply Active Arm Joint Targets
        viz.apply_arm_joint_targets(q_arm_target)

        # 5. Apply Active Hand Finger Closing Targets
        dist_to_ball = np.linalg.norm(pos_ball - p_int)
        impact_force = 5.0 if dist_to_ball < 0.05 else 0.0
        finger_cmds, hand_state = viz.hand_controller.update(impact_force, dt=1.0/60.0)
        viz.apply_hand_finger_targets(finger_cmds)

        if step_cnt % 60 == 0:
            print(f"[Step {step_cnt:04d} | Throw #{throw_cnt}] Ball Pos: ({pos_ball[0]:.2f}, {pos_ball[1]:.2f}, {pos_ball[2]:.2f}) | Intercept P_int: ({p_int[0]:.2f}, {p_int[1]:.2f}, {p_int[2]:.2f}) | Hand: {hand_state}")

        # Continuous automatic ball re-throw loop every 240 frames
        if step_cnt % 240 == 0:
            throw_cnt += 1
            vx = -1.5 + np.random.uniform(-0.25, 0.25)
            vy = np.random.uniform(-0.15, 0.15)
            vz = 0.35 + np.random.uniform(-0.1, 0.2)
            viz.launch_ball(vel=[vx, vy, vz])

    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
