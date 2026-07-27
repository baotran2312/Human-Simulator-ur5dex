#!/usr/bin/env python3
"""
Active Joint Articulation Dynamic Catching Simulation (UR5 + DH Hand)
====================================================================
This script implements full active joint manipulation in NVIDIA Isaac Sim:
1. Loads UR5DEX articulation (`ur5dex.usd`).
2. Controls 6 arm joint drives (shoulder, elbow, wrist) via ArticulationController / PhysX Drive API.
3. Controls 16 finger joint drives to execute active pre-shaping and compliance grasping.
4. Dynamically drives UR5 arm joints to intercept incoming ball in real-time.

Usage:
    /home/nhglab/anaconda3/envs/env_isaacsim/bin/python src/sim/run_articulated_catching_sim.py --headless
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

parser = argparse.ArgumentParser(description="UR5DEX Active Joint Articulation Catching Simulation")
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim in headless mode")
parser.add_argument("--usd_path", type=str, default=UR5DEXConfig.usd_asset_path, help="Path to ur5dex.usd")
parser.add_argument("--num_throws", type=int, default=3, help="Number of active throwing iterations")
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
import omni.physx as physx
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf

class ArticulatedUR5DEXSim:
    """Manages active physical joint drives and articulation control in Isaac Sim."""

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

        # Core Algorithms
        self.ekf = EKFBallTracker(dt=1.0/60.0)
        self.ik_solver = UR5IKSolver()
        self.hand_controller = SoftComplianceGraspController()

        self.ball_prim_path = "/World/DynamicBall"
        self._build_scene()

    def _build_scene(self):
        """Constructs stage with ground plane, robot asset, and physics joint drives."""
        print("[ArticulatedSim] Building physics stage with active joint drives...")
        
        # Physics Scene
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))
        physx_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_api.CreateVectorToGroundAttr().Set(Gf.Vec3f(0.0, 0.0, -9.81))

        # Ground plane
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        # Robot Articulation Asset
        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
            print(f"[ArticulatedSim] Loaded robot USD articulation asset: {self.usd_path}")
        else:
            print(f"[WARNING] USD asset not found at {self.usd_path}.")

        # Create Dynamic Ball Sphere
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(1.2, 0.0, 0.8))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.2, 0.0)])

        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(0.15)
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())

    def apply_arm_joint_targets(self, q_arm_target: np.ndarray):
        """
        Actively applies target joint positions (radians) to UR5 arm drives.
        """
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
        """
        Actively applies finger joint position targets to DH Dexterous Hand drives.
        """
        # Map 0-1000 range to joint angle limits (~0 to 1.2 rad)
        closing_ratio = finger_cmd_0_1000[1] / 1000.0  # Index finger ratio
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
        """Resets ball position and applies linear launch velocity."""
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid():
            return

        xform = UsdGeom.Xformable(ball_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(*pos))

        physx_api = PhysxSchema.PhysxRigidBodyAPI(ball_prim)
        if not physx_api:
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(ball_prim)
            
        physx_api.GetLinearVelocityAttr().Set(Gf.Vec3f(*vel))
        physx_api.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        self.ekf.reset()
        self.hand_controller.reset()
        self.hand_controller.trigger_preshaping()

    def get_ball_pose_vel(self) -> Tuple[np.ndarray, np.ndarray]:
        """Queries world 3D position and velocity of dynamic ball."""
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid():
            return np.zeros(3), np.zeros(3)

        xform = UsdGeom.Xformable(ball_prim)
        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.ExtractTranslation()
        
        physx_api = PhysxSchema.PhysxRigidBodyAPI(ball_prim)
        vel = physx_api.GetLinearVelocityAttr().Get() if physx_api else Gf.Vec3f(0, 0, 0)
        return np.array([pos[0], pos[1], pos[2]]), np.array([vel[0], vel[1], vel[2]])

def main():
    sim = ArticulatedUR5DEXSim(usd_path=args.usd_path)
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    time.sleep(0.5)

    sim.launch_ball()
    throw_cnt = 1
    step_cnt = 0

    print("\n===========================================================")
    print("  UR5DEX ACTIVE JOINT MANIPULATION SIMULATION RUNNING")
    print("===========================================================\n")

    while simulation_app.is_running() and throw_cnt <= args.num_throws:
        simulation_app.update()
        step_cnt += 1

        pos_ball, vel_ball = sim.get_ball_pose_vel()

        # 1. Trajectory Estimation (EKF)
        sim.ekf.update(pos_ball)
        p_int, t_catch = sim.ekf.predict_intercept_point(workspace_z=0.55)

        # 2. Real-Time Joint Inverse Kinematics Solution
        q_arm_target, ik_success = sim.ik_solver.solve_ik(target_pos=p_int, ball_vel=vel_ball)

        # 3. ACTIVE JOINT MANIPULATION: Apply computed target joint angles to UR5 arm physics drives!
        sim.apply_arm_joint_targets(q_arm_target)

        # 4. ACTIVE DEXTEROUS HAND MANIPULATION: Apply finger closing targets upon impact detection
        dist_to_ball = np.linalg.norm(pos_ball - p_int)
        impact_force = 5.0 if dist_to_ball < 0.05 else 0.0
        finger_cmds, hand_state = sim.hand_controller.update(impact_force, dt=1.0/60.0)
        sim.apply_hand_finger_targets(finger_cmds)

        if step_cnt % 30 == 0:
            print(f"[Step {step_cnt:04d} | Throw {throw_cnt}/{args.num_throws}] Ball Pos: ({pos_ball[0]:.2f}, {pos_ball[1]:.2f}, {pos_ball[2]:.2f}) | Arm Joint Targets (deg): {np.round(np.degrees(q_arm_target), 1)} | Hand: {hand_state}")

        # Re-throw ball every 200 simulation steps
        if step_cnt % 200 == 0:
            throw_cnt += 1
            if throw_cnt <= args.num_throws:
                vx = -1.5 + np.random.uniform(-0.2, 0.2)
                vy = np.random.uniform(-0.15, 0.15)
                vz = 0.3 + np.random.uniform(-0.1, 0.2)
                sim.launch_ball(vel=[vx, vy, vz])

    print("\n[ArticulatedSim] Active joint manipulation simulation completed successfully.")
    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
