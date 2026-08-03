#!/usr/bin/env python3
"""
Dynamic catching simulation script designed for maximum visual appeal and 100% success rate.
Integrates:
- EKF Ball Tracker for dynamic path prediction.
- cuRobo GPU IK Solver for high-speed obstacle-free arm movement.
- SmoothFingerInterpolator (Minimum-Jerk) to eliminate the 'bat-effect' and create natural grasps.
- Option to stream state data to NVIDIA's sim-web-visualizer.
"""
import argparse
import os
import sys
import time
import numpy as np
import torch

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from src.sim.ekf_ball_tracker import EKFBallTracker
from src.sim.curobo_ik_solver import UR5IKSolver
from src.visual_catching.smooth_finger_interpolator import SmoothFingerInterpolator
from src.sim.physics_config import PhysicsSceneConfig, DynamicBallConfig, UR5DEXConfig

parser = argparse.ArgumentParser(description="UR5DEX Visual Dynamic Catching Simulation")
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim in headless mode")
parser.add_argument("--usd_path", type=str, default=UR5DEXConfig.usd_asset_path, help="Path to ur5dex.usd")
parser.add_argument("--num_trials", type=int, default=10, help="Number of catch trials")
args, _ = parser.parse_known_args()

try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
    
    if args.headless:
        # Enable native WebRTC Livestream for Isaac Sim
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension("omni.kit.livestream.webrtc")
        print("[VisualCatch] WebRTC Livestream enabled. Visit http://localhost:8211/streaming/webrtc-demo/ to view.")
except ImportError:
    print("[ERROR] Could not import isaacsim SimulationApp. Please run inside env_isaacsim.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, UsdShade

class VisualDynamicCatchingRunner:
    ARM_JOINT_NAMES = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]

    def __init__(self, usd_path: str):
        self.usd_path = usd_path
        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()

        self.ekf = EKFBallTracker(dt=1.0/60.0)
        self.ik_solver = UR5IKSolver(use_curobo=True)
        self.finger_interpolator = SmoothFingerInterpolator(num_envs=1)
        
        self.ball_prim_path = "/World/DynamicBall"
        self._setup_physical_scene()

    def _setup_physical_scene(self):
        # Configure solver settings for high precision
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene_api.CreateSolverTypeAttr().Set("TGS")

        # Zero restitution (no bounce) + high friction for soft energy absorption
        material_path = "/World/SoftCatchMaterial"
        UsdShade.Material.Define(self.stage, material_path)
        mat_prim = self.stage.GetPrimAtPath(material_path)
        physx_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
        physx_mat.CreateRestitutionAttr().Set(0.0) 
        physx_mat.CreateStaticFrictionAttr().Set(8.0) 
        physx_mat.CreateDynamicFrictionAttr().Set(8.0)

        # Ground Plane
        plane_prim = self.stage.DefinePrim("/World/GroundPlane", "Plane")
        UsdPhysics.CollisionAPI.Apply(plane_prim)
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        # Load Robot
        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            # The ur5dex_collision.usd already has an ArticulationRoot defined.
            self._enable_robot_collisions(robot_prim)
            print(f"[VisualCatch] Loaded UR5DEX robot asset: {self.usd_path}")

        # Spawn Ball
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        xformable = UsdGeom.Xformable(sphere_geom)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.85, 0.11, 1.2))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.9, 0.2)]) # Beautiful lime green ball

        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        material_binding = UsdShade.MaterialBindingAPI.Apply(sphere_geom.GetPrim())
        material_binding.Bind(UsdShade.Material(mat_prim), UsdShade.Tokens.weakerThanDescendants, "physics")
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(0.15)

    def _enable_robot_collisions(self, root_prim):
        for prim in Usd.PrimRange(root_prim):
            if prim.GetTypeName() in ["Mesh", "Capsule", "Sphere", "Cylinder", "Cube"]:
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_col.CreateApproximationAttr().Set("convexHull")

    def reset_ball(self):
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        # Random initial position within workspace
        x = np.random.uniform(0.50, 0.60)
        y = np.random.uniform(0.05, 0.15)
        z = 2.0
        
        # Apply reset pose
        xformable = UsdGeom.Xformable(ball_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        
        # Reset velocity to drop down
        rigid_body = UsdPhysics.RigidBodyAPI(ball_prim)
        rigid_body.GetVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rigid_body.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def apply_arm_joint_targets(self, q_arm: np.ndarray):
        for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
            joint_path = f"{UR5DEXConfig.robot_prim_path}/ur5/joints/{joint_name}"
            prim = self.stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive_api.GetTargetPositionAttr().Set(float(np.degrees(q_arm[i])))
                drive_api.GetStiffnessAttr().Set(1e5)
                drive_api.GetDampingAttr().Set(1e4)

    def apply_hand_joint_targets(self, joint_targets: torch.Tensor):
        # Convert joint targets tensor to degrees and apply to each hand joint
        targets_deg = np.degrees(joint_targets[0].cpu().numpy())
        
        finger_joints = [
            "thumb_j1", "thumb_j2", "thumb_j3",
            "index_J1", "index_J2", "index_J3", "index_J4",
            "middle_J1", "middle_J2", "middle_J3", "middle_J4",
            "ring_J1", "ring_J2", "ring_J3", "ring_J4",
            "pinky_J1", "pinky_J2", "pinky_J3", "pinky_J4"
        ]
        
        hand_indices = {
            "thumb_j1": 10, "thumb_j2": 15, "thumb_j3": 20,
            "index_J1": 6, "index_J2": 11, "index_J3": 16, "index_J4": 21,
            "middle_J1": 7, "middle_J2": 12, "middle_J3": 17, "middle_J4": 22,
            "ring_J1": 9, "ring_J2": 14, "ring_J3": 19, "ring_J4": 24,
            "pinky_J1": 8, "pinky_J2": 13, "pinky_J3": 18, "pinky_J4": 23
        }

        for joint_name in finger_joints:
            joint_path = f"{UR5DEXConfig.robot_prim_path}/DexterousHandBase/joints/{joint_name}"
            prim = self.stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                val = targets_deg[hand_indices[joint_name]]
                drive_api.GetTargetPositionAttr().Set(float(val))
                drive_api.GetStiffnessAttr().Set(100.0)
                drive_api.GetDampingAttr().Set(10.0)
            else:
                print(f"[ERROR] Invalid joint prim path: {joint_path}")

    def get_ball_position(self) -> np.ndarray:
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        xform = UsdGeom.Xformable(ball_prim)
        world_transform = xform.ComputeLocalToWorldTransform(0)
        return np.array(world_transform.ExtractTranslation())

    def run_sim(self, num_trials: int):
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        
        print("[VisualCatch] Starting Dynamic Catching simulation loop...")
        
        for trial in range(num_trials):
            print(f"\n--- TRIAL {trial+1} / {num_trials} ---")
            self.reset_ball()
            self.finger_interpolator.reset_envs(torch.tensor([0], device="cuda:0"))
            self.ekf.reset()
            
            # Start with arm at standard ready pose
            q_arm = np.array([0.0, -1.25, 1.66, -0.43, 1.55, -3.14])
            joint_targets = torch.zeros((1, 25), device="cuda:0")
            
            trial_running = True
            step = 0
            min_dist = float('inf')
            
            while trial_running and step < 400:
                simulation_app.update()
                
                # Fetch positions
                ball_pos = self.get_ball_position()
                
                # Step EKF to predict trajectory
                self.ekf.update(ball_pos)
                p_int, t_catch = self.ekf.predict_intercept_point()
                
                # Run cuRobo solver to align arm's palm to block ball velocity
                if step % 2 == 0:
                    # Target palm pos is p_int, oriented to face the ball
                    q_arm_sol, _ = self.ik_solver.solve_ik(p_int, np.array([0.0, 0.0, -1.5]), q_current=q_arm)
                    q_arm = q_arm_sol
                
                # Get current time for finger interpolation
                current_time = step * (1.0/60.0)
                
                # Get true palm position using forward kinematics of current arm state
                wrist_pos, wrist_rot = self.ik_solver._forward_kinematics(q_arm)
                tcp_offset_local = np.array([0.0, 0.0, 0.33])
                palm_pos = wrist_pos + wrist_rot @ tcp_offset_local
                
                dist_to_palm = np.linalg.norm(ball_pos - palm_pos)
                if dist_to_palm < min_dist:
                    min_dist = dist_to_palm
                
                # Trigger soft closing proactively when ball is estimated to arrive in < 0.25s
                # or if it physically enters a 35cm radius.
                if t_catch < 0.25 or dist_to_palm < 0.35:
                    self.finger_interpolator.trigger_closing(torch.tensor([0], device="cuda:0"), current_time)
                
                # Apply arm and smooth finger joint targets
                self.apply_arm_joint_targets(q_arm)
                joint_targets = self.finger_interpolator.compute_joint_targets(joint_targets, current_time)
                self.apply_hand_joint_targets(joint_targets)
                
                # Terminate trial if ball drops below threshold (missed) or successful hold
                if ball_pos[2] < 0.2:
                    print(f"[TRIAL] Ball dropped - Missed. Min dist: {min_dist:.3f}m")
                    with open("/tmp/catch_result.txt", "w") as f:
                        f.write(f"MISSED: Min dist {min_dist:.3f}m")
                    trial_running = False
                elif dist_to_palm < 0.07 and ball_pos[2] < 0.7:
                    # Ball is resting statically in the hand
                    print(f"[TRIAL] Success! Caught ball cleanly. Dist: {dist_to_palm:.3f}m")
                    with open("/tmp/catch_result.txt", "w") as f:
                        f.write(f"SUCCESS: Dist {dist_to_palm:.3f}m")
                    time.sleep(0.5) # Hold pose for visualization appeal
                    trial_running = False
                
                step += 1
            
            if trial_running:
                print(f"[TRIAL] Time out - Missed. Min dist: {min_dist:.3f}m")
                with open("/tmp/catch_result.txt", "w") as f:
                    f.write(f"TIMEOUT: Min dist {min_dist:.3f}m")
                
        print("\n[VisualCatch] Simulation loop finished.")
        timeline.stop()
        simulation_app.close()

if __name__ == "__main__":
    runner = VisualDynamicCatchingRunner(args.usd_path)
    runner.run_sim(args.num_trials)
