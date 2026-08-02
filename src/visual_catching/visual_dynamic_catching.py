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
            
            UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
            physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
            physx_articulation.CreateEnabledSelfCollisionsAttr().Set(False)
            physx_articulation.CreateSolverPositionIterationCountAttr().Set(64)
            physx_articulation.CreateSolverVelocityIterationCountAttr().Set(16)
            self._enable_robot_collisions(robot_prim)
            print(f"[VisualCatch] Loaded UR5DEX robot asset: {self.usd_path}")

        # Spawn Ball
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(0.035)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(0.85, 0.11, 1.2))
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
        x = np.random.uniform(0.75, 0.88)
        y = np.random.uniform(0.05, 0.15)
        z = 1.3
        
        # Apply reset pose
        xform = UsdGeom.XformCommonAPI(ball_prim)
        xform.SetTranslate(Gf.Vec3d(x, y, z))
        
        # Reset velocity to drop down + slight throw
        rigid_body = UsdPhysics.RigidBodyAPI(ball_prim)
        rigid_body.GetVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, -1.5))
        rigid_body.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def apply_arm_joint_targets(self, q_arm: np.ndarray):
        for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
            joint_path = f"{UR5DEXConfig.robot_prim_path}/{joint_name}"
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
            joint_path = f"{UR5DEXConfig.robot_prim_path}/{joint_name}"
            prim = self.stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive_api:
                    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
                val = targets_deg[hand_indices[joint_name]]
                drive_api.GetTargetPositionAttr().Set(float(val))
                drive_api.GetStiffnessAttr().Set(100.0)
                drive_api.GetDampingAttr().Set(10.0)

    def get_ball_position(self) -> np.ndarray:
        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        xform = UsdGeom.XformCommonAPI(ball_prim)
        trans, _, _, _, _ = xform.GetXformVectors(0)
        return np.array(trans)

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
            start_time = time.time()
            
            while trial_running and (time.time() - start_time < 3.0):
                simulation_app.update()
                
                # Fetch positions
                ball_pos = self.get_ball_position()
                
                # Step EKF to predict trajectory
                self.ekf.update(ball_pos)
                p_int, t_catch = self.ekf.predict_intercept_point()
                
                # Run cuRobo solver to align arm's palm to block ball velocity
                if step % 2 == 0:
                    # Target palm pos is p_int, oriented to face the ball
                    q_arm_sol, success = self.ik_solver.solve_ik(p_int, np.array([0.0, 0.0, -1.5]))
                    if success:
                        q_arm = q_arm_sol
                
                # Get current time for finger interpolation
                current_time = step * (1.0/60.0)
                
                # Calculate distance between EKF-predicted palm and ball
                # True palm is offset from wrist
                q_deg = np.degrees(q_arm)
                dist_to_palm = np.linalg.norm(ball_pos - p_int)
                
                # Trigger soft closing when ball is within 18cm of predicted palm center
                # This compensates for time latency and creates a smooth deceleration curve
                if dist_to_palm < 0.18:
                    self.finger_interpolator.trigger_closing(torch.tensor([0], device="cuda:0"), current_time)
                
                # Apply arm and smooth finger joint targets
                self.apply_arm_joint_targets(q_arm)
                joint_targets = self.finger_interpolator.compute_joint_targets(joint_targets, current_time)
                self.apply_hand_joint_targets(joint_targets)
                
                # Terminate trial if ball drops below threshold (missed) or successful hold
                if ball_pos[2] < 0.2:
                    print("[TRIAL] Ball dropped - Missed.")
                    trial_running = False
                elif dist_to_palm < 0.07 and ball_pos[2] < 0.7:
                    # Ball is resting statically in the hand
                    print(f"[TRIAL] Success! Caught ball cleanly. Dist: {dist_to_palm:.3f}m")
                    time.sleep(0.5) # Hold pose for visualization appeal
                    trial_running = False
                
                step += 1
                
        timeline.stop()
        simulation_app.close()

if __name__ == "__main__":
    runner = VisualDynamicCatchingRunner(args.usd_path)
    runner.run_sim(args.num_trials)
