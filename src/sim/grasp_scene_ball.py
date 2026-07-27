#!/usr/bin/env python3
"""
Physical Simulation Scene: Dynamic Ball Catching with UR5DEX Robot (Isaac Sim)
==============================================================================
This standalone script executes Week 1 Task 1:
- Loads UR5 + DH Dexterous Hand scene (`ur5dex.usd`).
- Sets up physical scene with gravity, friction, and collision contact sensors.
- Instantiates Dynamic Ball Launcher to shoot a sphere according to impulse force F / velocity v_0.
- Logs ball trajectory (x,y,z), intercept errors, and collision contact forces.

Usage:
    /home/nhglab/anaconda3/envs/env_isaacsim/bin/python src/sim/grasp_scene_ball.py --headless
"""

import argparse
import os
import sys
import time
import numpy as np

# Add src root to Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from src.sim.physics_config import PhysicsSceneConfig, DynamicBallConfig, UR5DEXConfig

parser = argparse.ArgumentParser(description="UR5DEX Physical Simulation Scene")
parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI")
parser.add_argument("--usd_path", type=str, default=UR5DEXConfig.usd_asset_path, help="Path to ur5dex.usd")
parser.add_argument("--num_throws", type=int, default=5, help="Number of ball throw iterations")
args, _ = parser.parse_known_args()

# --- Initialize Isaac Sim App ---
try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
except ImportError:
    print("[ERROR] Failed to import `isaacsim.SimulationApp`. Please run inside env_isaacsim or env_isaaclab.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.timeline
import omni.physx as physx
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf

class UR5DEXPhysicalSimulation:
    def __init__(self, usd_path: str):
        self.usd_path = usd_path
        self.phys_cfg = PhysicsSceneConfig()
        self.ball_cfg = DynamicBallConfig()
        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()
        
        self.ball_prim_path = "/World/DynamicBall"
        self._setup_scene()

    def _setup_scene(self):
        """Constructs physical environment stage."""
        print(f"[PhysicalSim] Building scene stage...")
        
        # 1. Physics Scene
        physics_scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/World/PhysicsScene"))
        physics_scene.GetGravityAttr().Set(Gf.Vec3f(*self.phys_cfg.gravity))

        # 2. Ground Plane
        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

        # 3. Robot USD Asset
        if os.path.exists(self.usd_path):
            robot_prim = self.stage.DefinePrim(UR5DEXConfig.robot_prim_path, "Xform")
            robot_prim.GetReferences().AddReference(self.usd_path)
            print(f"[PhysicalSim] Loaded UR5DEX robot asset from: {self.usd_path}")
        else:
            print(f"[WARNING] USD asset not found at {self.usd_path}. Running with standalone ball launcher physics.")

        # 4. Create Dynamic Ball Rigid Body
        self._create_dynamic_ball()

    def _create_dynamic_ball(self):
        """Spawns dynamic ball with collision and physics API."""
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_prim_path))
        sphere_geom.GetRadiusAttr().Set(self.ball_cfg.radius)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(*self.ball_cfg.initial_position))
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(0.95, 0.35, 0.05)]) # Orange ball

        # Apply Physics
        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(self.ball_cfg.mass)

        # Contact Report API
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())
        print(f"[PhysicalSim] Dynamic ball spawned at {self.ball_cfg.initial_position}")

    def launch_ball(self, initial_pos=None, launch_vel=None):
        """Applies impulse velocity vector to launch ball towards UR5 hand workspace."""
        pos = initial_pos if initial_pos is not None else self.ball_cfg.initial_position
        vel = launch_vel if launch_vel is not None else self.ball_cfg.launch_velocity

        ball_prim = self.stage.GetPrimAtPath(self.ball_prim_path)
        if not ball_prim.IsValid():
            return

        # Reset Transform
        xform = UsdGeom.Xformable(ball_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(*pos))

        # Reset Velocity
        physx_api = PhysxSchema.PhysxRigidBodyAPI(ball_prim)
        if not physx_api:
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(ball_prim)

        physx_api.GetLinearVelocityAttr().Set(Gf.Vec3f(*vel))
        physx_api.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        print(f"[PhysicalSim] Ball Launched! Initial Pos={pos}, Initial Vel={vel} m/s")

    def get_ball_pose_vel(self):
        """Queries world 3D position and velocity of ball."""
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
    sim = UR5DEXPhysicalSimulation(usd_path=args.usd_path)
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    time.sleep(0.5)

    sim.launch_ball()

    step_cnt = 0
    throw_cnt = 1
    
    print("\n=======================================================")
    print("  UR5DEX PHYSICAL SIMULATION RUNNING (Week 1 Task 1)")
    print("=======================================================\n")

    while simulation_app.is_running() and throw_cnt <= args.num_throws:
        simulation_app.update()
        step_cnt += 1

        if step_cnt % 30 == 0:
            pos, vel = sim.get_ball_pose_vel()
            print(f"[Step {step_cnt:04d} | Throw {throw_cnt}/{args.num_throws}] Ball Pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) | Speed: {np.linalg.norm(vel):.2f} m/s")

        # Auto re-launch next throw every 200 steps
        if step_cnt % 200 == 0:
            throw_cnt += 1
            if throw_cnt <= args.num_throws:
                # Randomize throw velocity for dynamic trajectory testing
                vx = -1.5 + np.random.uniform(-0.3, 0.3)
                vy = np.random.uniform(-0.2, 0.2)
                vz = 0.3 + np.random.uniform(-0.1, 0.2)
                sim.launch_ball(launch_vel=[vx, vy, vz])

    print("\n[PhysicalSim] Simulation completed successfully.")
    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
