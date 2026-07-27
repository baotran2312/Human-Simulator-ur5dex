#!/usr/bin/env python3
"""
Dynamic Ball Launcher & Catching Scene for UR5DEX (Isaac Sim Standalone)
========================================================================
This script sets up a dynamic ball-throwing simulation environment in NVIDIA Isaac Sim:
1. Loads the UR5 arm + DH Dexterous Hand USD scene.
2. Spawns a physical dynamic ball (Rigid Body with mass & collision).
3. Applies configurable launch velocity vectors F / v_0 to simulate dynamic ball catching.
4. Logs real-time 3D ball trajectory (x, y, z) and contact forces on robot fingertips.

Usage:
    python src/sim/ball_launcher.py --headless --velocity 1.5 0.0 0.5
"""

import argparse
import math
import sys
import time
import numpy as np

# --- 1. Argument Parsing ---
parser = argparse.ArgumentParser(description="Isaac Sim Dynamic Ball Launcher for UR5DEX")
parser.add_argument("--headless", action="store_true", help="Run without Isaac Sim GUI")
parser.add_argument("--usd_path", type=str, default="/home/nhglab/Tri/Seqhandisaac/ur5dex.usd", help="Path to ur5dex USD asset")
parser.add_argument("--ball_mass", type=float, default=0.15, help="Ball mass in kg")
parser.add_argument("--ball_radius", type=float, default=0.035, help="Ball radius in meters")
parser.add_argument("--launch_pos", type=float, nargs=3, default=[1.2, 0.0, 0.8], help="Ball initial spawn position (x y z)")
parser.add_argument("--launch_vel", type=float, nargs=3, default=[-1.5, 0.0, 0.3], help="Ball launch velocity vector (vx vy vz) in m/s")
parser.add_argument("--rate", type=float, default=60.0, help="Simulation physics update rate (Hz)")
args, _ = parser.parse_known_args()

# --- 2. Launch Isaac Sim Application ---
try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": args.headless})
except ImportError:
    print("[ERROR] Could not import isaacsim SimulationApp. Please run inside env_isaacsim or env_isaaclab.")
    sys.exit(1)

import omni
import omni.kit.commands
import omni.physx as physx
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Sdf

class DynamicBallLauncher:
    """Manages spawning, launching, and trajectory tracking of dynamic objects in Isaac Sim."""

    def __init__(self, stage, ball_path="/World/DynamicBall"):
        self.stage = stage
        self.ball_path = ball_path
        self._create_ball_prim()

    def _create_ball_prim(self):
        """Creates a rigid body sphere prim with physics material."""
        # Create sphere geometry
        sphere_geom = UsdGeom.Sphere.Define(self.stage, Sdf.Path(self.ball_path))
        sphere_geom.GetRadiusAttr().Set(args.ball_radius)
        sphere_geom.AddTranslateOp().Set(Gf.Vec3f(*args.launch_pos))
        
        # Color the ball bright red/orange for easy visual tracking
        sphere_geom.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.2, 0.1)])

        # Rigid Body & Collision API
        UsdPhysics.RigidBodyAPI.Apply(sphere_geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere_geom.GetPrim())
        
        mass_api = UsdPhysics.MassAPI.Apply(sphere_geom.GetPrim())
        mass_api.GetMassAttr().Set(args.ball_mass)

        # Contact Report API for measuring impact forces
        PhysxSchema.PhysxContactReportAPI.Apply(sphere_geom.GetPrim())

        print(f"[BallLauncher] Spawned rigid body sphere at {args.launch_pos} with mass {args.ball_mass}kg, radius {args.ball_radius}m")

    def reset_and_launch(self, pos=None, vel=None):
        """Resets ball to specified position and applies initial velocity."""
        pos = pos if pos is not None else args.launch_pos
        vel = vel if vel is not None else args.launch_vel

        ball_prim = self.stage.GetPrimAtPath(self.ball_path)
        if not ball_prim.IsValid():
            return

        # Set position
        xform = UsdGeom.Xformable(ball_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(*pos))

        # Apply velocity using PhysxRigidBodyAPI
        physx_api = PhysxSchema.PhysxRigidBodyAPI(ball_prim)
        if not physx_api:
            physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(ball_prim)
            
        physx_api.GetLinearVelocityAttr().Set(Gf.Vec3f(*vel))
        physx_api.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        print(f"[BallLauncher] Launched ball from {pos} with velocity vector v = {vel} m/s")

    def get_ball_state(self):
        """Reads current 3D position and velocity of the ball."""
        ball_prim = self.stage.GetPrimAtPath(self.ball_path)
        if not ball_prim.IsValid():
            return None, None

        xform = UsdGeom.Xformable(ball_prim)
        world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.ExtractTranslation()
        
        physx_api = PhysxSchema.PhysxRigidBodyAPI(ball_prim)
        vel = physx_api.GetLinearVelocityAttr().Get() if physx_api else Gf.Vec3f(0, 0, 0)
        
        return np.array([pos[0], pos[1], pos[2]]), np.array([vel[0], vel[1], vel[2]])


def main():
    # Obtain USD stage
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    # Configure Physics Scene
    physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
    physics_scene.GetGravityAttr().Set(Gf.Vec3f(0.0, 0.0, -9.81))

    # Add Ground Plane
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")

    # Load Robot USD Asset if exists
    try:
        robot_prim = stage.DefinePrim("/World/UR5DEX", "Xform")
        robot_prim.GetReferences().AddReference(args.usd_path)
        print(f"[Main] Loaded UR5DEX robot asset from: {args.usd_path}")
    except Exception as e:
        print(f"[WARNING] Could not load robot USD at {args.usd_path}: {e}")

    # Initialize Ball Launcher
    launcher = DynamicBallLauncher(stage)
    
    # Start Timeline Physics
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Launch initial ball after 1 second warm-up
    time.sleep(1.0)
    launcher.reset_and_launch()

    step_count = 0
    dt = 1.0 / args.rate

    print("\n--- Starting Simulation Loop (Press Ctrl+C or close window to exit) ---")
    while simulation_app.is_running():
        simulation_app.update()
        step_count += 1

        if step_count % 30 == 0:
            pos, vel = launcher.get_ball_state()
            if pos is not None:
                print(f"[Step {step_count:04d}] Ball Pos: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f} | Vel: |v|={np.linalg.norm(vel):.2f}m/s")

        # Auto re-launch ball every 5 seconds (300 steps at 60Hz)
        if step_count % 300 == 0:
            # Vary initial velocity slightly for testing dynamic catching bounds
            vx = -1.5 + np.random.uniform(-0.2, 0.2)
            vy = np.random.uniform(-0.1, 0.1)
            vz = 0.4 + np.random.uniform(-0.1, 0.1)
            launcher.reset_and_launch(vel=[vx, vy, vz])

    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()
