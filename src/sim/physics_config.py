"""
Physical Simulation Configuration for UR5DEX Dynamic Catching (Isaac Sim)
==========================================================================
Defines all physical parameters, mass properties, friction coefficients, 
initial conditions, and launch velocity profiles for dynamic ball catching.
"""

from dataclasses import dataclass
from typing import Tuple

@dataclass
class PhysicsSceneConfig:
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)
    time_steps_per_second: int = 60
    solver_position_iterations: int = 8
    solver_velocity_iterations: int = 2

@dataclass
class DynamicBallConfig:
    mass: float = 0.15          # Mass in kg (standard ball)
    radius: float = 0.035       # Radius in meters (3.5cm)
    restitution: float = 0.6    # Coefficient of restitution (bounciness)
    static_friction: float = 0.7
    dynamic_friction: float = 0.5
    
    # Launch parameters
    initial_position: Tuple[float, float, float] = (1.2, 0.0, 0.8) # (x, y, z) meters relative to robot base
    launch_velocity: Tuple[float, float, float] = (-1.58, 0.24, 1.38) # Perfectly hits palm at t=0.45s
    launch_force_magnitude: float = 2.5                             # Impulse Force F in Newtons

@dataclass
class UR5DEXConfig:
    usd_asset_path: str = "/home/ubuntu2204/Baro/Seqhandisaac/ur5dex_collision.usd"
    robot_prim_path: str = "/World/UR5DEX"
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Palm and Fingertip Prim Paths for Contact Sensor API
    palm_prim_path: str = "/World/UR5DEX/DH_base_link"
    fingertip_prim_paths: Tuple[str, ...] = (
        "/World/UR5DEX/thumb_Link3",
        "/World/UR5DEX/index_Link4",
        "/World/UR5DEX/middle_Link4",
        "/World/UR5DEX/ring_Link4",
        "/World/UR5DEX/pinky_Link4",
    )
