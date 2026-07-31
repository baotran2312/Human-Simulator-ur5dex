import sys
import os
from isaaclab.app import AppLauncher
app_launcher = AppLauncher({"headless": True})
app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom

usd_path = "/home/ubuntu2204/Baro/Seqhandisaac/ur5dex.usd"
out_path = "/home/ubuntu2204/Baro/Seqhandisaac/ur5dex_collision.usd"

stage = Usd.Stage.Open(usd_path)
count = 0
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        # Add Collision API
        UsdPhysics.CollisionAPI.Apply(prim)
        # Add Mesh Collision API (set approximation to convexHull)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
        count += 1

print(f"Added collision to {count} meshes.")
stage.Export(out_path)
print(f"Saved to {out_path}")
