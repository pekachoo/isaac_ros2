#!/usr/bin/env python3
from omni.isaac.kit import SimulationApp
sim_app = SimulationApp({"headless": False})  # set True if headless

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd

ASSET_USD = "/home/jliu/isaac_ws/assets/box.usda"
PRIM_PATH  = "/World/Box"   # where to place it in the stage

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Reference the USD into the stage
    add_reference_to_stage(usd_path=ASSET_USD, prim_path=PRIM_PATH)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(PRIM_PATH)

    # The referenced defaultPrim is a Cube, so PRIM_PATH should be that Cube prim.
    # Give it physics so gravity and collisions work.
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

    # Move it a bit above the ground
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3f(0.0, 0.0, 0.5))

    # Optional: set mass (defaults are fine, but you can tweak)
    # from pxr import UsdPhysics
    # UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1.0)

    world.reset()
    while sim_app.is_running():
        world.step(render=True)

if __name__ == "__main__":
    main()
    sim_app.close()
