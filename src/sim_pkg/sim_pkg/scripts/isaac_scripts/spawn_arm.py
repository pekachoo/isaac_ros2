#!/usr/bin/env python3
from omni.isaac.kit import SimulationApp
sim_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf
import omni.usd

ASSET = "/home/jliu/isaac_ws/src/sim_pkg/sim_pkg/assets/two_link_arm.usda"
ROOT  = "/World/TwoLinkArm"   # parent xform under /World

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # (Re)spawn cleanly
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(ROOT):
        stage.RemovePrim(ROOT)

    # Add the arm and lift it 1.0 m
    add_reference_to_stage(usd_path=ASSET, prim_path=ROOT)
    x = UsdGeom.Xformable(stage.GetPrimAtPath(ROOT))
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))  # raise
    x.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))        # no rotation

    world.reset()
    while sim_app.is_running():
        world.step(render=True)

if __name__ == "__main__":
    main()
    sim_app.close()
