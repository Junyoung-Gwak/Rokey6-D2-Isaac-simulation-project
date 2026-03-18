import numpy as np
from math import cos, sin, radians
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation as R

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.prims import XFormPrim
from pxr import Sdf, UsdLux

GroundPlane(prim_path="/World/GroundPlane", z_position=0)

stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

rotate_offset = np.array([135.0, 0.0, 0.0])
rot = R.from_euler('xyz', [rotate_offset[0], rotate_offset[1], rotate_offset[2]], degrees=True)
q_scipy = rot.as_quat()
orientation_offset = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])

rotate_offset2 = np.array([45.0, 0.0, 0.0])
rot2 = R.from_euler('xyz', [rotate_offset2[0], rotate_offset2[1], rotate_offset2[2]], degrees=True)
q_scipy2 = rot2.as_quat()
orientation_offset2 = np.array([q_scipy2[3], q_scipy2[0], q_scipy2[1], q_scipy2[2]])


for i in range(2):
    z = 10.0 - 4* (i-1)
    DynamicCuboid(
        prim_path=f"/World/dynamic_cube_{i}", 
        name=f"dynamic_cube_{i}",
        position=np.array([0, -2.0, z]),
        orientation=orientation_offset,        
        scale=np.array([2.0, 5.0, 0.2]),
        color=np.array([1.0, 0.0, 0.0]), 
    )

for i in range(2):
    z = 8.0 - 4* (i-1)
    DynamicCuboid(
        prim_path=f"/World/dynamic_cube_1{i}", 
        name=f"dynamic_cube_1{i}",
        position=np.array([0, 2.0, z]),
        orientation=orientation_offset2,         
        scale=np.array([2.0, 5.0, 0.2]),
        color=np.array([1.0, 0.0, 0.0]), 
    ) 

DynamicSphere(
   prim_path="/World/dynamic_sphere",
   name="dynamic_sphere",
   position=np.array([0, -3.0, 10.0]),
   radius=0.5,                        
   color=np.array([0.0, 0.0, 1.0]),    
)

my_world = World(stage_units_in_meters=1.0)
my_world.reset() 

for j in range(1000):
    my_world.step(render=True) 

# 창이 닫히면 종료
simulation_app.close()