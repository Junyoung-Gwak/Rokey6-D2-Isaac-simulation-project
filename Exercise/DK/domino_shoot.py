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


for i in range(20):
    angle_deg = 9 * i  
    angle_rad = radians(angle_deg)
    
    x = 5 * cos(angle_rad)
    y = 5 * sin(angle_rad)
    
    rot = R.from_euler('z', angle_deg, degrees=True)
    q_scipy = rot.as_quat()
    q_isaac = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
    
    DynamicCuboid(
        prim_path=f"/World/dynamic_cube_{i}", 
        name=f"dynamic_cube_{i}",
        position=np.array([x, y, 1.0]),
        orientation=q_isaac,             
        scale=np.array([1.0, 0.2, 1.0]),
        color=np.array([1.0, 0.0, 0.0]), 
    )
     
my_sphere =DynamicSphere(
   prim_path="/World/dynamic_sphere",
   name="dynamic_sphere",
   position=np.array([5.0, -5.0, 0.5]),
   radius=0.5,                        
   color=np.array([0.0, 0.0, 1.0]),    
   mass=5.0
)

my_world = World(stage_units_in_meters=1.0)
my_world.reset() 

shoot_velocity = np.array([0.0, 10.0, 0.0])
my_sphere.set_linear_velocity(shoot_velocity)

for j in range(1000):
    my_world.step(render=True) 

# 창이 닫히면 종료
simulation_app.close()