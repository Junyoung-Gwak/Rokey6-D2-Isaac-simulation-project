import numpy as np
from math import cos, sin, radians
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation as R

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere, VisualSphere,VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.prims import XFormPrim
from pxr import Sdf, UsdLux, UsdPhysics, Gf
from isaacsim.core.prims import RigidPrim
from isaacsim.core.prims import GeometryPrim

GroundPlane(prim_path="/World/GroundPlane", z_position=0)

stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

domeLight = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
domeLight.CreateIntensityAttr(1000)

rotate_offset = np.array([150.0, 0.0, 0.0])
rot = R.from_euler('xyz', [rotate_offset[0], rotate_offset[1], rotate_offset[2]], degrees=True)
q_scipy = rot.as_quat()
orientation_offset = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])

rotate_offset2 = np.array([30.0, 0.0, 0.0])
rot2 = R.from_euler('xyz', [rotate_offset2[0], rotate_offset2[1], rotate_offset2[2]], degrees=True)
q_scipy2 = rot2.as_quat()
orientation_offset2 = np.array([q_scipy2[3], q_scipy2[0], q_scipy2[1], q_scipy2[2]])

rotate_offset3 = np.array([90.0, 0.0, 0.0])
rot3 = R.from_euler('xyz', [rotate_offset3[0], rotate_offset3[1], rotate_offset3[2]], degrees=True)
q_scipy3 = rot3.as_quat()
orientation_offset3 = np.array([q_scipy3[3], q_scipy3[0], q_scipy3[1], q_scipy3[2]])

rotate_offset4 = np.array([0.0, 90.0, 0.0])
rot4 = R.from_euler('xyz', [rotate_offset4[0], rotate_offset4[1], rotate_offset4[2]], degrees=True)
q_scipy4 = rot4.as_quat()
orientation_offset4 = np.array([q_scipy4[3], q_scipy4[0], q_scipy4[1], q_scipy4[2]])

for i in range(3):
    z = 14.0 - 4* (i)
    visual_cube = VisualCuboid(
        prim_path=f"/World/Visual_cube_{i}", 
        name=f"Visual_cube_{i}",
        position=np.array([0, -2.0, z]),
        orientation=orientation_offset,        
        scale=np.array([3.0, 5.0, 0.2]),
        color=np.array([1.0, 0.0, 0.0]), 
    )

for i in range(3):
    z = 12.0 - 4* (i)
    visual_cube = VisualCuboid(
        prim_path=f"/World/Visual_cube_1{i}", 
        name=f"Visual_cube_1{i}",
        position=np.array([0, 2.0, z]),
        orientation=orientation_offset2,         
        scale=np.array([3.0, 5.0, 0.2]),
        color=np.array([1.0, 0.0, 0.0]), 
    ) 

DynamicSphere(
   prim_path="/World/Dynamic_sphere",
   name="Dynamic_sphere",
   position=np.array([0, -3.0, 16.0]),
   radius=0.5,                        
   color=np.array([0.0, 0.0, 1.0]),    
)

DynamicCuboid(
    prim_path=f"/World/Dynamic_box0", 
    name=f"Dynamic_box0",
    position=np.array([0, -2.0, 0.1]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),       
    scale=np.array([1.0, 1.0, 0.2]),
    color=np.array([1.0, 0.0, 0.0]), 
) 

DynamicCuboid(
    prim_path=f"/World/Dynamic_box1", 
    name=f"Dynamic_box1",
    position=np.array([0, -1.4, 0.5]),
    orientation=orientation_offset3,     
    scale=np.array([1.0, 1.0, 0.2]),
    color=np.array([1.0, 0.0, 0.0]), 
) 

DynamicCuboid(
    prim_path=f"/World/Dynamic_box2", 
    name=f"Dynamic_box2",
    position=np.array([-0.6, -2.0, 0.5]),
    orientation=orientation_offset4,         
    scale=np.array([1.0, 1.0, 0.2]),
    color=np.array([1.0, 0.0, 0.0]), 
) 

DynamicCuboid(
    prim_path=f"/World/Dynamic_box3", 
    name=f"Dynamic_box3",
    position=np.array([0, -2.6, 0.5]),
    orientation=orientation_offset3,      
    scale=np.array([1.0, 1.0, 0.2]),
    color=np.array([1.0, 0.0, 0.0]), 
) 

DynamicCuboid(
    prim_path=f"/World/Dynamic_box4", 
    name=f"Dynamic_box4",
    position=np.array([0.6, -2.0, 0.5]),
    orientation=orientation_offset4,      
    scale=np.array([1.0, 1.0, 0.2]),
    color=np.array([1.0, 0.0, 0.0]), 
) 

my_world = World(stage_units_in_meters=1.0)

def to_gf_quat(q_array):
    return Gf.Quatf(float(q_array[0]), float(q_array[1]), float(q_array[2]), float(q_array[3]))

fixed_joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path("/World/my_fixed_joint"))
fixed_joint2 = UsdPhysics.FixedJoint.Define(stage, Sdf.Path("/World/my_fixed_joint2"))
fixed_joint3 = UsdPhysics.FixedJoint.Define(stage, Sdf.Path("/World/my_fixed_joint3"))
fixed_joint4 = UsdPhysics.FixedJoint.Define(stage, Sdf.Path("/World/my_fixed_joint4"))

fixed_joint.GetBody0Rel().SetTargets([Sdf.Path("/World/Dynamic_box0")])
fixed_joint.GetBody1Rel().SetTargets([Sdf.Path("/World/Dynamic_box1")])
fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.6, 0.4))
fixed_joint.CreateLocalRot0Attr().Set(to_gf_quat(orientation_offset3))
fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


fixed_joint2.GetBody0Rel().SetTargets([Sdf.Path("/World/Dynamic_box0")])
fixed_joint2.GetBody1Rel().SetTargets([Sdf.Path("/World/Dynamic_box2")])
fixed_joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.6, 0.0, 0.4))
fixed_joint2.CreateLocalRot0Attr().Set(to_gf_quat(orientation_offset4))
fixed_joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fixed_joint2.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

fixed_joint3.GetBody0Rel().SetTargets([Sdf.Path("/World/Dynamic_box0")])
fixed_joint3.GetBody1Rel().SetTargets([Sdf.Path("/World/Dynamic_box3")])
fixed_joint3.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, -0.6, 0.4))
fixed_joint3.CreateLocalRot0Attr().Set(to_gf_quat(orientation_offset3))
fixed_joint3.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fixed_joint3.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

fixed_joint4.GetBody0Rel().SetTargets([Sdf.Path("/World/Dynamic_box0")])
fixed_joint4.GetBody1Rel().SetTargets([Sdf.Path("/World/Dynamic_box4")])
fixed_joint4.CreateLocalPos0Attr().Set(Gf.Vec3f(0.6, 0.0, 0.4))
fixed_joint4.CreateLocalRot0Attr().Set(to_gf_quat(orientation_offset4))
fixed_joint4.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fixed_joint4.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

for i in range(3):
    prim = GeometryPrim(f"/World/Visual_cube_{i}")
    prim2 = GeometryPrim(f"/World/Visual_cube_1{i}")
    prim.apply_collision_apis()
    prim2.apply_collision_apis()

my_world.reset() 

for j in range(1000):
    my_world.step(render=True) 

# 창이 닫히면 종료
simulation_app.close()