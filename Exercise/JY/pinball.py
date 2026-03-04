import numpy as np
from math import cos, sin, radians
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation as R

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere, FixedCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.prims import XFormPrim
from pxr import Sdf, UsdLux

GroundPlane(prim_path="/World/GroundPlane", z_position=0)

stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(1000)

# world = World()
# world.scene.add_default_ground_plane()


num_slopes = 6
L = 7.0
width = 4.0
thickness = 0.3
angle_deg = 30
angle_rad = np.deg2rad(angle_deg)

base_x = 0.0
base_y = 0.0
start_z = 50.0

current_top = np.array([base_x, base_y, start_z])


for i in range(num_slopes):
    direction = 1 if i % 2 == 0 else -1

    rot = R.from_euler('y', direction * angle_deg, degrees=True)
    q = rot.as_quat()
    quat = np.array([q[3], q[0], q[1], q[2]])

    center = current_top + np.array([
        direction * (L/2) * np.cos(angle_rad),
        0.0,
        -(L/2) * np.sin(angle_rad)
    ])

    FixedCuboid(
        prim_path=f"/World/Slope_{i}",
        name=f"slope_{i}",
        position=center,
        scale=[L, width, thickness],
        orientation=quat,
        color=np.array([0.2, 0.6, 0.8])
    )

    bottom = current_top + np.array([
        direction * L * np.cos(angle_rad),
        0.0,
        -L * np.sin(angle_rad)
    ])

    overlap_x = 6
    gap_z = 4.0

    current_top = bottom + np.array([direction * overlap_x, 0, -gap_z])


sphere = DynamicSphere(
    prim_path="/World/FallingSphere",
    name="falling_sphere",
    position=[base_x + 5, base_y, start_z + 1.5],
    radius=0.5,
    mass=2.0
)

world = World(stage_units_in_meters=1.0)
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()