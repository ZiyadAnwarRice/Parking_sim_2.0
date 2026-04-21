from dataclasses import dataclass
import math
import pybullet as p
import pybullet_data

LOT_X_HALF = 5.0
LOT_Y_HALF = 4.5

PARKING_SPOTS = {
    "P1": [4.15, -2.9, 0.02],
    "P2": [4.15, -0.9, 0.02],
    "P3": [4.15,  1.1, 0.02],
    "P4": [4.15,  3.1, 0.02],
}

ENTRY_X = -2.8
EXIT_X = 2.8
GATE_WIDTH = 1.2

@dataclass
class Scene:
    car: int
    wheels: list
    steering: list

def wall(pos, half, color=(0.65, 0.65, 0.65, 1)):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos,
    )

def box(pos, half, color=(1, 1, 1, 1), mass=0, orientation=None):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    if orientation is None:
        orientation = [0, 0, 0, 1]
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos,
        baseOrientation=orientation,
    )

def cylinder(pos, radius, height, color=(0.55, 0.55, 0.55, 1), mass=0):
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    vis = p.createVisualShape(
        p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color
    )
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos,
    )

def line_segment(start_xy, end_xy, width=0.06, z=0.022, color=(1, 1, 1, 1)):
    sx, sy = start_xy
    ex, ey = end_xy
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)

    center = [(sx + ex) / 2.0, (sy + ey) / 2.0, z]
    half = [length / 2.0, width / 2.0, 0.01]
    quat = p.getQuaternionFromEuler([0, 0, yaw])

    box(center, half, color=color, mass=0, orientation=quat)

def polyline(points, width=0.06, z=0.022, color=(1, 1, 1, 1)):
    for i in range(len(points) - 1):
        line_segment(points[i], points[i + 1], width=width, z=z, color=color)

def set_gui_top_down():
    p.resetDebugVisualizerCamera(
        cameraDistance=11.5,
        cameraYaw=0,
        cameraPitch=-89.9,
        cameraTargetPosition=[0, 0, 0],
    )

def build_world(selected_spot: str) -> Scene:
    p.connect(p.GUI)
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setRealTimeSimulation(0)
    p.setTimeStep(1 / 240.0)

    p.loadURDF("plane.urdf")

    # Asphalt floor overlay
    box(
        pos=[0, 0, 0.01],
        half=[LOT_X_HALF, LOT_Y_HALF, 0.01],
        color=(0.16, 0.16, 0.16, 1.0),
        mass=0,
    )

    # ----------------------------
    # Boundary walls
    # Top wall
    # ----------------------------
    wall([0, LOT_Y_HALF, 0.5], [LOT_X_HALF, 0.2, 0.5])

    # Left and right side walls
    wall([-LOT_X_HALF, 0, 0.5], [0.2, LOT_Y_HALF, 0.5])
    wall([ LOT_X_HALF, 0, 0.5], [0.2, LOT_Y_HALF, 0.5])

    # Bottom wall with entry and exit gaps
    # Full span is x in [-5, 5]
    # Entry gap centered at ENTRY_X, Exit gap centered at EXIT_X
    entry_left = ENTRY_X - GATE_WIDTH / 2.0
    entry_right = ENTRY_X + GATE_WIDTH / 2.0
    exit_left = EXIT_X - GATE_WIDTH / 2.0
    exit_right = EXIT_X + GATE_WIDTH / 2.0

    # Segment 1: left of entry gap
    wall(
        [(-LOT_X_HALF + entry_left) / 2.0, -LOT_Y_HALF, 0.5],
        [(entry_left - (-LOT_X_HALF)) / 2.0, 0.2, 0.5],
    )

    # Segment 2: between entry and exit gaps
    wall(
        [(entry_right + exit_left) / 2.0, -LOT_Y_HALF, 0.5],
        [(exit_left - entry_right) / 2.0, 0.2, 0.5],
    )

    # Segment 3: right of exit gap
    wall(
        [(exit_right + LOT_X_HALF) / 2.0, -LOT_Y_HALF, 0.5],
        [(LOT_X_HALF - exit_right) / 2.0, 0.2, 0.5],
    )

    # ----------------------------
    # Columns
    # ----------------------------
    cylinder([0.0, 0.0, 0.60], radius=0.45, height=1.20, color=(0.70, 0.70, 0.70, 1.0))
    for yy in [-2.4, 0.0, 2.4]:
        cylinder([-3.2, yy, 0.50], radius=0.18, height=1.00, color=(0.78, 0.78, 0.78, 1.0))
        cylinder([ 3.2, yy, 0.50], radius=0.18, height=1.00, color=(0.78, 0.78, 0.78, 1.0))

    # ----------------------------
    # Driveway loop markings
    # These are placed INSIDE the side column rows, not near the outer wall.
    # Outer lane boundary
    # ----------------------------
    outer_loop = [
        (-2.65, -3.55),
        ( 2.65, -3.55),
        ( 2.65,  3.55),
        (-2.65,  3.55),
        (-2.65, -3.55),
    ]

    # Inner lane boundary around center pillar
    inner_loop = [
        (-1.15, -1.55),
        ( 1.15, -1.55),
        ( 1.15,  1.55),
        (-1.15,  1.55),
        (-1.15, -1.55),
    ]

    polyline(outer_loop, width=0.07, z=0.022, color=(1, 1, 1, 1))
    polyline(inner_loop, width=0.07, z=0.022, color=(1, 1, 1, 1))

    # Entry and exit connectors from the bottom wall into the loop
    line_segment((ENTRY_X, -4.45), (ENTRY_X, -3.55), width=0.07, z=0.022, color=(1, 1, 1, 1))
    line_segment((EXIT_X,  -4.45), (EXIT_X,  -3.55), width=0.07, z=0.022, color=(1, 1, 1, 1))

    # Optional short transverse bars to make the gate more visible
    line_segment((ENTRY_X - 0.32, -4.18), (ENTRY_X + 0.32, -4.18), width=0.05, z=0.022, color=(0.9, 0.9, 0.9, 1))
    line_segment((EXIT_X  - 0.32, -4.18), (EXIT_X  + 0.32, -4.18), width=0.05, z=0.022, color=(0.9, 0.9, 0.9, 1))

    # Entry / Exit labels
    p.addUserDebugText(
        text="ENTRY",
        textPosition=[ENTRY_X - 0.40, -4.05, 0.08],
        textColorRGB=[0.2, 1.0, 0.2],
        textSize=1.3,
    )
    p.addUserDebugText(
        text="EXIT",
        textPosition=[EXIT_X - 0.25, -4.05, 0.08],
        textColorRGB=[1.0, 0.6, 0.2],
        textSize=1.3,
    )

    # ----------------------------
    # Parking spots and labels
    # ----------------------------
    for name, pos in PARKING_SPOTS.items():
        color = (0.0, 0.85, 0.15, 1.0) if name == selected_spot else (0.15, 0.35, 0.8, 1.0)
        box(pos=pos, half=[0.45, 0.35, 0.02], color=color, mass=0)
        p.addUserDebugText(
            text=name,
            textPosition=[pos[0] - 0.18, pos[1] - 0.10, 0.08],
            textColorRGB=[1, 1, 1],
            textSize=1.6,
        )

    # ----------------------------
    # Car
    # ----------------------------
    start_pos = [-4.0, -3.3, 0.20]
    start_yaw = 0.0
    start_orn = p.getQuaternionFromEuler([0, 0, start_yaw])

    car = p.loadURDF("racecar/racecar.urdf", start_pos, start_orn)

    wheels = [2]
    steering = [4, 6]
    inactive_wheels = [3, 5, 7]

    for w in inactive_wheels:
        p.setJointMotorControl2(car, w, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

    for _ in range(120):
        p.stepSimulation()

    set_gui_top_down()
    return Scene(car=car, wheels=wheels, steering=steering)