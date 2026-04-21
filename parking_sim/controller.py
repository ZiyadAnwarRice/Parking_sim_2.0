import math
import time
import pybullet as p

from path_utils import load_path
from sim_env import build_world, set_gui_top_down
from views import MultiViewRenderer

BASE_SPEED   = 42.0
MIN_SPEED    = 9.0
MOTOR_FORCE  = 38.0

LOOKAHEAD    = 0.45
WHEELBASE    = 0.25
MAX_STEER    = 0.6

WAYPOINT_TOL = 0.18
GOAL_TOL     = 0.20

DISPLAY_EVERY_N = 12
BRAKE_STEPS     = 220
MAX_STEPS       = 18000

STEPS_PER_SLEEP = 10

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_control(car, wheels, steering, speed_cmd, steer_cmd):
    for w in wheels:
        p.setJointMotorControl2(
            car,
            w,
            p.VELOCITY_CONTROL,
            targetVelocity=speed_cmd,
            force=MOTOR_FORCE,
        )
    for s in steering:
        p.setJointMotorControl2(
            car,
            s,
            p.POSITION_CONTROL,
            targetPosition=steer_cmd,
        )

def brake_and_stop(car, wheels, steering, renderer):
    steps_per_phase = BRAKE_STEPS // 3
    for phase, (vel, force) in enumerate([(6.0, 8.0), (1.5, 5.0), (0.0, 4.0)]):
        for i in range(steps_per_phase):
            for w in wheels:
                p.setJointMotorControl2(
                    car, w,
                    p.VELOCITY_CONTROL,
                    targetVelocity=vel,
                    force=force,
                )
            for s in steering:
                p.setJointMotorControl2(
                    car, s,
                    p.POSITION_CONTROL,
                    targetPosition=0.0,
                )
            p.stepSimulation()
            if i % DISPLAY_EVERY_N == 0:
                renderer.render(car)
            time.sleep(1 / 240.0)

def run_parking_demo(selected_spot="P4"):
    path_data  = load_path(selected_spot)
    goal_xy    = tuple(path_data["goal"][:2])
    waypoints  = [tuple(pt[:2]) for pt in path_data["waypoints"]]

    scene    = build_world(selected_spot)
    renderer = MultiViewRenderer()

    for i, (wx, wy) in enumerate(waypoints):
        p.addUserDebugText(
            text=f"W{i+1}",
            textPosition=[wx, wy, 0.08],
            textColorRGB=[0.2, 0.9, 1.0],
            textSize=1.0,
        )
        if i < len(waypoints) - 1:
            nx, ny = waypoints[i + 1]
            p.addUserDebugLine([wx, wy, 0.04], [nx, ny, 0.04], [0.2, 0.8, 1.0], 2)

    set_gui_top_down()

    target_idx  = 0
    sim_step    = 0
    stall_steps = 0
    last_pos    = None

    try:
        while True:
            if sim_step % DISPLAY_EVERY_N == 0:
                renderer.render(scene.car)

            pos, orn = p.getBasePositionAndOrientation(scene.car)
            x, y, _  = pos
            _, _, yaw = p.getEulerFromQuaternion(orn)

            final_dist = math.hypot(goal_xy[0] - x, goal_xy[1] - y)
            if final_dist < GOAL_TOL:
                print(f"Reached {selected_spot}.")
                brake_and_stop(scene.car, scene.wheels, scene.steering, renderer)
                break

            if last_pos is not None and math.hypot(x - last_pos[0], y - last_pos[1]) < 0.015:
                stall_steps += STEPS_PER_SLEEP
            else:
                stall_steps = 0
            last_pos = (x, y)

            if stall_steps > 60:
                print(f"Stall detected at ({x:.2f}, {y:.2f}), reversing...")
                for _ in range(80):
                    apply_control(scene.car, scene.wheels, scene.steering, -MIN_SPEED, 0.0)
                    p.stepSimulation()
                stall_steps = 0
                continue

            tx, ty = waypoints[target_idx]
            if math.hypot(tx - x, ty - y) < WAYPOINT_TOL:
                if target_idx < len(waypoints) - 1:
                    target_idx += 1
                    tx, ty = waypoints[target_idx]
                else:
                    tx, ty = goal_xy

            dx, dy = tx - x, ty - y
            local_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

            ld        = max(math.hypot(local_x, local_y), LOOKAHEAD)
            curvature = 2.0 * local_y / (ld * ld)
            steer     = clamp(math.atan(WHEELBASE * curvature), -MAX_STEER, MAX_STEER)

            steer_ratio = abs(steer) / MAX_STEER
            speed_cmd   = BASE_SPEED * (1.0 - 0.55 * steer_ratio)
            speed_cmd   = max(MIN_SPEED, speed_cmd)

            if final_dist < 2.2:
                speed_limit = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * (final_dist / 2.2)
                speed_cmd   = min(speed_cmd, speed_limit)

            apply_control(scene.car, scene.wheels, scene.steering, speed_cmd, steer)

            for _ in range(STEPS_PER_SLEEP):
                p.stepSimulation()
            time.sleep(STEPS_PER_SLEEP / 240.0)

            sim_step += STEPS_PER_SLEEP
            if sim_step % 120 == 0:
                print(
                    f"step={sim_step:5d} "
                    f"pos=({x: .2f}, {y: .2f}) "
                    f"target_idx={target_idx} "
                    f"steer={steer: .3f} "
                    f"speed={speed_cmd: .2f} "
                    f"goal_dist={final_dist: .2f}"
                )

            if sim_step > MAX_STEPS:
                print("Timeout stop.")
                brake_and_stop(scene.car, scene.wheels, scene.steering, renderer)
                break
    finally:
        renderer.close()