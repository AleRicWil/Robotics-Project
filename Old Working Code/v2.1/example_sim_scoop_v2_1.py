"""
example_sim_scoop.py
Pick–scoop–carry–dump demo with simulation or hardware control.

- 5-DOF arm (gripper is separate).
- Uses IK to keep the scoop level, then adds a fixed tilt-back while carrying.
"""

import argparse
import time

import numpy as np
import serial

from kinematics_v2_1 import SerialArm
import transforms_v2_1 as tr
from visualization_v2_1 import VizScene
from Robot_Interpreter_v2_1 import RobotInterpreter, DEFAULT_RPS


# ---------------------------------------------------------------------------
# Constants & configuration
# ---------------------------------------------------------------------------

GRIPPER_OPEN = 0.0      # deg, adjust for hardware calibration
GRIPPER_CLOSE = 90.0    # deg, adjust for hardware
DEFAULT_SPEED = float(DEFAULT_RPS)  # RPS for motions
SERIAL_BAUD = 115200
N_STEPS = 80            # interpolation steps per segment

# How much to tilt the scoop back while carrying (deg)
CARRY_TILT_DEG = 20.0
CARRY_TILT = np.radians(CARRY_TILT_DEG)

# DH parameters for the arm (5-DOF positioning, gripper separate)
dh = [
    [0.0, 0.202, 0.0,  np.pi / 2],      # J1 base 
    [0.0, 0.00, 0.0, -np.pi / 2],       # J2 wrist pitch
    [0.0, 0.283082, 0.0,  np.pi / 2],   # J3 wrist roll
    [np.pi / 2, 0.00, 0.295085, np.pi],       # J4 elbow
    [0.0, 0.00, 0.05, 0.0],             # J5 hand
]
jt_types = ['r'] * 5

# Joint limits in degrees (then converted to radians)
JOINT_LIMITS_DEG = [
    [-180, 180],        # J1 base ±180°
    [-135, 135],        # J2
    [-180, 180],        # J3
    [-130, 130],        # J4 servo X
    [-135, 135],        # J5 servo Y
]
JOINT_LIMITS_RAD = np.radians(JOINT_LIMITS_DEG)

# Gripper offset along x-axis (tool frame)
tip = tr.se3(np.eye(3), np.array([0.05, 0.0, 0.0]))

# Toggle for very verbose tip-axis printing
DEBUG_TIP_AXIS = False


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def base_yaw_from_xy(x: float, y: float) -> float:
    """Estimate base joint angle from target XY position."""
    return np.arctan2(y, x)


def level_last_joint(
    arm: SerialArm,
    q_in: np.ndarray,
    tip_axis_index: int = 1,  # 0=x, 1=y, 2=z – “up” axis in tool frame
    max_iter: int = 40,
    step: float = 0.5,
    tol: float = 1e-3,
) -> np.ndarray:
    """
    Take a configuration q_in that already roughly hits the desired position,
    and adjust ONLY the last joint (q[4]) so the chosen tip axis is as vertical
    as possible (aligned with world z).
    """
    q = np.array(q_in, dtype=float)
    z_world = np.array([0.0, 0.0, 1.0])

    def tip_up_vec(q_):
        T = arm.fk(q_, tip=True)
        R = T[0:3, 0:3]
        return R[:, tip_axis_index]

    for _ in range(max_iter):
        up = tip_up_vec(q)

        # Horizontal component = how much the "up" vector is NOT vertical
        horiz = up.copy()
        horiz[2] = 0.0
        err = np.linalg.norm(horiz)
        if err < tol:
            break

        # Finite-difference gradient wrt q5
        dq = 1e-3
        q_perturb = q.copy()
        q_perturb[4] += dq
        up_p = tip_up_vec(q_perturb)
        horiz_p = up_p.copy()
        horiz_p[2] = 0.0
        err_p = np.linalg.norm(horiz_p)

        grad = (err_p - err) / dq  # d(err)/dq5

        # Gradient-descent step on q5
        q[4] -= step * grad

        # Respect joint limit on J5
        if arm.qlim is not None:
            q[4] = np.clip(q[4], arm.qlim[4][0], arm.qlim[4][1])

    return q


def solve_ik_pos(
    arm: SerialArm,
    target: np.ndarray,
    q_seed: np.ndarray,
    label: str = "",
    K: np.ndarray | None = None,
) -> np.ndarray:
    """Solve IK for position using the arm's method and print debug summary."""
    if K is None:
        K = np.eye(3) * 2.0

    q_sol, e, iters, success, msg = arm.ik_position(
        target, q0=q_seed, method="J_T", K=K, kd=0.001, max_iter=500
    )
    print(f"{label} IK -> success={success}, iters={iters}, err={e}, msg={msg}")
    return q_sol


def solve_ik_level_tip(
    arm: SerialArm,
    target_p: np.ndarray,
    q_seed: np.ndarray,
    label: str = "",
) -> np.ndarray:
    """
    2–stage IK:
      1) Use position-only IK (all joints) to reach target_p.
      2) Then adjust ONLY the last joint to keep the tip level.
    """
    # 1) Position-only IK
    q_pos = solve_ik_pos(arm, target_p, q_seed, label=label)

    # 2) Level the last joint
    q_level = level_last_joint(arm, q_pos)

    # Optional: print how level it is
    T = arm.fk(q_level, tip=True)
    R = T[0:3, 0:3]
    tip_up = R[:, 1]  # assuming tool Y is the “up” axis
    print(f"{label} final tip_up:", tip_up)

    return q_level


def interpolate(q_start: np.ndarray, q_end: np.ndarray, n_steps: int = N_STEPS):
    """Generator yielding linearly interpolated joint configurations."""
    for s in np.linspace(0.0, 1.0, n_steps):
        yield (1.0 - s) * q_start + s * q_end


def with_carry_tilt(q: np.ndarray, tilt: float = CARRY_TILT) -> np.ndarray:
    """Return a copy of q with the last joint tilted back by `tilt` radians."""
    q_tilt = np.array(q, dtype=float).copy()
    q_tilt[4] -= tilt  # J5 is index 4
    return q_tilt


# ---------------------------------------------------------------------------
# Hardware helpers
# ---------------------------------------------------------------------------

def wait_all_idle(interpreter: RobotInterpreter):
    """Block until all motors/servos report idle."""
    while not (
        interpreter.b_idle
        and interpreter.r_idle
        and interpreter.l_idle
        and interpreter.x_idle
        and interpreter.y_idle
        and interpreter.z_idle
    ):
        time.sleep(0.005)


def send_to_hardware(
    interpreter: RobotInterpreter,
    target_q_deg: np.ndarray,
    current_q_deg: np.ndarray,
    speed: float,
    gripper_angle: float | None = None,
):
    """
    Send commands to hardware.
      - Steppers (1-3): relative displacements.
      - Servos (4-5): absolute angles.
      - Gripper (6): optional absolute angle.
    """
    deltas_deg = target_q_deg - current_q_deg

    # Joint 1: base stepper, relative
    interpreter.send_joint_command(1, deltas_deg[0], speed)

    # Joints 2-3: wrist steppers, use composite for synchronization, relative
    interpreter.send_composite_command(deltas_deg[1], speed, deltas_deg[2], speed)

    # Joints 4-5: servos, absolute
    interpreter.send_joint_command(4, target_q_deg[3], speed)
    interpreter.send_joint_command(5, target_q_deg[4], speed)

    if gripper_angle is not None:
        interpreter.send_joint_command(6, gripper_angle, speed)

    wait_all_idle(interpreter)


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------

def plan_waypoints(arm: SerialArm) -> list[np.ndarray]:
    """
    Compute key poses/waypoints for:
      home → above scoop → scoop path → carry to bucket (tilted back) →
      dump → return home.
    """
    # Cartesian positions (m)
    pick_xy = np.array([0.5, 0.0])
    box_xy = np.array([-0.2, -0.2])
    z_above = 0.40
    z_table = 0.00          # where the scoop scrapes
    z_carry = 0.50          # level-carry height above table

    # Positions along the scoop & carry path
    p_above_scoop = np.array([pick_xy[0], pick_xy[1], z_carry])
    p_scoop_start = np.array([pick_xy[0], pick_xy[1], z_table])
    p_scoop_end   = np.array([pick_xy[0], pick_xy[1], z_table])
    p_above_bucket = np.array([box_xy[0], box_xy[1], z_carry])
    p_drop         = np.array([box_xy[0], box_xy[1], z_above])

    q_home = np.zeros(arm.n)

    # Seed yaw toward the scoop area
    q_seed_scoop = q_home.copy()
    q_seed_scoop[0] = base_yaw_from_xy(*pick_xy)

    # Level-tip IK for all poses where we want the tool to be flat
    q_above_scoop = solve_ik_level_tip(arm, p_above_scoop, q_seed_scoop, "above_scoop")
    q_scoop_start = solve_ik_level_tip(arm, p_scoop_start, q_above_scoop, "scoop_start")
    q_scoop_end   = solve_ik_level_tip(arm, p_scoop_end,   q_scoop_start, "scoop_end")

    # Seed yaw toward the bucket
    q_seed_bucket = q_home.copy()
    q_seed_bucket[0] = base_yaw_from_xy(*box_xy)

    q_above_bucket = solve_ik_level_tip(arm, p_above_bucket, q_seed_bucket, "above_bucket")
    q_drop         = solve_ik_level_tip(arm, p_drop,         q_above_bucket, "drop_level")

    # Tilted-back versions for carrying the scoop
    q_above_scoop_carry  = with_carry_tilt(q_above_scoop)
    q_above_bucket_carry = with_carry_tilt(q_above_bucket)
    q_drop_carry         = with_carry_tilt(q_drop)

    # Dump pose: intentionally break the level constraint
    q_dump = q_drop_carry.copy()
    q_dump[4] -= np.radians(-80.0)  # tip forward to pour out the scoop

    waypoints = [
        q_home,
        q_above_scoop,
        q_scoop_start,
        q_scoop_end,          # object is now in the scoop (still level)
        q_above_scoop_carry,  # lift with tilt-back
        q_above_bucket_carry, # carry over bucket, still tilted back
        q_drop_carry,         # descend over bucket, still tilted back
        q_dump,               # tip forward to dump
        q_drop_carry,         # tip back to carry tilt
        q_above_bucket_carry,
        q_home,
    ]
    return waypoints


def main(sim_only: bool = True, port: str | None = None):
    """Main function for demo, handles sim or hardware mode."""
    arm = SerialArm(dh, jt=jt_types, tip=tip, joint_limits=JOINT_LIMITS_RAD)

    viz = VizScene()
    viz.add_arm(arm, draw_frames=True)

    interpreter = None
    ser = None
    if not sim_only:
        if port is None:
            raise ValueError("Serial port required for hardware mode.")
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
        interpreter = RobotInterpreter(ser)
        print(f"Connected to hardware on {port}.")

    waypoints = plan_waypoints(arm)

    current_q = np.zeros(arm.n)       # radians
    current_q_deg = np.zeros(arm.n)   # degrees
    gripper_current = GRIPPER_OPEN    # not yet used but ready for extension

    for q_start, q_end in zip(waypoints[:-1], waypoints[1:]):
        for q in interpolate(q_start, q_end):
            viz.update(qs=[q])

            if DEBUG_TIP_AXIS:
                T_tip = arm.fk(q, tip=True)
                R_tip = T_tip[0:3, 0:3]
                print("tip up axis:", R_tip[:, 1])

            if not sim_only and interpreter is not None:
                target_q_deg = tr.rad_to_deg(q)
                send_to_hardware(interpreter, target_q_deg, current_q_deg, DEFAULT_SPEED)
                current_q_deg = target_q_deg

            current_q = q
            time.sleep(1.0 / 60)  # ~60 FPS viz

    viz.hold()

    if ser is not None:
        ser.close()
        print("Hardware connection closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Pick-and-scoop demo with simulation or hardware."
    )
    parser.add_argument(
        "--sim_only",
        action="store_true",
        help="Run in simulation only (default if flag present).",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port for hardware (e.g., COM3 or /dev/ttyUSB0).",
    )
    args = parser.parse_args()

    # If no port is given, force sim_only=True regardless of flag mistakes
    sim_mode = args.sim_only or (args.port is None)
    main(sim_only=sim_mode, port=args.port)
