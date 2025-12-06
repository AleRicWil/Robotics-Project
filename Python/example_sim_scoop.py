# example_sim_scoop.py - Demonstrates pick-and-place with simulation or hardware control
# v2.0: Added sim_only flag for pure simulation or hardware mode with visualization.
#       In hardware mode, connects to serial, uses RobotInterpreter to send commands.
#       Assumes arm starts at q_home=zeros; tracks software current_q.
#       Joint angles converted to degrees before sending (as hardware expects).
#       Gripper handled separately as servo 6 (absolute angles).
#       Waits for motions to complete by polling idle states.
#       Follows industry conventions: modular functions, error handling, comments.

import numpy as np
from kinematics_v2_0 import SerialArm
import transforms_v2_0 as tr
from visualization_v2_0 import VizScene
import time
import argparse
import serial
from Robot_Interpreter_v2_0 import RobotInterpreter, DEFAULT_RPS

# Constants
GRIPPER_OPEN = 0.0   # Degrees, adjust based on hardware calibration
GRIPPER_CLOSE = 90.0 # Degrees, adjust based on hardware
DEFAULT_SPEED = float(DEFAULT_RPS)  # RPS for motions
SERIAL_BAUD = 115200
N_STEPS = 80  # Interpolation steps per segment

# DH parameters for the arm (5-DOF positioning, gripper separate)
dh = [
    [0.0, 0.20, 0.0,  np.pi/2],   # J1 base yaw
    [0.0, 0.00, 0.0, -np.pi/2],   # J2 sideways
    [0.0, 0.20, 0.0,  np.pi/2],   # J3 vertical-ish
    [0.0, 0.00, 0.15, 0.0],       # J4 sideways
    [0.0, 0.00, 0.05, 0.0],       # J5 pitch
]
jt_types = ['r'] * 5
# Real measured limits in radians (example – adjust to your robot!)
JOINT_LIMITS = [
    [-180, 180],        # J1 base ±180°
    [-135, 135],        # J2 
    [-180, 180],        # J3
    [-120, 120],        # J4 servo X
    [-120, 120],        # J5 servo Y
]
JOINT_LIMITS = np.radians(JOINT_LIMITS)

# Gripper offset along x-axis
tip = tr.se3(np.eye(3), np.array([0.05, 0.0, 0.0]))

def base_yaw_from_xy(x, y):
    """Estimates base joint angle from target XY position."""
    return np.arctan2(y, x)

def level_last_joint(
    arm,
    q_in,
    tip_axis_index=1,   # 0=x, 1=y, 2=z – adjust if needed
    max_iter=40,
    step=0.5,
    tol=1e-3,
):
    """
    Take a configuration q_in that already roughly hits the desired position,
    and adjust ONLY the last joint (q[4]) so the chosen tip axis is as vertical
    as possible (i.e., aligned with world z).
    """
    q = np.array(q_in, dtype=float)
    z_world = np.array([0.0, 0.0, 1.0])

    def tip_up_vec(q_):
        T = arm.fk(q_, tip=True)
        R = T[0:3, 0:3]
        return R[:, tip_axis_index]  # e.g. R[:,1] if “up” is tool y-axis

    for _ in range(max_iter):
        up = tip_up_vec(q)

        # Horizontal component = how much the "up" vector is *not* vertical
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

        # Gradient descent step on q5
        q[4] -= step * grad

        # Respect joint limit on J5
        if arm.qlim is not None:
            q[4] = np.clip(q[4], arm.qlim[4][0], arm.qlim[4][1])

    return q

def solve_ik_pos(arm, target, q_seed, label="", K=np.eye(3)*2.0):
    """Solves IK for position using arm's method."""
    q_sol, e, iters, success, msg = arm.ik_position(
        target, q0=q_seed, method='J_T', K=K, kd=0.001, max_iter=500
    )
    print(f"{label} IK -> success={success}, iters={iters}, err={e}, msg={msg}")
    return q_sol

def solve_ik_level_tip(
    arm,
    target_p,
    q_seed,
    label="",
):
    """
    2–stage IK:

    1) Use position-only IK (all joints) to reach target_p.
    2) Then adjust ONLY the last joint to keep the tip level.
    """
    # 1) Position-only IK
    q_pos = solve_ik_pos(arm, target_p, q_seed, label=label)

    # 2) Level the last joint
    q_level = level_last_joint(arm, q_pos)

    # Optional: print/debug how level it is
    T = arm.fk(q_level, tip=True)
    R = T[0:3, 0:3]
    tip_up = R[:, 1]   # or whatever axis you chose
    print(f"{label} final tip_up:", tip_up)

    return q_level


def interpolate(q_start, q_end, n_steps=N_STEPS):
    """Yields interpolated joint configurations."""
    for s in np.linspace(0.0, 1.0, n_steps):
        yield (1.0 - s) * q_start + s * q_end

def wait_all_idle(interpreter):
    """Polls idle states until all motors/servos are stopped."""
    while not (interpreter.b_idle and interpreter.r_idle and interpreter.l_idle and
               interpreter.a_idle and interpreter.d_idle and interpreter.c_idle):
        time.sleep(0.005)

def send_to_hardware(interpreter, target_q_deg, current_q_deg, speed, gripper_angle=None):
    """
    Sends commands to hardware.
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
    
    # Wait for completion
    wait_all_idle(interpreter)

def main(sim_only=True, port=None):
    """Main function for demo, handles sim or hardware mode."""
    arm = SerialArm(dh, jt=jt_types, tip=tip, joint_limits=JOINT_LIMITS)
    viz = VizScene()
    viz.add_arm(arm, draw_frames=True)
    
    interpreter = None
    if not sim_only:
        if port is None:
            raise ValueError("Serial port required for hardware mode.")
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
        interpreter = RobotInterpreter(ser)
        print(f"Connected to hardware on {port}.")
    
    # Define positions (meters)
    pick_xy = np.array([0.25, 0.15])
    box_xy  = np.array([-0.25, -0.20])
    z_above = 0.35
    z_table = 0.15          # where the scoop scrapes
    z_carry = 0.25          # level-carry height above table

    # Positions along the scoop & carry path
    # Approach above the scoop
    p_above_scoop = np.array([pick_xy[0] - 0.10, pick_xy[1], z_carry])

    # Start scooping a bit behind the object
    p_scoop_start = np.array([pick_xy[0] - 0.10, pick_xy[1], z_table])

    # End scooping slightly past the object
    p_scoop_end   = np.array([pick_xy[0] + 0.05, pick_xy[1], z_table])

    # Carry position above bucket
    p_above_bucket = np.array([box_xy[0], box_xy[1], z_carry])
    p_drop         = np.array([box_xy[0], box_xy[1], z_table])

    q_home = np.zeros(arm.n)

    # Seed yaw toward the scoop area
    q_seed_scoop = q_home.copy()
    q_seed_scoop[0] = base_yaw_from_xy(*pick_xy)

    # Level-tip IK for all poses where we want the tool to be flat
    q_above_scoop = solve_ik_level_tip(arm, p_above_scoop, q_seed_scoop, "above_scoop")
    q_scoop_start = solve_ik_level_tip(arm, p_scoop_start, q_above_scoop, "scoop_start")
    q_scoop_end   = solve_ik_level_tip(arm, p_scoop_end,   q_scoop_start, "scoop_end")

    # Now move to bucket, still level
    q_seed_bucket = q_home.copy()
    q_seed_bucket[0] = base_yaw_from_xy(*box_xy)

    # Use a seed that already has the base yaw pointing at the bucket
    q_above_bucket = solve_ik_level_tip(arm, p_above_bucket, q_seed_bucket, "above_bucket")

    # From there, drop straight down, still level
    q_drop         = solve_ik_level_tip(arm, p_drop, q_above_bucket, "drop_level")

    # Finally, a 'dump' pose that intentionally breaks the level constraint:
    q_dump = q_drop.copy()
    # Tilt the last joint down to pour out the scoop
    q_dump[4] += np.radians(-60.0)
    
    waypoints = [
        q_home,
        q_above_scoop,
        q_scoop_start,
        q_scoop_end,      # object is now in the scoop
        q_above_scoop,    # lift while level
        q_above_bucket,   # move over bucket, level
        q_drop,           # descend, still level
        q_dump,           # tip to dump
        q_drop,           # tip back upright
        q_above_bucket,
        q_home,
    ]
    
    current_q = np.zeros(arm.n)  # Track in radians
    current_q_deg = np.zeros(arm.n)  # Track in degrees
    gripper_current = GRIPPER_OPEN


    for i in range(len(waypoints) - 1):
        q_start = waypoints[i]
        q_end = waypoints[i + 1]
        
        for q in interpolate(q_start, q_end):
            viz.update(qs=[q])
            T_tip = arm.fk(q, tip=True)
            R_tip = T_tip[0:3, 0:3]
            print("tip up axis:", R_tip[:, 2])
            if not sim_only:
                target_q_deg = tr.rad_to_deg(q)
                send_to_hardware(interpreter, target_q_deg, current_q_deg, DEFAULT_SPEED)
                current_q_deg = target_q_deg
            current_q = q
            time.sleep(1.0 / 60)  # ~60 FPS viz

    viz.hold()
    
    if not sim_only:
        ser.close()
        print("Hardware connection closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pick-and-place demo with sim or hardware.")
    parser.add_argument('--sim_only', default=True, help="Run in simulation only (default).")
    parser.add_argument('--port', type=str, default=None, help="Serial port for hardware (e.g., /dev/ttyUSB0).")
    args = parser.parse_args()
    main(sim_only=args.sim_only, port=args.port)