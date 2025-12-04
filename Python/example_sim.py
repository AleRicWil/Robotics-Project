# demo_pick_n_place_v2_0.py - Demonstrates pick-and-place with simulation or hardware control
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

# Gripper offset along x-axis
tip = tr.se3(np.eye(3), np.array([0.05, 0.0, 0.0]))

def base_yaw_from_xy(x, y):
    """Estimates base joint angle from target XY position."""
    return np.arctan2(y, x)

def solve_ik_pos(arm, target, q_seed, label="", K=np.eye(3)*2.0):
    """Solves IK for position using arm's method."""
    q_sol, e, iters, success, msg = arm.ik_position(
        target, q0=q_seed, method='J_T', K=K, kd=0.001, max_iter=500
    )
    print(f"{label} IK -> success={success}, iters={iters}, err={e}, msg={msg}")
    return q_sol

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
    arm = SerialArm(dh, jt=jt_types, tip=tip)
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
    box_xy = np.array([-0.25, -0.20])
    z_above = 0.35
    z_pick = 0.15
    z_place = 0.15
    
    p_above_pick = np.array([pick_xy[0], pick_xy[1], z_above])
    p_pick = np.array([pick_xy[0], pick_xy[1], z_pick])
    p_above_box = np.array([box_xy[0], box_xy[1], z_above])
    p_place = np.array([box_xy[0], box_xy[1], z_place])
    
    q_home = np.zeros(arm.n)
    
    # IK seeds
    q_seed_pick = q_home.copy()
    q_seed_pick[0] = base_yaw_from_xy(*pick_xy)
    q_above_pick = solve_ik_pos(arm, p_above_pick, q_seed_pick, "above_pick")
    
    q_pick = solve_ik_pos(arm, p_pick, q_above_pick, "pick")
    
    q_seed_box = q_home.copy()
    q_seed_box[0] = base_yaw_from_xy(*box_xy)
    q_above_box = solve_ik_pos(arm, p_above_box, q_seed_box, "above_box")
    q_place = solve_ik_pos(arm, p_place, q_above_box, "place")
    
    # Waypoints (5-DOF)
    waypoints = [
        q_home,
        q_above_pick,
        q_pick,        # Close gripper after
        q_above_pick,
        q_above_box,
        q_place,       # Open gripper after
        q_above_box,
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
            if not sim_only:
                target_q_deg = tr.rad_to_deg(q)
                send_to_hardware(interpreter, target_q_deg, current_q_deg, DEFAULT_SPEED)
                current_q_deg = target_q_deg
            current_q = q
            time.sleep(1.0 / 60)  # ~60 FPS viz
        
        # Handle gripper at specific waypoints
        gripper_target = None
        if np.allclose(q_end, q_pick):
            gripper_target = GRIPPER_CLOSE
        elif np.allclose(q_end, q_place):
            gripper_target = GRIPPER_OPEN
        
        if gripper_target is not None and not sim_only:
            send_to_hardware(interpreter, current_q_deg, current_q_deg, DEFAULT_SPEED, gripper_target)
            gripper_current = gripper_target
            print(f"Gripper set to {gripper_target}°.")
    
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