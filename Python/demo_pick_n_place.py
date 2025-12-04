import numpy as np
from kinematics import SerialArm
import transforms as tr
from visualization import VizScene

# ----- 1. Define DH and build the arm -----
dh = [
    [0.0, 0.20, 0.0,  np.pi/2],   # J1 base yaw
    [0.0, 0.00, 0.0, -np.pi/2],   # J2 sideways
    [0.0, 0.20, 0.0,  np.pi/2],   # J3 vertical-ish
    [0.0, 0.00, 0.15, 0.0],       # J4 sideways
    [0.0, 0.00, 0.05, 0.0],       # J5 / gripper pitch
]
jt_types = ['r'] * 5

# 50 mm gripper offset along the local x-axis
tip = tr.se3(np.eye(3), np.array([0.05, 0.0, 0.0]))
arm = SerialArm(dh, jt=jt_types, tip=tip)

# ----- 2. Set up the visualizer -----
viz = VizScene()
viz.add_arm(arm, draw_frames=True)

# ----- 3. Define Cartesian pick/place points and solve IK -----
def base_yaw_from_xy(x, y):
    """Rough guess for base joint based on where the target is in XY."""
    return np.arctan2(y, x)

# Some reachable positions (meters) in base frame
# Same (x, y) for "above" and "at" so the motion is vertically aligned.
pick_xy = np.array([0.25,  0.15])
box_xy  = np.array([-0.25, -0.20])

z_above = 0.35   # height above table
z_pick  = 0.15   # height at object
z_place = 0.15   # height at bottom of box

p_above_pick = np.array([pick_xy[0], pick_xy[1], z_above])
p_pick       = np.array([pick_xy[0], pick_xy[1], z_pick])

p_above_box  = np.array([box_xy[0],  box_xy[1],  z_above])
p_place      = np.array([box_xy[0],  box_xy[1],  z_place])

K = np.eye(3) * 2.0  # position gain for IK

def solve_ik_pos(target, q_seed, label=""):
    """Helper to call ik_position with a seed and print a bit of debug info."""
    q_sol, e, iters, success, msg = arm.ik_position(
        target,
        q0=q_seed,
        method='J_T',
        K=K,
        kd=0.001,
        max_iter=500
    )
    print(f"{label} IK -> success={success}, iters={iters}, err={e}, msg={msg}")
    return q_sol

n = arm.n
q_home = np.zeros(n)

# Use base joint to face the pick
q_seed_pick = q_home.copy()
q_seed_pick[0] = base_yaw_from_xy(*pick_xy)

q_above_pick = solve_ik_pos(p_above_pick, q_seed_pick,  label="above_pick")
q_pick       = solve_ik_pos(p_pick,       q_above_pick, label="pick")

# Use base joint to face the box
q_seed_box = q_home.copy()
q_seed_box[0] = base_yaw_from_xy(*box_xy)

q_above_box = solve_ik_pos(p_above_box, q_seed_box,  label="above_box")
q_place     = solve_ik_pos(p_place,     q_above_box, label="place")

# ----- 4. Define joint-space waypoints from the IK solutions -----
waypoints = [
    q_home,
    q_above_pick,
    q_pick,
    q_above_pick,  # lift straight back up
    q_above_box,
    q_place,
    q_above_box,   # lift out of the box
    q_home,
]

# ----- 5. Simple interpolation and animation -----

def interpolate(q_start, q_end, n_steps=100):
    for s in np.linspace(0.0, 1.0, n_steps):
        yield (1.0 - s) * q_start + s * q_end

for i in range(len(waypoints) - 1):
    q_start = waypoints[i]
    q_end   = waypoints[i + 1]

    for q in interpolate(q_start, q_end, n_steps=80):
        viz.update(qs=[q])

viz.hold()
