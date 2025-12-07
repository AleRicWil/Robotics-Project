# Robot_Interpreter_v2_1.py
# Interpreter for robot arm control, handling both steppers and servos.
# v2.0: Added joint limit enforcement and software tracking of current angles for all joints.
#       - Accepts joint_limits as a list of [min, max] pairs in degrees (similar to kinematics but list for simplicity).
#       - Defaults to safe limits if none provided; customizable per joint.
#       - Tracks current_angle for each joint independently in software to enforce limits without hardware feedback.
#       - For steppers (relative): Accumulates displacements; validates proposed angle before sending.
#       - For servos (absolute): Sets to target after validation; hardware query available but tracking decouples for safety.
#       - Added set_current_zero for manual zeroing via UI.
#       - get_tracked_angle for UI display; prefers tracked value for consistency.
#       - Concept: Decouples safety enforcement from IK; follows industry practice for open-loop control with virtual state.

import threading
import queue
import time
import math

# Constants (Steppers, unchanged)
G_B = 110.0 / 21.0   # Gear ratio for base stepper
G_RL = 7.8  # Gear ratio for wrist steppers
DEG_TOLERANCE = 1e-6
MAX_MOTOR_RPS = 1.00  # Max RPS for steppers
ACCEL = 1.0

# Constants (Servos, from specs)
MAX_SERVO_ANGLE = 270.0
MAX_SERVO_RPS = 1.0  # Conservative max, derated from specs (0.14sec/60° ~1.19 RPS at 7.2V)

DEFAULT_RPS = '0.05'

# Default joint limits in degrees (if none provided)
# Concept: Based on hardware calibration; prevents damage by default. User can override.
DEFAULT_JOINT_LIMITS_DEG = [
    [-180, 180],  # Joint 1: Base yaw
    [-135, 135],  # Joint 2: Wrist differential
    [-180, 180],  # Joint 3: Wrist common
    [-120, 120],  # Joint 4: Servo X
    [-120, 120],  # Joint 5: Servo Y
    [0, 90]       # Joint 6: Gripper (Servo Z, open/close)
]

def Motor_Map(joint: int, displacement: float, speed: float) -> list[str]:
    """
    Maps joint commands to motor/servo commands based on joint ID.
    For steppers (1-3): computes displacements with gear ratios.
    For servos (4-6): direct absolute angles, clamped in Arduino. Servo 6 controls gripper, not a real joint.
    Concept: Abstraction layer for hardware mapping, allowing easy reconfiguration.
    """
    if joint not in range(1, 7):
        raise ValueError("Invalid joint (1-6).")
    if speed <= 0.0:
        raise ValueError("Speed must be positive.")

    commands = []

    if joint == 1:  # Base stepper (B)
        motor_deg = displacement * G_B
        motor_rps = speed * G_B
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg) > DEG_TOLERANCE:
            commands.append(f'B {motor_deg:.2f} {motor_rps:.3f}')

    elif joint == 2:  # Wrist steppers (R, L differential)
        motor_deg_R = displacement * G_RL
        motor_deg_L = -displacement * G_RL
        motor_rps = speed * G_RL
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg_R) > DEG_TOLERANCE:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps:.3f}')
        if abs(motor_deg_L) > DEG_TOLERANCE:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps:.3f}')

    elif joint == 3:  # Wrist steppers (R, L common)
        motor_deg_R = -displacement * G_RL
        motor_deg_L = -displacement * G_RL
        motor_rps = speed * G_RL
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg_R) > DEG_TOLERANCE:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps:.3f}')
        if abs(motor_deg_L) > DEG_TOLERANCE:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps:.3f}')

    elif joint == 4:  # Servo X
        servo_rps = min(speed, MAX_SERVO_RPS)
        commands.append(f'X {displacement:.2f} {servo_rps:.3f}') # Actually does absolute angle. Just reusing argument name for simplicity

    elif joint == 5:  # Servo Y
        servo_rps = min(speed, MAX_SERVO_RPS)
        commands.append(f'Y {displacement:.2f} {servo_rps:.3f}')

    elif joint == 6:  # Servo Z
        servo_rps = min(speed, MAX_SERVO_RPS)
        commands.append(f'Z {displacement:.2f} {servo_rps:.3f}')

    return commands

def compute_min_time(d, v_max, accel=ACCEL):
    """
    Computes minimum time for stepper motion with trapezoidal profile.
    Concept: Ensures feasible speeds; prevents impossible commands.
    """
    if d <= DEG_TOLERANCE:
        return 0.0
    v_peak = math.sqrt(4.0 * d * accel / 3.0)
    if v_peak <= v_max:
        return math.sqrt(3.0 * d / accel)
    else:
        return d / v_max + (3.0 / 4.0) * v_max / accel

def solve_v(t, d, accel=ACCEL):
    """
    Solves for velocity given time and distance in trapezoidal profile.
    Concept: Used for composite motions to synchronize axes.
    """
    if d <= DEG_TOLERANCE:
        return 0.0
    min_t = math.sqrt(3.0 * d / accel)
    # Relax comparison for FP precision (e.g., allow t slightly below due to rounding)
    if t < min_t - 1e-8:  # Small epsilon to avoid false positives
        raise ValueError(f"Impossible time {t} < min {min_t} for d={d}")
    k = (3.0 / 4.0) / accel
    disc = t**2 - 4.0 * k * d
    # Clamp disc for FP errors (if slightly negative, treat as zero)
    if disc < 0:
        if abs(disc) < 1e-6:  # Tolerance; adjust if needed based on your floats
            disc = 0.0
        else:
            raise ValueError(f"Negative discriminant {disc} for t={t}, d={d}")
    v = (t - math.sqrt(disc)) / (2.0 * k)
    return v

class RobotInterpreter:
    def __init__(self, ser, joint_limits=None):
        self.ser = ser

        # Queues for independent control
        self.base_queue = queue.Queue(maxsize=100)   # For base stepper (B)
        self.wrist_queue = queue.Queue(maxsize=100)  # For wrist steppers (R, L)
        self.servo_queue = queue.Queue(maxsize=100)  # For servos (X, Y, Z)

        # Idle tracking for all
        self.r_idle = True
        self.l_idle = True
        self.b_idle = True
        self.x_idle = True
        self.y_idle = True
        self.z_idle = True

        self.lock = threading.Lock()

        # Start threads
        threading.Thread(target=self._monitor_serial, daemon=True).start()
        threading.Thread(target=self._base_worker, daemon=True).start()
        threading.Thread(target=self._wrist_worker, daemon=True).start()
        threading.Thread(target=self._servo_worker, daemon=True).start()

        # Joint limits (in degrees); list of [min, max] for joints 1-6
        # Concept: Centralized for easy config; defaults prevent damage if omitted.
        self.joint_limits = joint_limits if joint_limits else DEFAULT_JOINT_LIMITS_DEG

        # Tracked current angles (in degrees) for all joints
        # Concept: Virtual state for open-loop safety; initialized to zero (user must zero manually).
        self.current_angles = [0.0] * 6  # Indices 0-5 for joints 1-6

    def _monitor_serial(self):
        """
        Monitors serial for 'stopped' messages to update idle states.
        Concept: Non-blocking status updates; enables queue processing.
        """
        while True:
            with self.lock:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if not line:
                            continue
                        
                        if 'R motor stopped.' in line:
                            self.r_idle = True
                        elif 'L motor stopped.' in line:
                            self.l_idle = True
                        elif 'B motor stopped.' in line:
                            self.b_idle = True
                        elif 'X servo stopped.' in line:
                            self.x_idle = True
                        elif 'Y servo stopped.' in line:
                            self.y_idle = True
                        elif 'Z servo stopped.' in line:
                            self.z_idle = True
                    except Exception:
                        pass
                time.sleep(0.001)

    def _base_worker(self):
        """
        Processes base queue when idle.
        Concept: Dedicated thread for independent axis control.
        """
        while True:
            if self.b_idle and not self.base_queue.empty():
                commands = self.base_queue.get()
                self._send_commands(commands)
            time.sleep(0.001)

    def _wrist_worker(self):
        """
        Processes wrist queue when both R and L idle.
        """
        while True:
            if (self.r_idle and self.l_idle) and not self.wrist_queue.empty():
                commands = self.wrist_queue.get()
                self._send_commands(commands)
            time.sleep(0.001)

    def _servo_worker(self):
        """
        Processes servo queue; servos can move independently.
        Concept: Single queue for all servos; idle per servo but process if any ready.
        """
        while True:
            if not self.servo_queue.empty():
                commands = self.servo_queue.get()
                self._send_commands(commands)
            time.sleep(0.001)

    def send_joint_command(self, joint: int, displacement: float, speed: float):
        """
        Sends command for single joint; selects queue based on joint type.
        Validates against limits and clamps if needed before sending.
        For servos, displacement is absolute angle.
        Updates tracked current_angle after successful send.
        Returns True if started immediately.
        """
        idx = joint - 1
        lim_min, lim_max = self.joint_limits[idx]

        if joint in [1, 2, 3]:  # Steppers: relative displacement
            proposed_angle = self.current_angles[idx] + displacement
            if proposed_angle < lim_min:
                displacement = lim_min - self.current_angles[idx]
                print(f"Joint {joint} clamped to min limit {lim_min}°")
            elif proposed_angle > lim_max:
                displacement = lim_max - self.current_angles[idx]
                print(f"Joint {joint} clamped to max limit {lim_max}°")

        elif joint in [4, 5, 6]:  # Servos: absolute target
            proposed_angle = displacement
            if proposed_angle < lim_min:
                displacement = lim_min
                print(f"Joint {joint} clamped to min limit {lim_min}°")
            elif proposed_angle > lim_max:
                displacement = lim_max
                print(f"Joint {joint} clamped to max limit {lim_max}°")

        commands = Motor_Map(joint, displacement, speed)
        if not commands:
            return True

        if joint == 1:
            q = self.base_queue
            immediate = self.b_idle
        elif joint in [2, 3]:
            q = self.wrist_queue
            immediate = self.r_idle and self.l_idle
        elif joint in [4, 5, 6]:
            q = self.servo_queue
            immediate = True  # Servos independent
        else:
            return False

        with self.lock:
            try:
                q.put(commands, block=False)
                # Update tracked angle after queuing (assumes command will execute)
                if joint in [1, 2, 3]:
                    self.current_angles[idx] += displacement
                else:
                    self.current_angles[idx] = displacement
                return immediate and q.empty()  # Approximate immediate start
            except queue.Full:
                print("Queue full; command dropped.")
                return False

    def send_composite_command(self, disp2: float, speed2: float, disp3: float, speed3: float):
        """
        Sends synchronized command for wrist joints 2 and 3 (steppers only).
        Validates both against limits; clamps if needed.
        Computes common time for coordination.
        """
        if speed2 <= 0.0 or speed3 <= 0.0:
            raise ValueError("Speeds must be positive.")

        # Validate and clamp for joint 2
        idx2 = 1  # Joint 2
        lim_min2, lim_max2 = self.joint_limits[idx2]
        proposed2 = self.current_angles[idx2] + disp2
        if proposed2 < lim_min2:
            disp2 = lim_min2 - self.current_angles[idx2]
            print(f"Joint 2 clamped to min limit {lim_min2}°")
        elif proposed2 > lim_max2:
            disp2 = lim_max2 - self.current_angles[idx2]
            print(f"Joint 2 clamped to max limit {lim_max2}°")

        # Validate and clamp for joint 3
        idx3 = 2  # Joint 3
        lim_min3, lim_max3 = self.joint_limits[idx3]
        proposed3 = self.current_angles[idx3] + disp3
        if proposed3 < lim_min3:
            disp3 = lim_min3 - self.current_angles[idx3]
            print(f"Joint 3 clamped to min limit {lim_min3}°")
        elif proposed3 > lim_max3:
            disp3 = lim_max3 - self.current_angles[idx3]
            print(f"Joint 3 clamped to max limit {lim_max3}°")

        rev2 = abs(disp2) / 360.0
        rev3 = abs(disp3) / 360.0
        if rev2 == 0 and rev3 == 0:
            return True

        max_joint_rps = MAX_MOTOR_RPS / G_RL
        speed2 = min(speed2, max_joint_rps)
        speed3 = min(speed3, max_joint_rps)

        times = []
        if rev2 > 0:
            times.append(rev2 / speed2)
        if rev3 > 0:
            times.append(rev3 / speed3)
        quick_time = max(times) if times else 0.0

        motor_rev_R = abs(disp2 * G_RL - disp3 * G_RL) / 360.0
        motor_rev_L = abs(-disp2 * G_RL - disp3 * G_RL) / 360.0

        min_t_R = compute_min_time(motor_rev_R, MAX_MOTOR_RPS)
        min_t_L = compute_min_time(motor_rev_L, MAX_MOTOR_RPS)
        min_common_t = max(min_t_R, min_t_L)
        common_time = max(quick_time, min_common_t)

        motor_rps_R = solve_v(common_time, motor_rev_R) if motor_rev_R > DEG_TOLERANCE else 0.0
        motor_rps_L = solve_v(common_time, motor_rev_L) if motor_rev_L > DEG_TOLERANCE else 0.0

        motor_deg_R = (disp2 * G_RL) - (disp3 * G_RL)
        motor_deg_L = (-disp2 * G_RL) - (disp3 * G_RL)

        commands = []

        r_moving = abs(motor_deg_R) > DEG_TOLERANCE
        l_moving = abs(motor_deg_L) > DEG_TOLERANCE

        if r_moving:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps_R:.3f}')

        if l_moving:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps_L:.3f}')

        with self.lock:
            try:
                was_empty = self.wrist_queue.empty()
                self.wrist_queue.put(commands, block=False)
                # Update tracked angles
                self.current_angles[idx2] += disp2
                self.current_angles[idx3] += disp3
                return was_empty and self.r_idle and self.l_idle
            except queue.Full:
                print("Wrist queue full")
                return False

    def enable_hold(self):
        """
        Sends 'H' command to enable persistent hold mode on steppers.
        Concept: Overrides auto-disable when idle; useful for maintaining torque without motion.
        """
        self._send_commands(['H', 'H'])

    def disable_hold(self):
        """
        Sends 'R' command to disable persistent hold mode on steppers.
        Concept: Reverts to normal global enable/disable based on activity.
        """
        self._send_commands(['R', 'R'])

    def _send_commands(self, commands: list[str]):
        """
        Sends list of commands over serial with small delay.
        Concept: Prevents buffer overflow in serial communication.
        """
        for cmd in commands:
            print(cmd)
            with self.lock:
                self.ser.write((cmd + '\n').encode())
            
            if cmd.startswith('B '):
                self.b_idle = False
            elif cmd.startswith('R '):
                self.r_idle = False
            elif cmd.startswith('L '):
                self.l_idle = False
            elif cmd.startswith('X '):
                self.x_idle = False
            elif cmd.startswith('Y '):
                self.y_idle = False
            elif cmd.startswith('Z '):
                self.z_idle = False
            time.sleep(0.010)

    def query_motor(self, motor_id: str) -> tuple[float, float]:
        """
        Queries motor/servo state.
        For steppers: remaining steps (float for consistency), RPS.
        For servos: current angle, speed.
        Concept: Unified interface; parses serial response.
        """
        with self.lock:
            self.ser.write(f'Q {motor_id}\n'.encode())
            time.sleep(0.05)
            line1 = self.ser.readline().decode().strip()
            line2 = self.ser.readline().decode().strip()
            line3 = self.ser.readline().decode().strip()
        
        if 'steps' in line1:
            try: value1 = float(line1.split(': ')[1])  # steps
            except: 
                print('Query error: no steps')
                value1 = None
        else:
            try: value1 = float(line1.split(': ')[1])  # angle
            except:
                print('Query error: no angle')
                value1 = None
        
        try: value2 = float(line2.split(': ')[1])  # RPS/speed
        except:
            print('Query error: no speed')
            value2 = None

        if 'R motor stopped.' in line3:
            self.r_idle = True
        elif 'L motor stopped.' in line3:
            self.l_idle = True
        elif 'B motor stopped.' in line3:
            self.b_idle = True
        elif 'X servo stopped.' in line3:
            self.x_idle = True
        elif 'Y servo stopped.' in line3:
            self.y_idle = True
        elif 'Z servo stopped.' in line3:
            self.z_idle = True

        return value1, value2

    def get_current_angle(self, joint: int) -> float:
        """
        Helper to get current angle for servo joints (4-6) from hardware.
        Raises error if not servo joint.
        Concept: For servos, hardware query provides ground truth; use for sync if needed.
        """
        if joint == 4:
            motor_id = 'X'
        elif joint == 5:
            motor_id = 'Y'
        elif joint == 6:
            motor_id = 'Z'
        else:
            raise ValueError("Only for servo joints 4-6.")
        angle, _ = self.query_motor(motor_id)
        return angle

    def get_tracked_angle(self, joint: int) -> float:
        """
        Returns the software-tracked current angle for any joint.
        Concept: Consistent access for UI display; prefers virtual state for safety decoupling.
        """
        if joint < 1 or joint > 6:
            raise ValueError("Invalid joint (1-6).")
        return self.current_angles[joint - 1]

    def set_current_zero(self, joint: int):
        """
        Resets the tracked angle for a joint to zero.
        Concept: Manual calibration; assumes user has positioned hardware at zero.
        Does not send commands to hardware.
        """
        if joint < 1 or joint > 6:
            raise ValueError("Invalid joint (1-6).")
        self.current_angles[joint - 1] = 0.0
        print(f"Joint {joint} tracked angle reset to 0°")