# Robot_Interpreter_v1.4.py
# Interpreter for robot arm control, handling both steppers and servos.
# v1.4: Added support for servo joints 4 (X), 5 (Y), 6 (Z).
#       - Servos use absolute positioning (0-270°).
#       - Added servo queue and idle tracking for independent control.
#       - Unified monitoring for all motors/servos.
#       - Concept: Queues ensure serialized commands to prevent serial overload; idle tracking allows concurrent motions where possible.

import threading
import queue
import time
import math

# Constants (Steppers, unchanged)
G_B = 5.0   # Gear ratio for base stepper
G_RL = 7.8  # Gear ratio for wrist steppers
DEG_TOLERANCE = 1e-6
MAX_MOTOR_RPS = 1.56  # Max RPS for steppers
ACCEL = 1.0

# Constants (Servos, from specs)
MAX_SERVO_ANGLE = 270.0
MAX_SERVO_RPS = 1.0  # Conservative max, derated from specs (0.14sec/60° ~1.19 RPS at 7.2V)

def Motor_Map(joint: int, displacement: float, speed: float) -> list[str]:
    """
    Maps joint commands to motor/servo commands based on joint ID.
    For steppers (1-3): computes displacements with gear ratios.
    For servos (4-6): direct absolute angles, clamped in Arduino.
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
        if abs(displacement) > DEG_TOLERANCE:  # Displacement here is absolute target for servos
            commands.append(f'X {displacement:.2f} {servo_rps:.3f}')

    elif joint == 5:  # Servo Y
        servo_rps = min(speed, MAX_SERVO_RPS)
        if abs(displacement) > DEG_TOLERANCE:
            commands.append(f'Y {displacement:.2f} {servo_rps:.3f}')

    elif joint == 6:  # Servo Z
        servo_rps = min(speed, MAX_SERVO_RPS)
        if abs(displacement) > DEG_TOLERANCE:
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
    if t < min_t:
        raise ValueError(f"Impossible time {t} < min {min_t} for d={d}")
    k = (3.0 / 4.0) / accel
    disc = t**2 - 4.0 * k * d
    v = (t - math.sqrt(disc)) / (2.0 * k)
    return v

class RobotInterpreter:
    def __init__(self, ser):
        self.ser = ser

        # Queues for independent control
        self.base_queue = queue.Queue(maxsize=10)   # For base stepper (B)
        self.wrist_queue = queue.Queue(maxsize=10)  # For wrist steppers (R, L)
        self.servo_queue = queue.Queue(maxsize=10)  # For servos (A, D, C)

        # Idle tracking for all
        self.r_idle = True
        self.l_idle = True
        self.b_idle = True
        self.a_idle = True
        self.d_idle = True
        self.c_idle = True

        self.lock = threading.Lock()

        # Start threads
        threading.Thread(target=self._monitor_serial, daemon=True).start()
        threading.Thread(target=self._base_worker, daemon=True).start()
        threading.Thread(target=self._wrist_worker, daemon=True).start()
        threading.Thread(target=self._servo_worker, daemon=True).start()

    def _monitor_serial(self):
        """
        Monitors serial for 'stopped' messages to update idle states.
        Concept: Non-blocking status updates; enables queue processing.
        """
        while True:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if not line:
                        continue
                    with self.lock:
                        if 'R motor stopped.' in line:
                            self.r_idle = True
                        if 'L motor stopped.' in line:
                            self.l_idle = True
                        if 'B motor stopped.' in line:
                            self.b_idle = True
                        if 'X servo stopped.' in line:
                            self.a_idle = True
                        if 'Y servo stopped.' in line:
                            self.d_idle = True
                        if 'Z servo stopped.' in line:
                            self.c_idle = True
                except Exception:
                    pass
            time.sleep(0.005)

    def _base_worker(self):
        """
        Processes base queue when idle.
        Concept: Dedicated thread for independent axis control.
        """
        while True:
            if self.b_idle and not self.base_queue.empty():
                commands = self.base_queue.get()
                with self.lock:
                    self.b_idle = False
                self._send_commands(commands)
            time.sleep(0.005)

    def _wrist_worker(self):
        """
        Processes wrist queue when both R and L idle.
        """
        while True:
            if (self.r_idle and self.l_idle) and not self.wrist_queue.empty():
                commands = self.wrist_queue.get()
                with self.lock:
                    if any(c.startswith('R ') for c in commands):
                        self.r_idle = False
                    if any(c.startswith('L ') for c in commands):
                        self.l_idle = False
                self._send_commands(commands)
            time.sleep(0.005)

    def _servo_worker(self):
        """
        Processes servo queue; servos can move independently.
        Concept: Single queue for all servos; idle per servo but process if any ready.
        """
        while True:
            if not self.servo_queue.empty():
                commands = self.servo_queue.get()
                with self.lock:
                    if any(c.startswith('X ') for c in commands):
                        self.a_idle = False
                    if any(c.startswith('Y ') for c in commands):
                        self.d_idle = False
                    if any(c.startswith('Z ') for c in commands):
                        self.c_idle = False
                self._send_commands(commands)
            time.sleep(0.005)

    def send_joint_command(self, joint: int, displacement: float, speed: float):
        """
        Sends command for single joint; selects queue based on joint type.
        For servos, displacement is absolute angle.
        Returns True if started immediately.
        """
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
                return immediate and q.empty()  # Approximate immediate start
            except queue.Full:
                print("Queue full; command dropped.")
                return False

    def send_composite_command(self, disp2: float, speed2: float, disp3: float, speed3: float):
        """
        Sends synchronized command for wrist joints 2 and 3 (steppers only).
        Computes common time for coordination.
        """
        if speed2 <= 0.0 or speed3 <= 0.0:
            raise ValueError("Speeds must be positive.")

        rev2 = abs(disp2) / 360.0
        rev3 = abs(disp3) / 360.0
        if rev2 == 0 and rev3 == 0:
            return True

        max_joint_rps = MAX_MOTOR_RPS / G_RL
        speed2 = min(speed2, max_joint_rps)
        speed3 = min(speed3, max_joint_rps)

        quick_time = rev2 / speed2 if rev2 > 0 else rev3 / speed3

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
                return was_empty and self.r_idle and self.l_idle
            except queue.Full:
                print("Wrist queue full")
                return False

    def _send_commands(self, commands: list[str]):
        """
        Sends list of commands over serial with small delay.
        Concept: Prevents buffer overflow in serial communication.
        """
        for cmd in commands:
            self.ser.write((cmd + '\n').encode())
            time.sleep(0.001)

    def query_motor(self, motor_id: str) -> tuple[float, float]:
        """
        Queries motor/servo state.
        For steppers: remaining steps (float for consistency), RPS.
        For servos: current angle, speed.
        Concept: Unified interface; parses serial response.
        """
        self.ser.write(f'Q {motor_id}\n'.encode())
        time.sleep(0.05)
        line1 = self.ser.readline().decode().strip()
        line2 = self.ser.readline().decode().strip()
        if 'steps' in line1:
            value1 = float(line1.split(': ')[1])  # steps
        else:
            value1 = float(line1.split(': ')[1])  # angle
        value2 = float(line2.split(': ')[1])  # RPS/speed
        return value1, value2

    def get_current_angle(self, joint: int) -> float:
        """
        Helper to get current angle for servo joints (4-6).
        Raises error if not servo joint.
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