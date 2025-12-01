# Robot_Interpreter.py - Joint to Motor Mapper with Queuing
# Maps joint commands to motor strings; queues wrist (2/3) for serialization.
# Supports composite 2/3 commands with sync speed adjustment.
# v1.2: Added holding mechanism for zero-displacement motors in composite commands.
# When one wrist motor has zero displacement, it is enabled for holding torque to prevent drift,
# and disabled after the motion completes, using tracked holding sets per queued operation.

import threading
import queue
import time
import math
import copy

# Constants
G_B = 5.0
G_RL = 7.8
DEG_TOLERANCE = 1e-6  # Tolerance for skipping zero-displacement commands
MAX_MOTOR_RPS = 1.56  # Maximum RPS per motor, matching Arduino's STEP_MAX_RPS
ACCEL = 1.0  # Matches STEP_RAMP_ACCEL in Arduino

def Motor_Map(joint: int, displacement: float, speed: float) -> list[str]:
    if joint not in [1, 2, 3]:
        raise ValueError("Invalid joint.")
    if speed <= 0.0:
        raise ValueError("Speed positive.")

    commands = []

    if joint == 1:
        motor_deg = displacement * G_B
        motor_rps = speed * G_B
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg) > DEG_TOLERANCE:
            commands.append(f'B {motor_deg:.2f} {motor_rps:.3f}')

    elif joint == 2:
        motor_deg_R = displacement * G_RL
        motor_deg_L = -displacement * G_RL
        motor_rps = speed * G_RL
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg_R) > DEG_TOLERANCE:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps:.3f}')
        if abs(motor_deg_L) > DEG_TOLERANCE:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps:.3f}')

    elif joint == 3:
        motor_deg_R = -displacement * G_RL
        motor_deg_L = -displacement * G_RL
        motor_rps = speed * G_RL
        motor_rps = min(motor_rps, MAX_MOTOR_RPS)
        if abs(motor_deg_R) > DEG_TOLERANCE:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps:.3f}')
        if abs(motor_deg_L) > DEG_TOLERANCE:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps:.3f}')

    return commands

def compute_min_time(d, v_max, accel=ACCEL):
    if d <= DEG_TOLERANCE:
        return 0.0
    v_peak = math.sqrt(4.0 * d * accel / 3.0)
    if v_peak <= v_max:
        return math.sqrt(3.0 * d / accel)
    else:
        return d / v_max + (3.0 / 4.0) * v_max / accel

def solve_v(t, d, accel=ACCEL):
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

        # Two independent command queues
        self.base_queue = queue.Queue(maxsize=10)   # Only for Base (B) motor
        self.wrist_queue = queue.Queue(maxsize=10)  # For R and L (joints 2 & 3)

        # Motor idle tracking
        self.r_idle = True
        self.l_idle = True
        self.b_idle = True

        # Holding torque tracking for current operation
        self.current_holding = set()

        self.lock = threading.Lock()

        # Start independent worker threads
        threading.Thread(target=self._monitor_serial, daemon=True).start()
        threading.Thread(target=self._base_worker, daemon=True).start()
        threading.Thread(target=self._wrist_worker, daemon=True).start()

    # ──────────────────────────────────────────────────────────────
    # Serial monitor: detects when motors finish
    # ──────────────────────────────────────────────────────────────
    def _monitor_serial(self):
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

                        # When all relevant motors are idle, release held motors
                        if self.current_holding and self.is_arm_idle():
                            for motor_id in copy.copy(self.current_holding):
                                self.ser.write(f'D {motor_id}\n'.encode())
                            self.current_holding.clear()

                except Exception:
                    pass
            time.sleep(0.005)

    def is_arm_idle(self):
        return self.r_idle and self.l_idle and self.b_idle

    # ──────────────────────────────────────────────────────────────
    # Base worker: processes base commands independently
    # ──────────────────────────────────────────────────────────────
    def _base_worker(self):
        while True:
            if self.b_idle and not self.base_queue.empty():
                commands, holdings = self.base_queue.get()
                with self.lock:
                    self.b_idle = False
                    self.current_holding = holdings
                self._send_commands(commands)
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────────
    # Wrist worker: processes wrist commands only when both R & L are free
    # ──────────────────────────────────────────────────────────────
    def _wrist_worker(self):
        while True:
            if (self.r_idle and self.l_idle) and not self.wrist_queue.empty():
                commands, holdings = self.wrist_queue.get()
                with self.lock:
                    # Mark motors as busy only if they actually move
                    if any(c.startswith('R ') for c in commands):
                        self.r_idle = False
                    if any(c.startswith('L ') for c in commands):
                        self.l_idle = False
                    self.current_holding = holdings
                self._send_commands(commands)
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────────
    # Send single joint command (Joint 1 → B, Joint 2/3 → R/L)
    # ──────────────────────────────────────────────────────────────
    def send_joint_command(self, joint: int, displacement: float, speed: float):
        commands = Motor_Map(joint, displacement, speed)
        if not commands:
            return True  # Nothing to do

        holdings = set()
        b_moving = any(c.startswith('B ') for c in commands)
        wrist_moving = any(c.startswith('R ') or c.startswith('L ') for c in commands)

        # Auto-enable idle motors for holding torque
        if b_moving and not wrist_moving:
            commands[0:0] = ['E R', 'E L']
            holdings.update({'R', 'L'})
        elif wrist_moving and not b_moving:
            commands[0:0] = ['E B']
            holdings.add('B')

        with self.lock:
            q = self.base_queue if b_moving else self.wrist_queue
            try:
                was_empty = q.empty()
                q.put((commands, holdings), block=False)
                return was_empty  # True if command started immediately
            except queue.Full:
                print("Queue full; command dropped.")
                return False

    # ──────────────────────────────────────────────────────────────
    # Send composite wrist command (Joint 2 + Joint 3 together)
    # ──────────────────────────────────────────────────────────────
    def send_composite_command(self, disp2: float, speed2: float, disp3: float, speed3: float):
        if speed2 <= 0.0 or speed3 <= 0.0:
            raise ValueError("Speeds must be positive.")

        # Reuse your existing logic to compute motor_deg and motor_rps
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
        holdings = set()

        r_moving = abs(motor_deg_R) > DEG_TOLERANCE
        l_moving = abs(motor_deg_L) > DEG_TOLERANCE

        if r_moving:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps_R:.3f}')
        elif l_moving:
            commands.insert(0, 'E R')
            holdings.add('R')

        if l_moving:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps_L:.3f}')
        elif r_moving:
            commands.insert(0, 'E L')
            holdings.add('L')

        # Always hold base during wrist composite move
        if r_moving or l_moving:
            commands.insert(0, 'E B')
            holdings.add('B')

        with self.lock:
            try:
                was_empty = self.wrist_queue.empty()
                self.wrist_queue.put((commands, holdings), block=False)
                return was_empty and self.r_idle and self.l_idle
            except queue.Full:
                print("Wrist queue full")
                return False

    # ──────────────────────────────────────────────────────────────
    # Helper: send raw command strings
    # ──────────────────────────────────────────────────────────────
    def _send_commands(self, commands: list[str]):
        for cmd in commands:
            self.ser.write((cmd + '\n').encode())
            time.sleep(0.001)  # Tiny delay to prevent serial buffer overflow

    # ──────────────────────────────────────────────────────────────
    # Query motor state
    # ──────────────────────────────────────────────────────────────
    def query_motor(self, motor_id: str) -> tuple[int, float]:
        self.ser.write(f'Q {motor_id}\n'.encode())
        time.sleep(0.05)
        steps_line = self.ser.readline().decode().strip()
        rps_line = self.ser.readline().decode().strip()
        steps = int(steps_line.split(': ')[1])
        rps = float(rps_line.split(': ')[1])
        return steps, rps

if __name__ == "__main__":
    # Tests
    print(Motor_Map(1, 90.0, 0.5))  # ['B 450.00 2.500']
    print(Motor_Map(2, -45.0, 1.0))  # ['R -351.00 7.800', 'L 351.00 7.800']
    print(Motor_Map(3, 30.0, 0.75))  # ['R -234.00 5.850', 'L -234.00 5.850']