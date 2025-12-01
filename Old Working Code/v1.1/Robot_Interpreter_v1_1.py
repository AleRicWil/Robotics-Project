# Robot_Interpreter.py - Joint to Motor Mapper with Queuing
# v1.1
# Maps joint commands to motor strings; queues wrist (2/3) for serialization.
# Supports composite 2/3 commands with sync speed adjustment.

import threading
import queue
import time
import math

# Constants
G_B = 5.0
G_RL = 7.8
DEG_TOLERANCE = 1e-6  # Tolerance for skipping zero-displacement commands
MAX_MOTOR_RPS = 1.56  # Maximum RPS per motor, matching Arduino's STEP_MAX_RPS
ACCEL = 2.0  # Matches STEP_RAMP_ACCEL in Arduino

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
        self.wrist_command_queue = queue.Queue(maxsize=10)  # Prevent overflow
        self.r_idle = True
        self.l_idle = True
        self.lock = threading.Lock()
        self.monitor_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        self.monitor_thread.start()

    def _monitor_serial(self):
        while True:
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        with self.lock:
                            if 'R motor stopped.' in line:
                                self.r_idle = True
                            if 'L motor stopped.' in line:
                                self.l_idle = True
                            if self.is_wrist_idle():
                                self._process_queue()
                except Exception:
                    pass
            else:
                time.sleep(0.01)

    def is_wrist_idle(self):
        return self.r_idle and self.l_idle

    def send_joint_command(self, joint: int, displacement: float, speed: float):
        commands = Motor_Map(joint, displacement, speed)
        with self.lock:
            if joint == 1:
                if commands:
                    self._send_commands(commands)
                return True
            else:
                if not commands:
                    return True  # No action needed for zero displacement
                try:
                    self.wrist_command_queue.put(commands, block=False)
                    self._process_queue()
                    return False
                except queue.Full:
                    print("Queue full; command dropped.")
                    return False

    def send_composite_command(self, disp2: float, speed2: float, disp3: float, speed3: float):
        if speed2 <= 0.0 or speed3 <= 0.0:
            raise ValueError("Speeds positive.")

        rev2 = abs(disp2) / 360.0
        rev3 = abs(disp3) / 360.0
        if rev2 == 0 and rev3 == 0:
            return True

        # Clamp input speeds to max joint RPS
        max_joint_rps = MAX_MOTOR_RPS / G_RL
        speed2 = min(speed2, max_joint_rps)
        speed3 = min(speed3, max_joint_rps)

        # Compute quick_time according to joint 2 if possible, else joint 3
        if rev2 > 0:
            quick_time = rev2 / speed2
        else:
            quick_time = rev3 / speed3

        # Compute motor revs (abs)
        motor_rev_R = abs((disp2 * G_RL) + (-disp3 * G_RL)) / 360.0
        motor_rev_L = abs((-disp2 * G_RL) + (-disp3 * G_RL)) / 360.0

        # Compute min times
        min_t_R = compute_min_time(motor_rev_R, MAX_MOTOR_RPS)
        min_t_L = compute_min_time(motor_rev_L, MAX_MOTOR_RPS)
        min_common_t = max(min_t_R, min_t_L)

        # Set common_time
        common_time = max(quick_time, min_common_t)

        # Compute motor RPS
        motor_rps_R = solve_v(common_time, motor_rev_R) if motor_rev_R > DEG_TOLERANCE else 0.0
        motor_rps_L = solve_v(common_time, motor_rev_L) if motor_rev_L > DEG_TOLERANCE else 0.0

        # Compute signed degrees
        motor_deg_R = (disp2 * G_RL) + (-disp3 * G_RL)
        motor_deg_L = (-disp2 * G_RL) + (-disp3 * G_RL)

        commands = []
        if abs(motor_deg_R) > DEG_TOLERANCE:
            commands.append(f'R {motor_deg_R:.2f} {motor_rps_R:.3f}')
        if abs(motor_deg_L) > DEG_TOLERANCE:
            commands.append(f'L {motor_deg_L:.2f} {motor_rps_L:.3f}')

        with self.lock:
            if not commands:
                return True  # No action needed
            try:
                self.wrist_command_queue.put(commands, block=False)
                self._process_queue()
                return False
            except queue.Full:
                print("Queue full; command dropped.")
                return False

    def query_motor(self, motor_id: str) -> tuple[int, float]:
        self.ser.write(f'Q {motor_id}\n'.encode())
        steps_line = self.ser.readline().decode().strip()
        rps_line = self.ser.readline().decode().strip()
        steps = int(steps_line.split(': ')[1])
        rps = float(rps_line.split(': ')[1])
        return steps, rps

    def _process_queue(self):
        if self.is_wrist_idle() and not self.wrist_command_queue.empty():
            commands = self.wrist_command_queue.get()
            if not commands:
                return
            r_involved = any(c.startswith('R ') for c in commands)
            l_involved = any(c.startswith('L ') for c in commands)
            if r_involved:
                self.r_idle = False
            if l_involved:
                self.l_idle = False
            self._send_commands(commands)

    def _send_commands(self, commands: list[str]):
        for cmd in commands:
            self.ser.write((cmd + '\n').encode())

if __name__ == "__main__":
    # Tests
    print(Motor_Map(1, 90.0, 0.5))  # ['B 450.00 2.500']
    print(Motor_Map(2, -45.0, 1.0))  # ['R -351.00 7.800', 'L 351.00 7.800']
    print(Motor_Map(3, 30.0, 0.75))  # ['R -234.00 5.850', 'L -234.00 5.850']