import threading
import queue
import time
import math

# Constants (unchanged)
G_B = 5.0
G_RL = 7.8
DEG_TOLERANCE = 1e-6
MAX_MOTOR_RPS = 1.56
ACCEL = 1.0

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

        self.lock = threading.Lock()

        # Start independent worker threads
        threading.Thread(target=self._monitor_serial, daemon=True).start()
        threading.Thread(target=self._base_worker, daemon=True).start()
        threading.Thread(target=self._wrist_worker, daemon=True).start()

    # ──────────────────────────────────────────────────────────────
    # Serial monitor: detects when motors finish (REMOVED holding disables)
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
                except Exception:
                    pass
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────────
    # Base worker: processes base commands independently
    # ──────────────────────────────────────────────────────────────
    def _base_worker(self):
        while True:
            if self.b_idle and not self.base_queue.empty():
                commands = self.base_queue.get()
                with self.lock:
                    self.b_idle = False
                self._send_commands(commands)
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────────
    # Wrist worker: processes wrist commands only when both R & L are free
    # ──────────────────────────────────────────────────────────────
    def _wrist_worker(self):
        while True:
            if (self.r_idle and self.l_idle) and not self.wrist_queue.empty():
                commands = self.wrist_queue.get()
                with self.lock:
                    # Mark motors as busy only if they actually move
                    if any(c.startswith('R ') for c in commands):
                        self.r_idle = False
                    if any(c.startswith('L ') for c in commands):
                        self.l_idle = False
                self._send_commands(commands)
            time.sleep(0.005)

    # ──────────────────────────────────────────────────────────────
    # Send single joint command (REMOVED all holding logic)
    # ──────────────────────────────────────────────────────────────
    def send_joint_command(self, joint: int, displacement: float, speed: float):
        commands = Motor_Map(joint, displacement, speed)
        if not commands:
            return True  # Nothing to do

        b_moving = any(c.startswith('B ') for c in commands)

        with self.lock:
            q = self.base_queue if b_moving else self.wrist_queue
            try:
                was_empty = q.empty()
                q.put(commands, block=False)  # Now just commands, no holdings
                return was_empty  # True if command started immediately
            except queue.Full:
                print("Queue full; command dropped.")
                return False

    # ──────────────────────────────────────────────────────────────
    # Send composite wrist command (REMOVED all holding logic)
    # ──────────────────────────────────────────────────────────────
    def send_composite_command(self, disp2: float, speed2: float, disp3: float, speed3: float):
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