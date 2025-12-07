# Joint_Control_UI_v2_1.py - GUI for Robot Arm Joint/Servo Control
# v2.0: Added display of current angles, updated at configurable rate (1Hz default).
#       - Added "Zero" button per joint for manual zeroing (calls interpreter.set_current_zero).
#       - Current angles queried from interpreter.get_tracked_angle for consistency.
#       - Polling thread for non-blocking updates; configurable UPDATE_RATE_HZ at top.
#       - Labels for currents next to each row; updates include gripper label fix.
#       - Concept: Enhances usability with real-time feedback; follows MVC-like separation for serviceability.

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
from Robot_Interpreter_v2_1 import RobotInterpreter, MAX_SERVO_ANGLE
import threading
import time

# New imports for scoop routine (kinematics and math operations)
# Concept: These enable IK planning and unit conversions; kept minimal for UI focus.
import numpy as np
from kinematics_v2_1 import SerialArm
import transforms_v2_1 as tr
from example_sim_scoop_v2_1 import dh, jt_types, CARRY_TILT, CARRY_TILT_DEG, JOINT_LIMITS_DEG, JOINT_LIMITS_RAD, tip

# Configurable update rate for current angle display (Hz)
UPDATE_RATE_HZ = 2  # Default 1Hz; adjust for smoother display vs. performance

# Constants
BAUD_RATE = 115200
TIMEOUT = 1
DEFAULT_RPS = '0.1'
NEGATIVE_DELTAS = [-360, -180, -90, -10, -5, -1]
POSITIVE_DELTAS = [1, 5, 10, 90, 180, 360]
SERVO_JOINTS = [4, 5, 6]  # Joints using servos. 6 is actually the gripper. Called joint for convenience

# New constants for scoop routine (duplicated from example_sim_scoop_v2_1.py for self-containment)
# Concept: Derived from hardware specs; can be centralized in a config file for future flexibility.
GRIPPER_OPEN = 0.0      # deg, adjust for hardware calibration
GRIPPER_CLOSE = 90.0    # deg, adjust for hardware
DEFAULT_SPEED = float(DEFAULT_RPS)  # RPS for motions
N_STEPS = 1            # interpolation steps per segment (reduced for hardware to avoid queue overflow)
# CARRY_TILT_DEG = 15.0
# CARRY_TILT = np.radians(CARRY_TILT_DEG)

# DH parameters for the arm (5-DOF positioning, gripper separate)
# dh = [
#     [0.0, 0.20, 0.0,  np.pi / 2],   # J1 base yaw
#     [0.0, 0.00, 0.0, -np.pi / 2],   # J2 sideways
#     [0.0, 0.20, 0.0,  np.pi / 2],   # J3 vertical-ish
#     [0.0, 0.00, 0.15, 0.0],         # J4 sideways
#     [0.0, 0.00, 0.05, 0.0],         # J5 pitch
# ]
# jt_types = ['r'] * 5

# Joint limits in degrees (then converted to radians for kinematics)
# JOINT_LIMITS_DEG = [
#     [-180, 180],        # J1 base ±180°
#     [-135, 135],        # J2
#     [-180, 180],        # J3
#     [-120, 120],        # J4 servo X
#     [-120, 120],        # J5 servo Y
# ]
# JOINT_LIMITS_RAD = np.radians(JOINT_LIMITS_DEG)

# Gripper offset along x-axis (tool frame)
# tip = tr.se3(np.eye(3), np.array([0.05, 0.0, 0.0]))

class JointControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Control UI")
        self.root.minsize(800, 600)  # Adjusted for more rows
        self.root.configure(bg="#E3F2FD")

        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TLabel', font=('Arial', 12), background="#E3F2FD")
        self.style.configure('TButton', font=('Arial', 10), padding=6)
        self.style.configure('TEntry', font=('Arial', 12), padding=5)
        self.style.configure('TCombobox', font=('Arial', 10))
        self.style.map('TButton', background=[('active', '#E0E0E0')])
        self.style.configure('Joint.TFrame', background='#FFFFFF')

        self.ser = None
        self.interpreter = None

        self.available_ports = [port.device for port in serial.tools.list_ports.comports()]

        serial_frame = ttk.Frame(root, padding=10, style='Joint.TFrame')
        serial_frame.grid(row=0, column=0, columnspan=20, sticky='ew')

        ttk.Label(serial_frame, text="Serial Port:", font=('Arial', 12, 'bold')).pack(side='left', padx=5)

        default_port = self.available_ports[0] if self.available_ports else ''
        self.port_var = tk.StringVar(value=default_port)
        port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, values=self.available_ports, width=15)
        port_combo.pack(side='left', padx=5)

        self.connect_button = ttk.Button(serial_frame, text="Connect", command=self.connect_to_port, style='Accent.TButton')
        self.style.configure('Accent.TButton', background='#4C8BF5', foreground='white')
        self.style.map('Accent.TButton', background=[('active', '#3A7BF0')])
        self.connect_button.pack(side='left', padx=5)

        self.status_label = ttk.Label(serial_frame, text="Not Connected", foreground="red", font=('Arial', 12, 'bold'))
        self.status_label.pack(side='left', padx=5)

        self.style.configure('Hold.TButton', background='#FFC107', foreground='white')  # Yellow for hold
        self.style.map('Hold.TButton', background=[('active', '#FFA000')])
        self.style.configure('Release.TButton', background='#FF6B6B', foreground='white')  # Red for release
        self.style.map('Release.TButton', background=[('active', '#FF4757')])
        self.hold_button = ttk.Button(serial_frame, text="Hold Motors", command=self.toggle_hold, style='Hold.TButton')
        self.hold_button.pack(side='left', padx=5)

        self.displacement_entries = {}
        self.speed_entries = {}
        self.current_labels = {}  # New: Labels for displaying current angles

        # Create rows for all joints
        self.create_joint_row(1, "Joint 1 (Base Stepper)", 1)
        self.create_joint_row(2, "Joint 2 (Wrist Stepper Diff)", 2)
        self.create_joint_row(3, "Joint 3 (Wrist Stepper Common)", 3)
        self.create_joint_row(4, "Joint 4 (Servo X)", 4)
        self.create_joint_row(5, "Joint 5 (Servo Y)", 5)
        self.create_joint_row(6, "Gripper (Servo Z)", 6)

        self.create_composite_section(7)

        # New: Automated Routines section
        # Concept: Extensible area for routine buttons; follows UI modularity for future additions.
        routines_frame = ttk.Frame(self.root, padding=5, relief='ridge', borderwidth=1, style='Joint.TFrame')
        routines_frame.grid(row=8, column=0, columnspan=20, pady=5, sticky='ew')
        ttk.Label(routines_frame, text="Automated Routines", font=('Arial', 14, 'bold')).pack(side='left', padx=10)
        self.style.configure('Routine.TButton', background='#51CF66', foreground='white')
        self.style.map('Routine.TButton', background=[('active', '#38D9A9')])
        run_scoop_button = ttk.Button(routines_frame, text="Run Scoop Routine", command=self.run_scoop, style='Routine.TButton')
        run_scoop_button.pack(side='left', padx=5)
        self.routine_status_label = ttk.Label(routines_frame, text="Idle", font=('Arial', 12))
        self.routine_status_label.pack(side='left', padx=10)

        self.message_label = ttk.Label(root, text="", foreground="black", font=('Arial', 12, 'italic'), background="#E3F2FD")
        self.message_label.grid(row=9, column=0, columnspan=20, pady=10, sticky='w')

        if self.available_ports:
            self.connect_to_port()
        else:
            self.status_label.config(text="No active ports", foreground="red")
            self.message_label.config(text="No ports detected.", foreground="red")

        # Polling thread for updating current angles
        self.update_thread = threading.Thread(target=self._update_current_angles, daemon=True)
        self.update_thread.start()

        # New: Initialize kinematics model for scoop routine
        # Concept: Models the 5-DOF arm; gripper handled separately per hardware convention.
        self.arm = SerialArm(dh, jt=jt_types, tip=tip, joint_limits=JOINT_LIMITS_RAD, gripper=True)

    def create_joint_row(self, joint: int, label_text: str, row: int):
        """
        Creates UI row for a joint/servo.
        Adds "Zero" button and current angle label.
        Concept: Modular rows for extensibility; binds Enter for convenience.
        """
        joint_frame = ttk.Frame(self.root, padding=5, relief='ridge', borderwidth=1, style='Joint.TFrame')
        joint_frame.grid(row=row, column=0, columnspan=20, pady=5, sticky='ew')

        ttk.Label(joint_frame, text=label_text, font=('Arial', 14, 'bold')).pack(side='left', padx=10)

        for delta in NEGATIVE_DELTAS:
            btn = ttk.Button(joint_frame, text=str(delta), width=5, command=lambda d=delta, j=joint: self.send_preset_command(j, d))
            self.style.configure('Neg.TButton', background='#FF6B6B', foreground='white')
            self.style.map('Neg.TButton', background=[('active', '#FF4757')])
            btn['style'] = 'Neg.TButton'
            btn.pack(side='left', padx=2)

        disp_entry = ttk.Entry(joint_frame, width=12, font=('Arial', 12))
        disp_entry.pack(side='left', padx=5)
        disp_entry.bind("<Return>", lambda event, j=joint: self.send_command(j))
        self.displacement_entries[joint] = disp_entry

        for delta in POSITIVE_DELTAS:
            btn = ttk.Button(joint_frame, text=f"+{delta}", width=5, command=lambda d=delta, j=joint: self.send_preset_command(j, d))
            self.style.configure('Pos.TButton', background='#51CF66', foreground='white')
            self.style.map('Pos.TButton', background=[('active', '#38D9A9')])
            btn['style'] = 'Pos.TButton'
            btn.pack(side='left', padx=2)

        check_button = ttk.Button(joint_frame, text="✓", width=4, command=lambda j=joint: self.send_command(j))
        self.style.configure('Check.TButton', background='#2F9E44', foreground='white')
        self.style.map('Check.TButton', background=[('active', '#1B7837')])
        check_button['style'] = 'Check.TButton'
        check_button.pack(side='left', padx=5)

        speed_entry = ttk.Entry(joint_frame, width=8, font=('Arial', 12))
        speed_entry.insert(0, DEFAULT_RPS)
        speed_entry.pack(side='left', padx=5)
        speed_entry.bind("<Return>", lambda event, j=joint: self.send_command(j))
        self.speed_entries[joint] = speed_entry

        ttk.Label(joint_frame, text="RPS", font=('Arial', 12)).pack(side='left', padx=5)

        query_button = ttk.Button(joint_frame, text="Query", width=6, command=lambda j=joint: self.query_joint(j))
        self.style.configure('Query.TButton', background='#FFC107', foreground='white')
        self.style.map('Query.TButton', background=[('active', '#FFA000')])
        query_button['style'] = 'Query.TButton'
        query_button.pack(side='left', padx=5)

        # New: Zero button
        zero_button = ttk.Button(joint_frame, text="Zero", width=6, command=lambda j=joint: self.zero_joint(j))
        self.style.configure('Zero.TButton', background='#FFD700', foreground='white')  # Gold for zero
        self.style.map('Zero.TButton', background=[('active', '#FFC700')])
        zero_button['style'] = 'Zero.TButton'
        zero_button.pack(side='left', padx=5)

        # New: Current angle label
        current_label = ttk.Label(joint_frame, text="Current: 0.0°", font=('Arial', 12))
        current_label.pack(side='left', padx=10)
        self.current_labels[joint] = current_label

    def create_composite_section(self, row: int):
        """
        Creates composite section for wrist steppers (joints 2 & 3).
        Independent speeds for coordination.
        """
        composite_frame = ttk.Frame(self.root, padding=5, relief='ridge', borderwidth=1, style='Joint.TFrame')
        composite_frame.grid(row=row, column=0, columnspan=20, pady=5, sticky='ew')

        ttk.Label(composite_frame, text="Wrist Composite", font=('Arial', 14, 'bold')).pack(side='left', padx=10)

        ttk.Label(composite_frame, text="Joint 2 Disp:", font=('Arial', 12)).pack(side='left', padx=5)
        self.disp2_entry = ttk.Entry(composite_frame, width=8, font=('Arial', 12))
        self.disp2_entry.pack(side='left', padx=5)

        ttk.Label(composite_frame, text="Speed2 (RPS):", font=('Arial', 12)).pack(side='left', padx=5)
        self.speed2_entry = ttk.Entry(composite_frame, width=6, font=('Arial', 12))
        self.speed2_entry.insert(0, DEFAULT_RPS)
        self.speed2_entry.pack(side='left', padx=5)

        ttk.Label(composite_frame, text="Joint 3 Disp:", font=('Arial', 12)).pack(side='left', padx=5)
        self.disp3_entry = ttk.Entry(composite_frame, width=8, font=('Arial', 12))
        self.disp3_entry.pack(side='left', padx=5)

        ttk.Label(composite_frame, text="Speed3 (RPS):", font=('Arial', 12)).pack(side='left', padx=5)
        self.speed3_entry = ttk.Entry(composite_frame, width=6, font=('Arial', 12))
        self.speed3_entry.insert(0, DEFAULT_RPS)
        self.speed3_entry.pack(side='left', padx=5)

        combined_button = ttk.Button(composite_frame, text="Send Combined", command=self.send_composite)
        self.style.configure('Combined.TButton', background='#4C8BF5', foreground='white')
        self.style.map('Combined.TButton', background=[('active', '#3A7BF0')])
        combined_button['style'] = 'Combined.TButton'
        combined_button.pack(side='left', padx=15)

    def _update_current_angles(self):
        """
        Polling loop to update current angle labels at UPDATE_RATE_HZ.
        Concept: Non-blocking thread; queries tracked angles for display.
        """
        while True:
            if self.interpreter:
                for joint in range(1, 7):
                    try:
                        angle = self.interpreter.get_tracked_angle(joint)
                        self.current_labels[joint].config(text=f"Current: {angle:.1f}°")
                    except Exception as e:
                        print(f"UI angle update failed for joint {joint}: {e}")
            time.sleep(1.0 / UPDATE_RATE_HZ)

    def send_preset_command(self, joint: int, delta: float):
        """
        Sends preset delta; for servos, computes new absolute from tracked current.
        Validates/clamps in interpreter.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            speed_str = self.speed_entries[joint].get().strip()
            if not speed_str:
                raise ValueError("RPS empty.")
            speed = float(speed_str)

            if joint in SERVO_JOINTS:
                current = self.interpreter.get_tracked_angle(joint)
                target = current + delta
                was_sent = self.interpreter.send_joint_command(joint, target, speed)
                msg = f"Relative {delta}° sent (new abs {target:.1f}°)" if was_sent else f"Relative {delta}° queued"
            else:
                was_sent = self.interpreter.send_joint_command(joint, delta, speed)
                msg = f"Displacement {delta}° sent" if was_sent else f"Displacement {delta}° queued"

            color = "green" if was_sent else "blue"
            self.message_label.config(text=f"{msg} for Joint {joint}.", foreground=color)
        except ValueError as ve:
            self.message_label.config(text=f"Invalid RPS: {ve}", foreground="red")
        except Exception as e:
            self.message_label.config(text=f"Send failed: {e}", foreground="red")

    def send_command(self, joint: int):
        """
        Sends from entry; absolute for servos, displacement for steppers.
        Validates/clamps in interpreter.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            disp_str = self.displacement_entries[joint].get().strip()
            speed_str = self.speed_entries[joint].get().strip()
            if not disp_str or not speed_str:
                raise ValueError("Fields empty.")
            displacement = float(disp_str)
            speed = float(speed_str)
            was_sent = self.interpreter.send_joint_command(joint, displacement, speed)
            label = "Absolute" if joint in SERVO_JOINTS else "Displacement"
            msg = f"{label} {displacement}° sent" if was_sent else f"{label} {displacement}° queued"
            color = "green" if was_sent else "blue"
            self.message_label.config(text=f"{msg} for Joint {joint}.", foreground=color)
        except ValueError as ve:
            self.message_label.config(text=f"Invalid input: {ve}", foreground="red")
        except Exception as e:
            self.message_label.config(text=f"Send failed: {e}", foreground="red")

    def send_composite(self):
        """
        Sends composite for wrist steppers.
        Validates/clamps in interpreter.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            disp2_str = self.disp2_entry.get().strip()
            speed2_str = self.speed2_entry.get().strip()
            disp3_str = self.disp3_entry.get().strip()
            speed3_str = self.speed3_entry.get().strip()

            if not all([disp2_str, speed2_str, disp3_str, speed3_str]):
                raise ValueError("All fields required.")

            disp2 = float(disp2_str)
            speed2 = float(speed2_str)
            disp3 = float(disp3_str)
            speed3 = float(speed3_str)

            was_sent = self.interpreter.send_composite_command(disp2, speed2, disp3, speed3)
            msg = "Composite sent" if was_sent else "Composite queued"
            color = "green" if was_sent else "blue"
            self.message_label.config(text=f"{msg} (J2:{disp2}@{speed2} RPS, J3:{disp3}@{speed3} RPS).", foreground=color)
        except ValueError as ve:
            self.message_label.config(text=f"Invalid input: {ve}", foreground="red")
        except Exception as e:
            self.message_label.config(text=f"Send failed: {e}", foreground="red")

    def connect_to_port(self):
        """
        Connects to selected serial port.
        Passes default limits if none specified (can override here if needed).
        """
        port = self.port_var.get()
        if not port:
            self.message_label.config(text="No port.", foreground="red")
            return
        if self.ser:
            self.ser.close()
            self.ser = None
            self.interpreter = None
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT, write_timeout=None)
            self.interpreter = RobotInterpreter(self.ser)  # Use defaults; or pass custom limits
            self.status_label.config(text="Connected", foreground="green")
            self.message_label.config(text=f"Connected to {port}.", foreground="green")
            self.hold_active = False
            self.hold_button.config(text="Hold Motors", style='Hold.TButton')
        except Exception as e:
            self.ser = None
            self.interpreter = None
            self.status_label.config(text="Not Connected", foreground="red")
            self.message_label.config(text=f"Failed: {e}", foreground="red")

    def toggle_hold(self):
        """
        Toggles stepper hold mode via serial commands.
        Concept: Provides UI control for persistent torque hold; tracks state locally for button feedback.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            # Initialize state if not set
            if not hasattr(self, 'hold_active'):
                self.hold_active = False
            if not self.hold_active:
                self.interpreter.enable_hold()
                self.hold_button.config(text="Release Motors", style='Release.TButton')
                self.message_label.config(text="Hold mode enabled.", foreground="green")
                self.hold_active = True
            else:
                self.interpreter.disable_hold()
                self.hold_button.config(text="Hold Motors", style='Hold.TButton')
                self.message_label.config(text="Hold mode disabled.", foreground="blue")
                self.hold_active = False
        except Exception as e:
            self.message_label.config(text=f"Toggle failed: {e}", foreground="red")

    def query_joint(self, joint: int):
        """
        Queries joint state; formats based on type.
        For steppers: combined R/L if applicable.
        Uses hardware query where available.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            if joint == 1:
                steps, rps = self.interpreter.query_motor('B')
                self.message_label.config(text=f"Joint 1 (B): Remaining steps {steps:.0f}, RPS {rps:.3f}", foreground="black")
            elif joint in [2, 3]:
                steps_r, rps_r = self.interpreter.query_motor('R')
                steps_l, rps_l = self.interpreter.query_motor('L')
                self.message_label.config(text=f"Joint {joint} (R): Steps {steps_r:.0f}, RPS {rps_r:.3f}; (L): Steps {steps_l:.0f}, RPS {rps_l:.3f}", foreground="black")
            elif joint == 4:
                angle, speed = self.interpreter.query_motor('X')
                self.message_label.config(text=f"Joint 4 (X): Angle {angle:.1f}°, Speed {speed:.3f} RPS", foreground="black")
            elif joint == 5:
                angle, speed = self.interpreter.query_motor('Y')
                self.message_label.config(text=f"Joint 5 (Y): Angle {angle:.1f}°, Speed {speed:.3f} RPS", foreground="black")
            elif joint == 6:
                angle, speed = self.interpreter.query_motor('Z')
                self.message_label.config(text=f"Gripper (Z): Angle {angle:.1f}°, Speed {speed:.3f} RPS", foreground="black")
        except Exception as e:
            self.message_label.config(text=f"Query failed: {e}", foreground="red")

    def zero_joint(self, joint: int):
        """
        Resets tracked angle to zero for the joint.
        Concept: Manual calibration; prompts user to confirm physical alignment.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return
        try:
            self.interpreter.set_current_zero(joint)
            self.message_label.config(text=f"Joint {joint} zeroed.", foreground="green")
        except Exception as e:
            self.message_label.config(text=f"Zero failed: {e}", foreground="red")

    # Scoop Routine Helpers
    # Concept: Adapted from example_sim_scoop_v2_1.py; focuses on hardware execution with unit conversions and safety waits.

    def _base_yaw_from_xy(self, x: float, y: float) -> float:
        """Estimate base joint angle from target XY position."""
        return np.arctan2(y, x)

    def _level_last_joint(
        self, arm: SerialArm,
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
        Concept: Uses gradient descent to level tip axis, common in robotics for orientation control.
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

    def _solve_ik_pos(
        self, arm: SerialArm,
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

    def _solve_ik_level_tip(
        self, arm: SerialArm,
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
        q_pos = self._solve_ik_pos(arm, target_p, q_seed, label=label)

        # 2) Level the last joint
        q_level = self._level_last_joint(arm, q_pos)

        # Optional: print how level it is
        T = arm.fk(q_level, tip=True)
        R = T[0:3, 0:3]
        tip_up = R[:, 1]  # assuming tool Y is the “up” axis
        print(f"{label} final tip_up:", tip_up)

        return q_level

    def _interpolate(self, q_start: np.ndarray, q_end: np.ndarray, n_steps: int = N_STEPS):
        """Generator yielding linearly interpolated joint configurations."""
        # Concept: Linear interpolation is a standard method in robotics for smooth trajectory generation
        # between waypoints. Here, we yield points from q_start to q_end. To bypass interpolation 
        # (i.e., move directly to q_end without intermediate steps), set n_steps=1, which yields 
        # only q_end. This follows industry practice for optional smoothing in motion planning, 
        # enhancing flexibility without altering the calling code structure.
        if n_steps == 1:
            yield q_end  # Directly yield the end configuration to bypass interpolation
        else:
            # Standard linear interpolation over n_steps points, including start and end
            for s in np.linspace(0.0, 1.0, n_steps):
                yield (1.0 - s) * q_start + s * q_end

    def _with_carry_tilt(self, q: np.ndarray, tilt: float = CARRY_TILT) -> np.ndarray:
        """Return a copy of q with the last joint tilted back by `tilt` radians."""
        q_tilt = np.array(q, dtype=float).copy()
        q_tilt[4] -= tilt  # J5 is index 4
        return q_tilt

    def _plan_waypoints(self, arm: SerialArm) -> list[np.ndarray]:
        """
        Compute key poses/waypoints for:
          home → above scoop → scoop path → carry to bucket (tilted back) →
          dump → return home.
        """
        # Cartesian positions (m)
        pick_xy = np.array([0.5, 0.0])
        box_xy = np.array([-0.2, -0.2])
        z_above = 0.40
        z_table = 0.05          # where the scoop scrapes
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
        q_seed_scoop[0] = self._base_yaw_from_xy(*pick_xy)

        # Level-tip IK for all poses where we want the tool to be flat
        q_above_scoop = self._solve_ik_level_tip(arm, p_above_scoop, q_seed_scoop, "above_scoop")
        q_scoop_start = self._solve_ik_level_tip(arm, p_scoop_start, q_above_scoop, "scoop_start")
        q_scoop_end   = self._solve_ik_level_tip(arm, p_scoop_end,   q_scoop_start, "scoop_end")

        # Seed yaw toward the bucket
        q_seed_bucket = q_home.copy()
        q_seed_bucket[0] = self._base_yaw_from_xy(*box_xy)

        q_above_bucket = self._solve_ik_level_tip(arm, p_above_bucket, q_seed_bucket, "above_bucket")
        q_drop         = self._solve_ik_level_tip(arm, p_drop,         q_above_bucket, "drop_level")

        # Tilted-back versions for carrying the scoop
        q_above_scoop_carry  = self._with_carry_tilt(q_above_scoop)
        q_above_bucket_carry = self._with_carry_tilt(q_above_bucket)
        q_drop_carry         = self._with_carry_tilt(q_drop)

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

    def _wait_all_idle(self, max_wait: float = 50.0, query_period: float = 0.1) -> bool:
        # Allow all motor commands to be sent before queries
        time.sleep(0.5)    
        print('Waiting for motion to finish')
        start = time.time()
        next_query_time = time.time() + query_period  # start polling almost immediately after

        # ONE-TIME forced refresh — eliminates the race condition completely
        for motor_id in 'BRLXYZ':
            self.interpreter.query_motor(motor_id)
            time.sleep(0.022)  # ~115200 baud → 22 ms is safe for 3 lines response

        # Now your existing (excellent) loop runs with correct, fresh flags
        while not (self.interpreter.b_idle and self.interpreter.r_idle and
                self.interpreter.l_idle and self.interpreter.x_idle and
                self.interpreter.y_idle and self.interpreter.z_idle):

            now = time.time()
            if now >= next_query_time:
                # Only query the ones that are still busy
                for motor_id, attr in [('B','b'), ('R','r'), ('L','l'), ('X','x'), ('Y','y'), ('Z','z')]:
                    if not getattr(self.interpreter, f'{attr}_idle'):
                        self.interpreter.query_motor(motor_id)
                        # time.sleep(0.022)
                next_query_time = now + query_period

            if now - start >= max_wait:
                print("Timeout in _wait_all_idle")
                return False

            # print(self.interpreter.b_idle, self.interpreter.r_idle, self.interpreter.l_idle, 
            #       self.interpreter.x_idle, self.interpreter.y_idle, self.interpreter.z_idle)
            time.sleep(0.01)

        print(f"Motion complete\n")
        return True

    def _send_to_hardware(
        self,
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
        Concept: Adapts to interpreter's command structure; waits for idle to prevent overlap.
        """
        deltas_deg = target_q_deg - current_q_deg

        # Joint 1: base stepper, relative
        self.interpreter.send_joint_command(1, deltas_deg[0], speed)

        # Joints 2-3: wrist steppers, use composite for synchronization, relative
        self.interpreter.send_composite_command(deltas_deg[1], speed, deltas_deg[2], speed)

        # Joints 4-5: servos, absolute
        self.interpreter.send_joint_command(4, target_q_deg[3], speed)
        self.interpreter.send_joint_command(5, target_q_deg[4], speed)

        if gripper_angle is not None:
            self.interpreter.send_joint_command(6, gripper_angle, speed)

        self._wait_all_idle()

    def run_scoop(self):
        """
        Executes the scoop routine on hardware.
        Concept: Orchestrates waypoints from plan_waypoints; interpolates in radians for precision,
        converts to degrees for hardware; inserts gripper commands at key points. Waits for idle
        to ensure safe, sequential execution per industry real-time control standards.
        """
        if self.ser is None or self.interpreter is None:
            self.message_label.config(text="No connection.", foreground="red")
            return

        try:
            self.routine_status_label.config(text="Running...")
            self.message_label.config(text="Starting scoop routine...", foreground="blue")

            # Enable hold mode for torque during carry
            self.interpreter.enable_hold()

            # Get current angles (joints 1-5) from tracked state, convert to radians
            current_q_deg = np.array(self.interpreter.current_angles[0:5])
            current_q_rad = np.radians(current_q_deg)

            # Open gripper initially
            self._send_to_hardware(np.zeros(5), current_q_deg, DEFAULT_SPEED, gripper_angle=GRIPPER_OPEN)
            current_q_deg = np.zeros(5)  # Assume starts at home; adjust if needed

            # Plan waypoints (in radians, 5-DOF)
            waypoints_rad = self._plan_waypoints(self.arm)

            # Execute sequence
            for i in range(len(waypoints_rad) - 1):
                q_start_rad = waypoints_rad[i]
                q_end_rad = waypoints_rad[i + 1]
                print(f'\nWaypoint: ', i, np.degrees(q_end_rad))

                for j, q_rad in enumerate(self._interpolate(q_start_rad, q_end_rad)):
                    q_deg = np.degrees(q_rad)
                    # print(f'\tStep: ', j, q_deg)
                    self._send_to_hardware(q_deg, current_q_deg, DEFAULT_SPEED)
                    current_q_deg = q_deg

                # Insert gripper commands at specific points
                # After scoop_end (index 3)
                if i == 2:  # After moving to scoop_end
                    self._send_to_hardware(current_q_deg, current_q_deg, DEFAULT_SPEED, gripper_angle=GRIPPER_CLOSE)
                # At dump (index 7)
                elif i == 6:  # After moving to q_dump
                    self._send_to_hardware(current_q_deg, current_q_deg, DEFAULT_SPEED, gripper_angle=GRIPPER_OPEN)

            print('Finished scoop routine. Returning to user control.')

            self.routine_status_label.config(text="Idle")
            self.message_label.config(text="Scoop routine complete.", foreground="green")

        except Exception as e:
            print(f'Routine failed during waypoint {i}')
            self.message_label.config(text=f"Routine failed: {e}", foreground="red")
            self.routine_status_label.config(text="Error")
            # self.interpreter.disable_hold()  # Safety: release hold on error

        # self.interpreter.b_idle = True
        # self.interpreter.r_idle = True
        # self.interpreter.l_idle = True
        # self.interpreter.x_idle = True
        # self.interpreter.y_idle = True
        # self.interpreter.z_idle = True
        

if __name__ == "__main__":
    root = tk.Tk()
    app = JointControlUI(root)
    root.mainloop()
    if app.ser:
        app.ser.close()