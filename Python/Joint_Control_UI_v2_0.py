# Joint_Control_UI_v1.4.py - GUI for Robot Arm Joint/Servo Control
# v1.4
# Added rows for servo joints 4 (X), 5 (Y), 6 (Z).
# - For servos: entry sets absolute angle (0-270°); presets are relative increments (query current, add delta, clamp, send absolute).
# - Query shows angle/speed for servos, steps/RPS for steppers.
# - Updated defaults and labels for clarity.
# - Concept: Unified UI for all actuators; differentiates behavior based on type for usability.

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
from Robot_Interpreter_v2_0 import RobotInterpreter, MAX_SERVO_ANGLE

# Constants
BAUD_RATE = 115200
TIMEOUT = 1
DEFAULT_RPS = '0.05'
NEGATIVE_DELTAS = [-360, -180, -90, -10, -5, -1]
POSITIVE_DELTAS = [1, 5, 10, 90, 180, 360]
SERVO_JOINTS = [4, 5, 6]  # Joints using servos. 6 is actually the gripper. Called joint for convenience

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

        # Create rows for all joints
        self.create_joint_row(1, "Joint 1 (Base Stepper)", 1)
        self.create_joint_row(2, "Joint 2 (Wrist Stepper Diff)", 2)
        self.create_joint_row(3, "Joint 3 (Wrist Stepper Common)", 3)
        self.create_joint_row(4, "Joint 4 (Servo X)", 4)
        self.create_joint_row(5, "Joint 5 (Servo Y)", 5)
        self.create_joint_row(6, "Gripper (Servo Z)", 6)

        self.create_composite_section(7)

        self.message_label = ttk.Label(root, text="", foreground="black", font=('Arial', 12, 'italic'), background="#E3F2FD")
        self.message_label.grid(row=8, column=0, columnspan=20, pady=10, sticky='w')

        if self.available_ports:
            self.connect_to_port()
        else:
            self.status_label.config(text="No active ports", foreground="red")
            self.message_label.config(text="No ports detected.", foreground="red")

    def create_joint_row(self, joint: int, label_text: str, row: int):
        """
        Creates UI row for a joint/servo.
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

    def send_preset_command(self, joint: int, delta: float):
        """
        Sends preset delta; for servos, queries current, computes new absolute, clamps.
        For steppers, sends as displacement.
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
                current = self.interpreter.get_current_angle(joint)
                target = max(0.0, min(MAX_SERVO_ANGLE, current + delta))
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
            self.ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
            self.interpreter = RobotInterpreter(self.ser)
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

if __name__ == "__main__":
    root = tk.Tk()
    app = JointControlUI(root)
    root.mainloop()
    if app.ser:
        app.ser.close()