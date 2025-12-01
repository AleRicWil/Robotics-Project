# ======================================================
# Joint_Control_UI.py - User Interface for Robot Arm Joint Control
# ======================================================

# This script creates a graphical user interface (GUI) using Tkinter for controlling the joints of a robot arm.
# It provides a row for each joint (1: Bottom, 2: Second, 3: Third), with an entry field for angular displacement
# in degrees (signed for direction), a green check mark button to send the command for that joint, and an entry
# field for angular speed in RPS (positive, defaults to 0.1). The RPS entry always displays the current value.
#
# Modernization Updates:
# - Utilized ttk for themed widgets to achieve a more contemporary look.
# - Applied the 'clam' theme for cleaner, rounded aesthetics where possible.
# - Used a consistent sans-serif font (Arial) with varying sizes for hierarchy (larger for labels).
# - Added frames with subtle borders and padding for each joint row to group elements visually.
# - Color-coded buttons: Negative presets in soft red (#FF6B6B), positive in soft green (#51CF66), connect button in blue (#4C8BF5),
#   check mark in green on white background for snappiness.
# - Increased entry widths and added focus highlights for better usability.
# - Message label with bold font and dynamic color for feedback.
# - Overall layout tightened with reduced padding for a "snappy" feel, while maintaining readability.
# - Creative touch: Subtle hover effects on buttons (change background on enter/leave) for interactivity,
#   and joint labels with bold styling to stand out.
# - Enhanced Background: Root background set to a light blue (#E3F2FD) for more color; joint frames to white (#FFFFFF) for contrast.
# - Alignment: Ensured consistent padx and widths for buttons/entries; used pack with consistent options for horizontal alignment.
# - Removed Parentheses: Joint labels simplified to "Joint 1", "Joint 2", "Joint 3" without descriptive terms in parentheses.

# Additional Features:
# - Auto-detection of active serial ports using serial.tools.list_ports.
# - Dropdown menu populated with active ports for user selection.
# - Automatic connection attempt to the first active port on startup.
# - "Connect" button for manual connection/reconnection using the selected port.
# - Persistent status indicator label ("Connected" in green or "Not Connected" in red).
# - A message label at the bottom for displaying success/error messages without blocking the UI (replaces modal message boxes).
# - Enter key binding: Pressing Enter in either the displacement or RPS entry field for a joint triggers the send_command
#   for that joint, mimicking the check mark button click. This follows industry UI conventions for form submission
#   (e.g., like in CAD software or robot control panels), improving usability without mouse dependency.
# - Preset Degree Buttons: On either side of the displacement entry, buttons for preset displacements (-360, -180, -90, -10, -5, -1 on left;
#   +1, +5, +10, +90, +180, +360 on right). Clicking immediately sends the command using the preset value as displacement and the current RPS,
#   without modifying the entry field. This enhances serviceability for quick, predefined movements in robot control, following conventions
#   in industrial HMI panels where presets allow rapid testing or homing without manual input.
#
# Key Concepts:
# - Serial Communication: Establishes a connection to the Arduino on the selected port at 115200 baud.
#   If the auto-connection fails, the user can select another port and click "Connect".
# - GUI Design: Uses Tkinter for a simple, cross-platform interface. Serial configuration is at the top,
#   followed by joint control rows. This modular approach enhances readability and serviceability.
# - Input Validation: Checks for valid numeric inputs to prevent errors, enhancing robustness.
# - Command Transmission: Sends motor commands for the selected joint followed by a newline (\n) to match the
#   Arduino's parsing logic.
# - Error Handling: Catches exceptions during serial operations and input parsing, displaying feedback via
#   the message label (non-blocking).
# - Defaults: RPS entries default to 0.1, as per requirements. The "current RPS" is always displayed in the entry
#   field, allowing users to view and modify it easily.
#
# Usage:
# - Run this script to launch the GUI.
# - If auto-connection succeeds, the status will show "Connected"; otherwise, select a port and click "Connect".
# - For each joint, enter displacement and speed (if different from default), use preset buttons for quick sends,
#   or click the green check mark / press Enter in an entry field to send custom values.
# - Ensure the Arduino is connected and running Motor_Control_v1.0.ino.
#
# Cutting-Edge Note: For more advanced UIs, consider integrating with ROS (Robot Operating System) for visualization
# and simulation, but here we stick to basic Tkinter for simplicity and educational value.

import tkinter as tk
from tkinter import ttk
import serial  # For serial communication with Arduino
import serial.tools.list_ports  # For auto-detecting active ports
from Robot_Interpreter import Motor_Map  # Import the mapping function

# ------------------------------------------------------
# Configuration Constants
# ------------------------------------------------------
BAUD_RATE = 115200    # Matches the Arduino's Serial.begin(115200)
TIMEOUT = 1           # Serial read timeout in seconds
DEFAULT_RPS = '0.05'   # Default RPS value for entries
NEGATIVE_DELTAS = [-360, -180, -90, -10, -5, -1]  # Presets for negative displacements (left side)
POSITIVE_DELTAS = [1, 5, 10, 90, 180, 360]        # Presets for positive displacements (right side)

# ------------------------------------------------------
# GUI Class
# ------------------------------------------------------
class JointControlUI:
    def __init__(self, root):
        """
        Initializes the GUI components and serial connection.
        
        Args:
            root (tk.Tk): The root window of the Tkinter application.
        """
        self.root = root
        self.root.title("Robot Arm Joint Control")
        self.root.geometry("1200x400")  # Slightly taller for better spacing
        self.root.configure(bg="#E3F2FD")  # Light blue background for more color
        
        # Set modern theme
        self.style = ttk.Style()
        self.style.theme_use('clam')
        
        # Customize styles
        self.style.configure('TLabel', font=('Arial', 12), background="#E3F2FD")
        self.style.configure('TButton', font=('Arial', 10), padding=6)
        self.style.configure('TEntry', font=('Arial', 12), padding=5)
        self.style.configure('TCombobox', font=('Arial', 10))
        self.style.map('TButton', background=[('active', '#E0E0E0')])
        self.style.configure('Joint.TFrame', background='#FFFFFF')  # White frames for contrast
        
        self.ser = None  # Initialize serial connection as None
        
        # Auto-detect available ports
        self.available_ports = [port.device for port in serial.tools.list_ports.comports()]
        
        # Serial configuration frame (top)
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
        
        # Dictionary to hold entries for each joint
        self.displacement_entries = {}
        self.speed_entries = {}
        
        # Create rows for each joint starting from row 1
        self.create_joint_row(1, "Joint 1", 1)
        self.create_joint_row(2, "Joint 2", 2)
        self.create_joint_row(3, "Joint 3", 3)
        
        # Message label for non-blocking feedback (row 4)
        self.message_label = ttk.Label(root, text="", foreground="black", font=('Arial', 12, 'italic'), background="#E3F2FD")
        self.message_label.grid(row=4, column=0, columnspan=20, pady=10, sticky='w')
        
        # Auto-attempt connection if ports are available
        if self.available_ports:
            self.connect_to_port()
        else:
            self.status_label.config(text="No active ports detected", foreground="red")
            self.message_label.config(text="No active ports detected. Please check connections.", foreground="red")
    
    def create_joint_row(self, joint: int, label_text: str, row: int):
        """
        Creates a GUI row for a specific joint with label, preset buttons on either side of the displacement entry,
        check button, and RPS entry. Binds the Enter key (<Return>) to both entries to trigger send_command for the joint,
        following UI conventions for keyboard-driven workflows in control software.
        
        Args:
            joint (int): The joint number.
            label_text (str): The label text for the joint.
            row (int): The grid row index for placement.
        """
        # Frame for the joint row with border
        joint_frame = ttk.Frame(self.root, padding=5, relief='ridge', borderwidth=1, style='Joint.TFrame')
        joint_frame.grid(row=row, column=0, columnspan=20, pady=5, sticky='ew')
        
        ttk.Label(joint_frame, text=label_text, font=('Arial', 14, 'bold')).pack(side='left', padx=10)
        
        # Negative preset buttons (left side)
        for delta in NEGATIVE_DELTAS:
            btn = ttk.Button(joint_frame, text=str(delta), width=5, command=lambda d=delta, j=joint: self.send_preset_command(j, d))
            self.style.configure('Neg.TButton', background='#FF6B6B', foreground='white')
            self.style.map('Neg.TButton', background=[('active', '#FF4757')])
            btn['style'] = 'Neg.TButton'
            btn.pack(side='left', padx=2)
        
        # Displacement entry
        disp_entry = ttk.Entry(joint_frame, width=12, font=('Arial', 12))
        disp_entry.pack(side='left', padx=5)
        disp_entry.bind("<Return>", lambda event, j=joint: self.send_command(j))  # Bind Enter to send
        self.displacement_entries[joint] = disp_entry
        
        # Positive preset buttons (right side)
        for delta in POSITIVE_DELTAS:
            btn = ttk.Button(joint_frame, text=f"+{delta}", width=5, command=lambda d=delta, j=joint: self.send_preset_command(j, d))
            self.style.configure('Pos.TButton', background='#51CF66', foreground='white')
            self.style.map('Pos.TButton', background=[('active', '#38D9A9')])
            btn['style'] = 'Pos.TButton'
            btn.pack(side='left', padx=2)
        
        # Green check mark button
        check_button = ttk.Button(joint_frame, text="✓", width=4, command=lambda j=joint: self.send_command(j))
        self.style.configure('Check.TButton', background='#2F9E44', foreground='white')
        self.style.map('Check.TButton', background=[('active', '#1B7837')])
        check_button['style'] = 'Check.TButton'
        check_button.pack(side='left', padx=5)
        
        # RPS entry with default
        speed_entry = ttk.Entry(joint_frame, width=8, font=('Arial', 12))
        speed_entry.insert(0, DEFAULT_RPS)  # Default to 0.1
        speed_entry.pack(side='left', padx=5)
        speed_entry.bind("<Return>", lambda event, j=joint: self.send_command(j))  # Bind Enter to send
        self.speed_entries[joint] = speed_entry
        
        # Label for RPS (for clarity)
        ttk.Label(joint_frame, text="RPS", font=('Arial', 12)).pack(side='left', padx=5)
    
    def send_preset_command(self, joint: int, displacement: int):
        """
        Sends a command for the given joint using the preset displacement value and the current RPS from the entry,
        without modifying the displacement entry field.
        
        Args:
            joint (int): The joint number.
            displacement (int): The preset displacement value to send.
        """
        if self.ser is None:
            self.message_label.config(text="No serial connection. Please connect first.", foreground="red")
            return
        
        try:
            speed = float(self.speed_entries[joint].get())
            
            # Generate motor commands using Motor_Map with preset displacement
            commands = Motor_Map(joint, displacement, speed)
            
            # Send each command to Arduino
            for cmd in commands:
                self.ser.write((cmd + '\n').encode())  # Encode to bytes and send with newline
                print(f"Sent preset for Joint {joint}: {cmd}")  # Debug print to console
            
            self.message_label.config(text=f"Preset command ({displacement}°) sent for Joint {joint}.", foreground="green")
        
        except ValueError as ve:
            self.message_label.config(text=f"Invalid RPS for Joint {joint}: {ve}", foreground="red")
        except Exception as e:
            self.message_label.config(text=f"Failed to send preset for Joint {joint}: {e}", foreground="red")
    
    def connect_to_port(self):
        """
        Attempts to connect to the selected serial port, updates the status label,
        and handles connection errors with non-blocking feedback.
        """
        port = self.port_var.get()
        if not port:
            self.message_label.config(text="No port selected or available.", foreground="red")
            return
        
        # Close existing connection if open
        if self.ser:
            self.ser.close()
            self.ser = None
        
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
            self.status_label.config(text="Connected", foreground="green")
            self.message_label.config(text=f"Connected to {port}.", foreground="green")
        except Exception as e:
            self.ser = None
            self.status_label.config(text="Not Connected", foreground="red")
            self.message_label.config(text=f"Connection failed: {str(e)}", foreground="red")
    
    def send_command(self, joint: int):
        """
        Handles the check button click or Enter key press for a specific joint: validates inputs, maps to motor commands,
        and sends them to the Arduino via serial if connected, with non-blocking feedback.
        
        Args:
            joint (int): The joint number to send the command for.
        """
        if self.ser is None:
            self.message_label.config(text="No serial connection. Please connect first.", foreground="red")
            return
        
        try:
            displacement = float(self.displacement_entries[joint].get())
            speed = float(self.speed_entries[joint].get())
            
            # Generate motor commands using Motor_Map
            commands = Motor_Map(joint, displacement, speed)
            
            # Send each command to Arduino
            for cmd in commands:
                self.ser.write((cmd + '\n').encode())  # Encode to bytes and send with newline
                print(f"Sent for Joint {joint}: {cmd}")  # Debug print to console
            
            self.message_label.config(text=f"Commands sent for Joint {joint}.", foreground="green")
        
        except ValueError as ve:
            self.message_label.config(text=f"Invalid input for Joint {joint}: {ve}", foreground="red")
        except Exception as e:
            self.message_label.config(text=f"Failed to send commands for Joint {joint}: {e}", foreground="red")

# ------------------------------------------------------
# Main Execution
# ------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = JointControlUI(root)
    root.mainloop()
    
    # Close serial connection on exit (good practice for resource management)
    if app.ser:
        app.ser.close()