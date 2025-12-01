# ======================================================
# Robot_Interpreter.py - Joint to Motor Command Mapper
# ======================================================

# This script provides a mapping function to translate high-level joint commands
# (joint number, angular displacement in degrees, and angular speed in RPS) into
# low-level motor commands suitable for sending to the Arduino running
# Motor_Control_v1.0.ino.
#
# Key Concepts:
# - The robot arm has three rotary joints:
#   - Joint 1: Bottom turntable, directly driven by motor 'B' with a gear reduction.
#   - Joint 2: Second joint (e.g., pitch/elevation), controlled by motors 'R' and 'L'
#     moving in the same direction via a differential mechanism.
#   - Joint 3: Third joint (e.g., yaw/azimuth), controlled by motors 'R' and 'L'
#     moving in opposite directions via the differential mechanism.
# - Gear ratios are defined as constants. These represent the reduction factor G,
#   where motor_angle = joint_angle * G, and motor_rps = joint_rps * G.
#   Adjust these based on your hardware specifications (e.g., gear teeth ratios).
# - Direction is encoded in the sign of the displacement (positive: clockwise/HIGH,
#   negative: counterclockwise/LOW, depending on wiring).
# - Angular speed (RPS) is assumed positive; direction comes from displacement sign.
# - Commands are formatted as strings like 'B 180 1.0', where degrees are rounded
#   to integers (as Arduino parses them with toInt()). For fractional degrees,
#   consider updating Arduino to use toFloat() for degrees if higher precision needed.
# - This follows industry conventions: modular functions, clear constants, error
#   handling, and comments for maintainability. For cutting-edge, we could integrate
#   inverse kinematics later, but here we stick to tried-and-true direct mapping.

# ------------------------------------------------------
# Configuration Constants
# ------------------------------------------------------
G_B = 5.0   # Gear ratio for bottom motor 'B' (motor turns per joint turn). Example: 5:1 reduction.
G_RL = 7.8 # Gear ratio for right/left motors 'R'/'L' in the differential (identical for both).

# ------------------------------------------------------
# Mapping Function
# ------------------------------------------------------
def Motor_Map(joint: int, displacement: float, speed: float) -> list[str]:
    """
    Maps a joint command to motor commands.

    Args:
        joint (int): Joint number (1: bottom, 2: second/differential opp-dir, 3: third/differential same-dir).
        displacement (float): Angular displacement in degrees for the joint (signed for direction).
        speed (float): Angular speed in revolutions per second (RPS) for the joint (positive).

    Returns:
        list[str]: List of command strings to send to Arduino, e.g., ['B 180 1.0'].

    Raises:
        ValueError: If invalid joint number or negative speed.

    Example:
        >>> Motor_Map(1, 90.0, 0.5)
        ['B 450 2.5']  # Assuming G_B=5.0
        >>> Motor_Map(2, -45.0, 1.0)
        ['R -450 10.0', 'L -450 10.0']  # Assuming G_RL=10.0
    """
    if joint not in [1, 2, 3]:
        raise ValueError("Invalid joint number. Must be 1, 2, or 3.")
    if speed <= 0.0:
        raise ValueError("Speed must be positive. Direction is from displacement sign.")

    commands = []

    if joint == 1:
        # Bottom joint: Direct mapping to motor 'B' with gear ratio.
        motor_deg = round(displacement * G_B)  # Round to int for Arduino toInt()
        motor_rps = speed * G_B
        commands.append(f'B {motor_deg} {motor_rps:.3f}')  # Format RPS to 3 decimals for precision

    elif joint == 2:
        # Second joint: 'R' and 'L' move in opposite directions.
        # Differential: delta_R = displacement * G_RL, delta_L = -displacement * G_RL
        motor_deg_R = round(displacement * G_RL)
        motor_deg_L = round(-displacement * G_RL)
        motor_rps = speed * G_RL
        commands.append(f'R {motor_deg_R} {motor_rps:.3f}')
        commands.append(f'L {motor_deg_L} {motor_rps:.3f}')
        

    elif joint == 3:
        # Third joint: 'R' and 'L' move in same direction.
        # Differential: delta_R = displacement * G_RL, delta_L = displacement * G_RL
        motor_deg_R = round(-displacement * G_RL)
        motor_deg_L = round(-displacement * G_RL)
        motor_rps = speed * G_RL
        commands.append(f'R {motor_deg_R} {motor_rps:.3f}')
        commands.append(f'L {motor_deg_L} {motor_rps:.3f}')

    return commands

# ======================================================
# Usage Example (for testing; remove in production)
# ======================================================
if __name__ == "__main__":
    # Test Joint 1
    print(Motor_Map(1, 90.0, 0.5))  # Expected: ['B 450 2.5'] with G_B=5.0

    # Test Joint 2
    print(Motor_Map(2, -45.0, 1.0))  # Expected: ['R -450 10.0', 'L -450 10.0'] with G_RL=10.0

    # Test Joint 3
    print(Motor_Map(3, 30.0, 0.75))  # Expected: ['R 300 7.5', 'L -300 7.5'] with G_RL=10.0