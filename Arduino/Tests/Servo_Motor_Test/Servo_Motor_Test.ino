// SERVO CONTROL - TRIPLE SERVOS WITH FSM AND SPEED
// Controls three servos via serial commands for ID (A, B, C), signed degrees, positive RPS.
// Supports query ('Q ID') for current angle and speed (0 if idle).
// Supports independent enable/disable commands ('E ID', 'D ID') for holding torque without motion.
// v1.0: Basic position control with custom pulse mapping for 270° range.
// v1.1: Added speed control with non-blocking FSM and fixed 100ms updates.
//       - Simulates angular speed by incremental position updates.
//       - Speed in rotations per second (RPS); defaults to max if omitted.
//       - Follows industry convention: FSM for responsive multi-axis control without blocking.
// v1.2: Updated to use RPS for speed inputs; renamed 'velocity' to 'speed' for non-directionality.
//       - Added final position write in STOP state to ensure reaching the commanded angle.

// Pin Assignments
#define SERVO_A_PIN 9    // Servo A control pin
#define SERVO_B_PIN 10   // Servo B control pin
#define SERVO_C_PIN 11   // Servo C control pin

// Shared Constants
const int MIN_PULSE_US = 500;         // Minimum pulse width for 0° (from servo specs)
const int MAX_PULSE_US = 2500;        // Maximum pulse width for 270° (from servo specs)
const float MAX_ANGLE_DEG = 270.0f;   // Controlled running degree range
const unsigned long UPDATE_INTERVAL_MS = 100UL;  // Fixed update interval for non-blocking motion
const float MAX_SPEED_RPS = 1.0f;     // Conservative max speed (RPS) based on no-load specs (derated for safety)

// Enumerations
enum ServoState { SERVO_IDLE, SERVO_MOVING, SERVO_STOP };
enum ServoID { SERVO_A = 'A', SERVO_B = 'B', SERVO_C = 'C' };

// Include Libraries
#include <Servo.h>

// Servo State Structure
struct ServoMotor {
  float current_angle = 0.0f;
  float target_angle = 0.0f;
  float speed = 0.0f;  // In RPS (rotations per second)
  int direction = 1;   // 1 for positive, -1 for negative
  unsigned long last_update_ms = 0;
  bool is_moving = false;  // For query: true if speed > 0
};

// Global Instances
Servo servo_a, servo_b, servo_c;
ServoMotor motor_a, motor_b, motor_c;
ServoState state_a = SERVO_IDLE, state_b = SERVO_IDLE, state_c = SERVO_IDLE;

// Track enable states (attached or detached)
bool enabled_a = false, enabled_b = false, enabled_c = false;

// Function Declarations
void Servo_Setup();
void Servo_Start(char servo_id, float angle, float speed);
void Handle_Servo_FSM(ServoMotor* motor, ServoState* state, Servo* servo, char servo_id);
void Servo_Enable(char servo_id);
void Servo_Disable(char servo_id);
unsigned int Calculate_Pulse_us(float angle);
void Handle_Serial_Input();
void Handle_Query(char servo_id);

void setup() {
  Serial.begin(115200);
  Servo_Setup();
  delay(500);
  Serial.println("Enter 'ID degrees [speed]' or 'Q ID' or 'E ID' or 'D ID':");
}

void loop() {
  Handle_Serial_Input();
  Handle_Servo_FSM(&motor_a, &state_a, &servo_a, SERVO_A);
  Handle_Servo_FSM(&motor_b, &state_b, &servo_b, SERVO_B);
  Handle_Servo_FSM(&motor_c, &state_c, &servo_c, SERVO_C);
}

// Hardware Initialization
void Servo_Setup() {
  // Servos start detached to save power; enable on demand
  // Concept: Detaching servos when idle reduces heat and current draw, a common practice in battery-powered robotics.
}

// Core Controls
void Servo_Start(char servo_id, float angle, float speed) {
  ServoMotor* motor;
  Servo* servo;
  ServoState* state;
  bool* enabled;
  switch (servo_id) {
    case SERVO_A: motor = &motor_a; servo = &servo_a; state = &state_a; enabled = &enabled_a; break;
    case SERVO_B: motor = &motor_b; servo = &servo_b; state = &state_b; enabled = &enabled_b; break;
    case SERVO_C: motor = &motor_c; servo = &servo_c; state = &state_c; enabled = &enabled_c; break;
    default: return;
  }

  // Clamp angle to valid range (0 to 270°); no wrapping for position servos
  // Concept: Servos have physical limits; clamping prevents damage or erratic behavior.
  if (angle < 0.0f) angle = 0.0f;
  if (angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;

  float delta = angle - motor->current_angle;
  if (abs(delta) < 0.1f) {  // Small tolerance to ignore negligible moves
    Serial.println("Move too small; ignored.");
    return;
  }

  // Use default max if speed invalid or omitted
  if (speed <= 0.0f || speed > MAX_SPEED_RPS) {
    speed = MAX_SPEED_RPS;
    Serial.print("Speed set to max: ");
    Serial.println(speed, 3);
  }

  motor->target_angle = angle;
  motor->speed = speed;
  motor->direction = (delta > 0) ? 1 : -1;
  motor->last_update_ms = millis();
  motor->is_moving = true;

  if (!*enabled) {
    Servo_Enable(servo_id);
  }

  motor->last_update_ms = millis() - UPDATE_INTERVAL_MS;   // makes the very next loop tick immediately
  *state = SERVO_MOVING;
}

void Servo_Enable(char servo_id) {
  Servo* servo;
  int pin;
  bool* enabled;
  switch (servo_id) {
    case SERVO_A: servo = &servo_a; pin = SERVO_A_PIN; enabled = &enabled_a; break;
    case SERVO_B: servo = &servo_b; pin = SERVO_B_PIN; enabled = &enabled_b; break;
    case SERVO_C: servo = &servo_c; pin = SERVO_C_PIN; enabled = &enabled_c; break;
    default: return;
  }
  servo->attach(pin);
  *enabled = true;
  Serial.print(servo_id);
  Serial.println(" servo enabled.");
}

void Servo_Disable(char servo_id) {
  Servo* servo;
  bool* enabled;
  switch (servo_id) {
    case SERVO_A: servo = &servo_a; enabled = &enabled_a; break;
    case SERVO_B: servo = &servo_b; enabled = &enabled_b; break;
    case SERVO_C: servo = &servo_c; enabled = &enabled_c; break;
    default: return;
  }
  servo->detach();
  *enabled = false;
  Serial.print(servo_id);
  Serial.println(" servo disabled.");
}

void Handle_Servo_FSM(ServoMotor* motor, ServoState* state, Servo* servo, char servo_id) {
  switch (*state) {
    case SERVO_IDLE: break;

    case SERVO_MOVING: {
      if (millis() - motor->last_update_ms >= UPDATE_INTERVAL_MS) {
        // Calculate step size based on speed and interval
        // Concept: Fixed-time updates ensure non-blocking operation; step = (speed RPS * 360 deg/rot) * dt for linear speed profile.
        float dt_sec = UPDATE_INTERVAL_MS / 1000.0f;
        float speed_deg_s = motor->speed * 360.0f;
        float step = speed_deg_s * dt_sec;

        float new_angle = motor->current_angle + step * motor->direction;

        // Check if we've reached or passed the target
        // Concept: Overshoot prevention by clamping to target, ensuring precise positioning.
        bool reached = (motor->direction > 0 && new_angle >= motor->target_angle) ||
                       (motor->direction < 0 && new_angle <= motor->target_angle);

        if (reached) {
          new_angle = motor->target_angle;
          motor->is_moving = false;
          *state = SERVO_STOP;
        }

        motor->current_angle = new_angle;
        unsigned int pulse_us = Calculate_Pulse_us(new_angle);
        servo->writeMicroseconds(pulse_us);
        motor->last_update_ms = millis();
      }
      break;
    }

    case SERVO_STOP: {
      // Send the original target position one last time to ensure arrival
      // Concept: Redundant write accounts for any potential servo drift or timing issues, common in position control.
      unsigned int pulse_us = Calculate_Pulse_us(motor->target_angle);
      servo->writeMicroseconds(pulse_us);

      // Keep enabled for holding torque; disable manually if needed
      // Concept: In robotics, holding position post-move is common for stability.
      Serial.print(servo_id);
      Serial.println(" servo stopped.");
      *state = SERVO_IDLE;
      break;
    }
  }
}

// Input Handling
void Handle_Serial_Input() {
  static String input = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.startsWith("Q ")) {
        char servo_id = input.charAt(2);
        if (servo_id == 'A' || servo_id == 'B' || servo_id == 'C') {
          Handle_Query(servo_id);
        } else {
          Serial.println("Invalid query ID.");
        }
      } else if (input.startsWith("E ")) {
        char servo_id = input.charAt(2);
        if (servo_id == 'A' || servo_id == 'B' || servo_id == 'C') {
          Servo_Enable(servo_id);
        } else {
          Serial.println("Invalid enable ID.");
        }
      } else if (input.startsWith("D ")) {
        char servo_id = input.charAt(2);
        if (servo_id == 'A' || servo_id == 'B' || servo_id == 'C') {
          Servo_Disable(servo_id);
        } else {
          Serial.println("Invalid disable ID.");
        }
      } else {
        int first = input.indexOf(' ');
        if (first == -1) {
          Serial.println("Invalid format.");
          input = "";
          return;
        }
        char servo_id = input.substring(0, first).charAt(0);
        if (servo_id != 'A' && servo_id != 'B' && servo_id != 'C') {
          Serial.println("Invalid servo ID.");
          input = "";
          return;
        }
        String rest = input.substring(first + 1);
        int second = rest.indexOf(' ');
        float speed = 0.0f;  // 0 means use default
        float angle;
        if (second == -1) {
          angle = rest.toFloat();
        } else {
          String angle_str = rest.substring(0, second);
          String speed_str = rest.substring(second + 1);
          angle = angle_str.toFloat();
          speed = speed_str.toFloat();
        }
        Servo_Start(servo_id, angle, speed);
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void Handle_Query(char servo_id) {
  ServoMotor* motor;
  switch (servo_id) {
    case SERVO_A: motor = &motor_a; break;
    case SERVO_B: motor = &motor_b; break;
    case SERVO_C: motor = &motor_c; break;
    default: Serial.println("Invalid ID."); return;
  }
  Serial.print(servo_id);
  Serial.print(" current angle: ");
  Serial.println(motor->current_angle, 1);
  Serial.print(servo_id);
  Serial.print(" current speed: ");
  Serial.println(motor->is_moving ? motor->speed : 0.0f, 3);
}

// Helpers
unsigned int Calculate_Pulse_us(float angle) {
  // Linear interpolation from 0° to 270° -> MIN_PULSE_US to MAX_PULSE_US
  // Concept: Pulse width modulation (PWM) for servo position control.
  // The servo interprets pulse widths between 500us and 2500us as positions from 0° to 270°.
  if (angle < 0.0f) angle = 0.0f;
  if (angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
  return MIN_PULSE_US + (unsigned int)((angle / MAX_ANGLE_DEG) * (MAX_PULSE_US - MIN_PULSE_US));
}