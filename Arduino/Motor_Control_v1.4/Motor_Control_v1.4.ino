// MOTOR CONTROL - TRIPLE STEPPERS AND TRIPLE SERVOS WITH FSM AND RAMPING/SPEED
// Controls three stepper motors (IDs: R, L, B) and three servos (IDs: X, Y, Z) via serial commands for ID, signed degrees, positive RPS.
// Supports query ('Q ID') for steppers: remaining steps and RPS; for servos: current angle and speed (0 if idle).
// Supports independent enable/disable commands ('E ID', 'D ID') for holding torque without motion.
// v1.3: Simplified holding logic to global enable/disable for steppers.
//       - Enable ALL motors when ANY starts moving.
//       - Disable ALL only when ALL are idle.
//       - Removes per-motor disable in STOP state; uses global check in loop for efficiency.
//       - Follows industry convention: global enables reduce drift in multi-axis systems without overhead.
// v1.4: Incorporated servo control from Servo_Motor_Test, with servo IDs A, D, C (changed B to D to avoid conflict with stepper B).
//       - Servos use non-blocking FSM for speed simulation via incremental updates.
//       - Unified serial input handling to dispatch to stepper or servo based on ID.
//       - Servos have individual enable/disable; steppers use global.
//       - Added per-servo constants for mixed types (9g on D with 180° range).
//       - Per-servo max_angle and max_speed_rps in struct for flexibility.
//       - Updated clamping and pulse calculation to use per-servo limits.
//       - Concept: Supports heterogeneous servos; ensures code readability and serviceability by localizing limits.
//       - Changed servo IDs to X,Y,Z (X,Y=20kg, Z=9g); ordered as X,Y,Z for consistency.

// Pin Assignments (Steppers)
#define STEPB_DIS_PIN 4    // Bottom DISABLE
#define STEPB_STEP_PIN 3   // Bottom STEP
#define STEPB_DIR_PIN 2    // Bottom DIR

#define STEPR_STEP_PIN 5    // Right STEP
#define STEPR_DIR_PIN 6     // Right DIR
#define STEPR_DIS_PIN 7    // Right DISABLE

#define STEPL_STEP_PIN 13   // Left STEP
#define STEPL_DIR_PIN 12    // Left DIR
#define STEPL_DIS_PIN 8    // Left DISABLE

// Pin Assignments (Servos)
#define SERVO_X_PIN 9    // Servo X control pin
#define SERVO_Y_PIN 10   // Servo Y control pin
#define SERVO_Z_PIN 11   // Servo Z control pin

// Shared Constants (Steppers)
const float MAX_COLD_START_RPS = 0.001f;
const float STEP_RAMP_ACCEL = 1.0f;
const long LARGE_TEMP_COUNT = 2147483647L;

// Stepper Constants
const int STEPS_PER_REV = 200 * 4;
const float STEP_MAX_RPS = 1.56;

// Shared Constants (Servos)
// Constants for 20kg servos (X, Y)
const float MAX_ANGLE_DEG_20KG = 270.0f;
const float MAX_SPEED_RPS_20KG = 1.0f;  // Derated from ~1.19 RPS at 7.2V for safety

// Constants for 9g servo (Z)
const float MAX_ANGLE_DEG_9G = 180.0f;
const float MAX_SPEED_RPS_9G = 1.0f;  // Derated from ~1.39 RPS at 4.8V for safety

// Shared pulse widths (assuming standard 500-2500us for both; confirm with testing to avoid damage)
const int MIN_PULSE_US = 500;         // Minimum pulse width for 0° (from servo specs)
const int MAX_PULSE_US = 2500;        // Maximum pulse width for max_angle° (from servo specs)

const unsigned long UPDATE_INTERVAL_MS = 100UL;  // Fixed update interval for non-blocking motion

// Enumerations (Steppers)
enum MotorState { MOTOR_IDLE, MOTOR_START, MOTOR_RAMP, MOTOR_RUN, MOTOR_DECEL, MOTOR_STOP };
enum MotorID { MOTOR_R = 'R', MOTOR_L = 'L', MOTOR_B = 'B' };

// Enumerations (Servos)
enum ServoState { SERVO_IDLE, SERVO_MOVING, SERVO_STOP };
enum ServoID { SERVO_X = 'X', SERVO_Y = 'Y', SERVO_Z = 'Z' };

// Include Libraries
#include <Servo.h>

// Stepper State Structure
struct StepperMotor {
  volatile long steps_remaining = 0;
  unsigned long step_interval_us = 1000;
  int direction = HIGH;
  float current_rps = 0.0f;
  float target_rps = 0.0f;
  unsigned long ramp_start_time_ms = 0;
  unsigned long last_step_time_us = 0;
  float ramp_start_rps = 0.0f;
  long blend_steps = 0;
  float blend_target_rps = 0.0f;
  int blend_direction = HIGH;
  bool is_blending = false;
};

// Servo State Structure
struct ServoMotor {
  float current_angle = 0.0f;
  float target_angle = 0.0f;
  float speed = 0.0f;  // In RPS (rotations per second)
  int direction = 1;   // 1 for positive, -1 for negative
  unsigned long last_update_ms = 0;
  bool is_moving = false;  // For query: true if speed > 0
  // New: per-servo limits for flexibility in mixed systems
  float max_angle;
  float max_speed_rps;
};

// Global Instances (Steppers)
StepperMotor motor_right, motor_left, motor_bottom;
MotorState state_right = MOTOR_IDLE, state_left = MOTOR_IDLE, state_bottom = MOTOR_IDLE;

// Global Instances (Servos)
Servo servo_x, servo_y, servo_z;
ServoMotor motor_x, motor_y, motor_z;
ServoState state_x = SERVO_IDLE, state_y = SERVO_IDLE, state_z = SERVO_IDLE;

// Track enable states (Servos, individual)
bool enabled_x = false, enabled_y = false, enabled_z = false;

// Track last all-idle state for global enable/disable (Steppers, low overhead)
bool all_motors_idle = true;

// Function Declarations (Steppers)
void Stepper_Setup();
void Motor_Start(char motor_id, long steps, float rps, int direction);
void Motor_Disable(char motor_id);
void Stepper_Timed_Step(char motor_id);
void Stepper_Set_Direction(char motor_id, int direction);
void Stepper_Enable(char motor_id);
void Stepper_Init_Cold_Start(StepperMotor* motor);
void Stepper_Init_Ramp(StepperMotor* motor);
unsigned long Stepper_Calculate_Interval_us(float rps);
long Calculate_Decel_Steps(float rps);
void Handle_Stepper_Query(char motor_id);
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id);
void Handle_Active_Motors();
void Enable_All_Motors();
void Disable_All_Motors();

// Function Declarations (Servos)
void Servo_Setup();
void Servo_Start(char servo_id, float angle, float speed);
void Handle_Servo_FSM(ServoMotor* motor, ServoState* state, Servo* servo, char servo_id);
void Servo_Enable(char servo_id);
void Servo_Disable(char servo_id);
unsigned int Calculate_Pulse_us(ServoMotor* motor, float angle);
void Handle_Servo_Query(char servo_id);

// Shared
void Handle_Serial_Input();

void setup() {
  Serial.begin(115200);
  Stepper_Setup();
  Servo_Setup();
  Disable_All_Motors();  // Start with steppers disabled
  delay(500);
  Serial.println("Enter 'ID degrees [speed]' or 'Q ID' or 'E ID' or 'D ID': (IDs: X,Y,Z for servos; R,L,B for steppers)");
}

void loop() {
  Handle_Serial_Input();

  // Stepper FSMs and active handling
  Handle_Stepper_FSM(&motor_right, &state_right, MOTOR_R);
  Handle_Stepper_FSM(&motor_left, &state_left, MOTOR_L);
  Handle_Stepper_FSM(&motor_bottom, &state_bottom, MOTOR_B);
  Handle_Active_Motors();

  // Servo FSMs
  Handle_Servo_FSM(&motor_x, &state_x, &servo_x, SERVO_X);
  Handle_Servo_FSM(&motor_y, &state_y, &servo_y, SERVO_Y);
  Handle_Servo_FSM(&motor_z, &state_z, &servo_z, SERVO_Z);

  // Global holding check for steppers
  bool current_all_idle = (state_right == MOTOR_IDLE && state_left == MOTOR_IDLE && state_bottom == MOTOR_IDLE);
  if (!current_all_idle && all_motors_idle) {
    Enable_All_Motors();
    all_motors_idle = false;
  } else if (current_all_idle && !all_motors_idle) {
    Disable_All_Motors();
    all_motors_idle = true;
  }
}

// Hardware Initialization (Steppers)
void Stepper_Setup() {
  pinMode(STEPR_STEP_PIN, OUTPUT);
  pinMode(STEPR_DIR_PIN, OUTPUT);
  pinMode(STEPR_DIS_PIN, OUTPUT);
  digitalWrite(STEPR_DIS_PIN, HIGH);

  pinMode(STEPL_STEP_PIN, OUTPUT);
  pinMode(STEPL_DIR_PIN, OUTPUT);
  pinMode(STEPL_DIS_PIN, OUTPUT);
  digitalWrite(STEPL_DIS_PIN, HIGH);

  pinMode(STEPB_STEP_PIN, OUTPUT);
  pinMode(STEPB_DIR_PIN, OUTPUT);
  pinMode(STEPB_DIS_PIN, OUTPUT);
  digitalWrite(STEPB_DIS_PIN, HIGH);
}

// Hardware Initialization (Servos)
void Servo_Setup() {
  // Servos start detached to save power; enable on demand
  // Concept: Detaching servos when idle reduces heat and current draw, a common practice in battery-powered robotics.

  // Set per-servo limits
  // Concept: Allows mixing servo types; here, X and Y are 20kg (270°), Z is 9g (180°).
  // Ensure power supply <=6V to protect 9g servo (max 6V vs. 7.2V for 20kg).
  motor_x.max_angle = MAX_ANGLE_DEG_20KG;
  motor_x.max_speed_rps = MAX_SPEED_RPS_20KG;
  motor_y.max_angle = MAX_ANGLE_DEG_20KG;
  motor_y.max_speed_rps = MAX_SPEED_RPS_20KG;
  motor_z.max_angle = MAX_ANGLE_DEG_9G;
  motor_z.max_speed_rps = MAX_SPEED_RPS_9G;
}

// NEW: Enable all motors (LOW on disable pins) (Steppers)
void Enable_All_Motors() {
  Stepper_Enable(MOTOR_R);
  Stepper_Enable(MOTOR_L);
  Stepper_Enable(MOTOR_B);
}

// NEW: Disable all motors (HIGH on disable pins) (Steppers)
void Disable_All_Motors() {
  Motor_Disable(MOTOR_R);
  Motor_Disable(MOTOR_L);
  Motor_Disable(MOTOR_B);
}

// Core Controls (Steppers)
void Motor_Start(char motor_id, long steps, float rps, int direction) {
  if (steps <= 0 || rps <= 0.0f) {
    Serial.println("Invalid input.");
    return;
  }

  StepperMotor* motor;
  MotorState* state;
  switch (motor_id) {
    case MOTOR_R: motor = &motor_right; state = &state_right; break;
    case MOTOR_L: motor = &motor_left; state = &state_left; break;
    case MOTOR_B: motor = &motor_bottom; state = &state_bottom; break;
    default: return;
  }

  motor->steps_remaining = steps;
  motor->direction = direction;
  motor->target_rps = rps;

  Stepper_Set_Direction(motor_id, motor->direction);

  if (motor->target_rps <= MAX_COLD_START_RPS) {
    Stepper_Init_Cold_Start(motor);
    *state = MOTOR_RUN;
  } else {
    Stepper_Init_Ramp(motor);
    *state = MOTOR_RAMP;
  }

  motor->last_step_time_us = micros();
}

void Motor_Disable(char motor_id) {
  int dis_pin;
  switch (motor_id) {
    case MOTOR_R: dis_pin = STEPR_DIS_PIN; break;
    case MOTOR_L: dis_pin = STEPL_DIS_PIN; break;
    case MOTOR_B: dis_pin = STEPB_DIS_PIN; break;
    default: return;
  }
  digitalWrite(dis_pin, HIGH);
}

void Stepper_Timed_Step(char motor_id) {
  StepperMotor* motor;
  int step_pin;
  switch (motor_id) {
    case MOTOR_R: motor = &motor_right; step_pin = STEPR_STEP_PIN; break;
    case MOTOR_L: motor = &motor_left; step_pin = STEPL_STEP_PIN; break;
    case MOTOR_B: motor = &motor_bottom; step_pin = STEPB_STEP_PIN; break;
    default: return;
  }

  if (micros() - motor->last_step_time_us >= motor->step_interval_us) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(step_pin, LOW);
    motor->last_step_time_us = micros();
    motor->steps_remaining--;
  }
}

void Stepper_Set_Direction(char motor_id, int direction) {
  int dir_pin;
  switch (motor_id) {
    case MOTOR_R: dir_pin = STEPR_DIR_PIN; break;
    case MOTOR_L: dir_pin = STEPL_DIR_PIN; break;
    case MOTOR_B: dir_pin = STEPB_DIR_PIN; break;
    default: return;
  }
  digitalWrite(dir_pin, direction);
}

void Stepper_Enable(char motor_id) {
  int dis_pin;
  switch (motor_id) {
    case MOTOR_R: dis_pin = STEPR_DIS_PIN; break;
    case MOTOR_L: dis_pin = STEPL_DIS_PIN; break;
    case MOTOR_B: dis_pin = STEPB_DIS_PIN; break;
    default: return;
  }
  digitalWrite(dis_pin, LOW);
}

void Stepper_Init_Cold_Start(StepperMotor* motor) {
  motor->current_rps = motor->target_rps;
  motor->step_interval_us = Stepper_Calculate_Interval_us(motor->target_rps);
}

void Stepper_Init_Ramp(StepperMotor* motor) {
  motor->current_rps = MAX_COLD_START_RPS;
  motor->step_interval_us = Stepper_Calculate_Interval_us(MAX_COLD_START_RPS);
  motor->ramp_start_time_ms = millis();
  motor->ramp_start_rps = MAX_COLD_START_RPS;
}

unsigned long Stepper_Calculate_Interval_us(float rps) {
  if (rps <= 0.0f) return 0xFFFFFFFFUL;
  return 1000000UL / (rps * STEPS_PER_REV);
}

long Calculate_Decel_Steps(float rps) {
  if (rps <= 0.0f) return 0;
  float revs_decel = (rps * rps) / (2.0f * 2 * STEP_RAMP_ACCEL);
  return (long)ceil(revs_decel * STEPS_PER_REV);
}

void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id) {
  switch (*state) {
    case MOTOR_IDLE: break;
    case MOTOR_START: break;
    case MOTOR_RAMP: {
      float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
      float new_rps = motor->ramp_start_rps + STEP_RAMP_ACCEL * elapsed_sec;
      if (new_rps >= motor->target_rps) {
        new_rps = motor->target_rps;
        *state = MOTOR_RUN;
      }
      if (new_rps != motor->current_rps) {
        motor->current_rps = new_rps;
        motor->step_interval_us = Stepper_Calculate_Interval_us(new_rps);
      }
      long decel_steps = Calculate_Decel_Steps(motor->current_rps);
      if (motor->steps_remaining <= decel_steps) {
        motor->target_rps = 0.0f;
        motor->ramp_start_rps = motor->current_rps;
        motor->ramp_start_time_ms = millis();
        *state = MOTOR_DECEL;
      }
      break;
    }
    case MOTOR_RUN: {
      long decel_steps = Calculate_Decel_Steps(motor->current_rps);
      if (motor->steps_remaining <= decel_steps) {
        motor->target_rps = 0.0f;
        motor->ramp_start_rps = motor->current_rps;
        motor->ramp_start_time_ms = millis();
        *state = MOTOR_DECEL;
      }
      break;
    }
    case MOTOR_DECEL: {
      float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
      float new_rps = motor->ramp_start_rps - 2 * STEP_RAMP_ACCEL * elapsed_sec;
      if (new_rps < 0.0f) new_rps = 0.0f;
      if (new_rps <= motor->target_rps) {
        new_rps = motor->target_rps;
        if (motor->is_blending) {
          long decel_count = LARGE_TEMP_COUNT - motor->steps_remaining;
          Stepper_Set_Direction(motor_id, motor->blend_direction);
          motor->steps_remaining = motor->blend_steps + decel_count;
          motor->target_rps = motor->blend_target_rps;
          motor->direction = motor->blend_direction;
          motor->is_blending = false;
          if (motor->target_rps <= MAX_COLD_START_RPS) {
            motor->current_rps = motor->target_rps;
            motor->step_interval_us = Stepper_Calculate_Interval_us(motor->target_rps);
            *state = MOTOR_RUN;
          } else {
            motor->current_rps = MAX_COLD_START_RPS;
            motor->step_interval_us = Stepper_Calculate_Interval_us(MAX_COLD_START_RPS);
            motor->ramp_start_rps = MAX_COLD_START_RPS;
            motor->ramp_start_time_ms = millis();
            *state = MOTOR_RAMP;
          }
          motor->last_step_time_us = micros();
        } else if (motor->target_rps == 0.0f) {
          *state = MOTOR_STOP;
        } else {
          *state = MOTOR_RUN;
        }
      }
      if (new_rps != motor->current_rps) {
        motor->current_rps = new_rps;
        motor->step_interval_us = Stepper_Calculate_Interval_us(new_rps);
      }
      break;
    }
    case MOTOR_STOP:
      Serial.print(motor_id);
      Serial.println(" motor stopped.");
      *state = MOTOR_IDLE;
      break;
  }
}

void Handle_Active_Motors() {
  if (state_right == MOTOR_RAMP || state_right == MOTOR_RUN || state_right == MOTOR_DECEL) {
    if (motor_right.steps_remaining > 0) Stepper_Timed_Step(MOTOR_R);
    else state_right = MOTOR_STOP;
  }
  if (state_left == MOTOR_RAMP || state_left == MOTOR_RUN || state_left == MOTOR_DECEL) {
    if (motor_left.steps_remaining > 0) Stepper_Timed_Step(MOTOR_L);
    else state_left = MOTOR_STOP;
  }
  if (state_bottom == MOTOR_RAMP || state_bottom == MOTOR_RUN || state_bottom == MOTOR_DECEL) {
    if (motor_bottom.steps_remaining > 0) Stepper_Timed_Step(MOTOR_B);
    else state_bottom = MOTOR_STOP;
  }
}

// Core Controls (Servos)
void Servo_Start(char servo_id, float angle, float speed) {
  ServoMotor* motor;
  Servo* servo;
  ServoState* state;
  bool* enabled;
  int pin;
  switch (servo_id) {
    case SERVO_X: motor = &motor_x; servo = &servo_x; state = &state_x; enabled = &enabled_x; pin = SERVO_X_PIN; break;
    case SERVO_Y: motor = &motor_y; servo = &servo_y; state = &state_y; enabled = &enabled_y; pin = SERVO_Y_PIN; break;
    case SERVO_Z: motor = &motor_z; servo = &servo_z; state = &state_z; enabled = &enabled_z; pin = SERVO_Z_PIN; break;
    default: return;
  }

  // Clamp angle to valid range (0 to motor->max_angle); no wrapping for position servos
  // Concept: Servos have physical limits; clamping prevents damage or erratic behavior.
  if (angle < 0.0f) angle = 0.0f;
  if (angle > motor->max_angle) angle = motor->max_angle;

  float delta = angle - motor->current_angle;
  if (abs(delta) < 0.1f) {  // Small tolerance to ignore negligible moves
    Serial.println("Move too small; ignored.");
    return;
  }

  // Use default max if speed invalid or omitted
  if (speed <= 0.0f || speed > motor->max_speed_rps) {
    speed = motor->max_speed_rps;
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
    case SERVO_X: servo = &servo_x; pin = SERVO_X_PIN; enabled = &enabled_x; break;
    case SERVO_Y: servo = &servo_y; pin = SERVO_Y_PIN; enabled = &enabled_y; break;
    case SERVO_Z: servo = &servo_z; pin = SERVO_Z_PIN; enabled = &enabled_z; break;
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
    case SERVO_X: servo = &servo_x; enabled = &enabled_x; break;
    case SERVO_Y: servo = &servo_y; enabled = &enabled_y; break;
    case SERVO_Z: servo = &servo_z; enabled = &enabled_z; break;
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
        unsigned int pulse_us = Calculate_Pulse_us(motor, new_angle);
        servo->writeMicroseconds(pulse_us);
        motor->last_update_ms = millis();
      }
      break;
    }

    case SERVO_STOP: {
      // Send the original target position one last time to ensure arrival
      // Concept: Redundant write accounts for any potential servo drift or timing issues, common in position control.
      unsigned int pulse_us = Calculate_Pulse_us(motor, motor->target_angle);
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

unsigned int Calculate_Pulse_us(ServoMotor* motor, float angle) {
  // Linear interpolation from 0° to motor->max_angle -> MIN_PULSE_US to MAX_PULSE_US
  // Concept: Pulse width modulation (PWM) for servo position control.
  // The servo interprets pulse widths between 500us and 2500us as positions from 0° to max_angle.
  if (angle < 0.0f) angle = 0.0f;
  if (angle > motor->max_angle) angle = motor->max_angle;
  return MIN_PULSE_US + (unsigned int)((angle / motor->max_angle) * (MAX_PULSE_US - MIN_PULSE_US));
}

// Input Handling (Unified)
void Handle_Serial_Input() {
  static String input = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.startsWith("Q ")) {
        char id = input.charAt(2);
        if (id == 'X' || id == 'Y' || id == 'Z') {
          Handle_Servo_Query(id);
        } else if (id == 'R' || id == 'L' || id == 'B') {
          Handle_Stepper_Query(id);
        } else {
          Serial.println("Invalid query ID.");
        }
      } else if (input.startsWith("E ")) {
        char id = input.charAt(2);
        if (id == 'X' || id == 'Y' || id == 'Z') {
          Servo_Enable(id);
        } else if (id == 'R' || id == 'L' || id == 'B') {
          Stepper_Enable(id);
          Serial.print(id);
          Serial.println(" motor enabled.");
        } else {
          Serial.println("Invalid enable ID.");
        }
      } else if (input.startsWith("D ")) {
        char id = input.charAt(2);
        if (id == 'X' || id == 'Y' || id == 'Z') {
          Servo_Disable(id);
        } else if (id == 'R' || id == 'L' || id == 'B') {
          Motor_Disable(id);
          Serial.print(id);
          Serial.println(" motor disabled.");
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
        char id = input.substring(0, first).charAt(0);
        String rest = input.substring(first + 1);
        int second = rest.indexOf(' ');
        float speed = 0.0f;  // 0 means use default
        float degrees;
        if (second == -1) {
          degrees = rest.toFloat();
        } else {
          String deg_str = rest.substring(0, second);
          String speed_str = rest.substring(second + 1);
          degrees = deg_str.toFloat();
          speed = speed_str.toFloat();
        }
        if (id == 'X' || id == 'Y' || id == 'Z') {
          // Servo command
          Servo_Start(id, degrees, speed);
        } else if (id == 'R' || id == 'L' || id == 'B') {
          // Stepper command
          float steps_per_degree = (float)STEPS_PER_REV / 360.0f;
          float max_allowed_rps = STEP_MAX_RPS;
          if (speed > max_allowed_rps) {
            speed = max_allowed_rps;
            Serial.print("RPS clamped to ");
            Serial.println(speed, 3);
          } else if (speed < 0.0f) {
            speed = -speed;
            Serial.println("RPS made positive.");
          }
          long steps = labs(round(degrees * steps_per_degree));
          if (steps <= 0 || speed <= 0.0f) {
            Serial.println("Invalid input.");
            input = "";
            return;
          }
          int new_direction = (degrees >= 0.0f) ? HIGH : LOW;
          float new_target_rps = speed;
          StepperMotor* motor;
          MotorState* state;
          switch (id) {
            case MOTOR_R: motor = &motor_right; state = &state_right; break;
            case MOTOR_L: motor = &motor_left; state = &state_left; break;
            case MOTOR_B: motor = &motor_bottom; state = &state_bottom; break;
            default: input = ""; return;
          }
          if (*state == MOTOR_IDLE) {
            Motor_Start(id, steps, speed, new_direction);
          } else {
            if (new_direction == motor->direction) {
              motor->steps_remaining = steps;
              motor->target_rps = new_target_rps;
              float curr_rps = motor->current_rps;
              if (curr_rps > new_target_rps) {
                motor->ramp_start_rps = curr_rps;
                motor->ramp_start_time_ms = millis();
                *state = MOTOR_DECEL;
              } else if (curr_rps < new_target_rps) {
                motor->ramp_start_rps = curr_rps;
                motor->ramp_start_time_ms = millis();
                *state = MOTOR_RAMP;
              } else {
                *state = MOTOR_RUN;
              }
            } else {
              motor->is_blending = true;
              motor->blend_steps = steps;
              motor->blend_target_rps = new_target_rps;
              motor->blend_direction = new_direction;
              motor->steps_remaining = LARGE_TEMP_COUNT;
              motor->target_rps = 0.0f;
              motor->ramp_start_rps = motor->current_rps;
              motor->ramp_start_time_ms = millis();
              *state = MOTOR_DECEL;
            }
          }
        } else {
          Serial.println("Invalid ID.");
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void Handle_Stepper_Query(char motor_id) {
  StepperMotor* motor;
  switch (motor_id) {
    case MOTOR_R: motor = &motor_right; break;
    case MOTOR_L: motor = &motor_left; break;
    case MOTOR_B: motor = &motor_bottom; break;
    default: Serial.println("Invalid ID."); return;
  }
  Serial.print(motor_id);
  Serial.print(" remaining steps: ");
  Serial.println(motor->steps_remaining);
  Serial.print(motor_id);
  Serial.print(" current RPS: ");
  Serial.println(motor->current_rps, 3);
}

void Handle_Servo_Query(char servo_id) {
  ServoMotor* motor;
  switch (servo_id) {
    case SERVO_X: motor = &motor_x; break;
    case SERVO_Y: motor = &motor_y; break;
    case SERVO_Z: motor = &motor_z; break;
    default: Serial.println("Invalid ID."); return;
  }
  // Note: current_angle respects per-servo max_angle.
  Serial.print(servo_id);
  Serial.print(" current angle: ");
  Serial.println(motor->current_angle, 1);
  Serial.print(servo_id);
  Serial.print(" current speed: ");
  Serial.println(motor->is_moving ? motor->speed : 0.0f, 3);
}