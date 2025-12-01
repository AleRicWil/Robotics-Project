// ======================================================
// MOTOR CONTROL - TRIPLE STEPPERS WITH FSM AND RAMPING
// ======================================================

// This sketch controls three stepper motors using a finite state machine (FSM) for smooth ramping acceleration and deceleration.
// Commands are received via serial from a Python script, specifying motor ID (R, L, B), angular displacement in degrees (signed for direction), and angular speed in RPS (positive).
// The direction is determined by the sign of the displacement, allowing for positive or negative rotation.
// All motors have identical control logic, with individual pin assignments for flexibility.
// Code structure follows industry conventions: clear pin assignments, constants, structs for state, and modular functions for readability and serviceability.

// ------------------------------------------------------
// Pin Assignments
// ------------------------------------------------------
#define STEPB_DIR_PIN 10   // Bottom stepper DIR
#define STEPB_STEP_PIN 9  // Bottom stepper STEP
#define STEPB_DIS_PIN 8   // Bottom stepper DISABLE

#define STEPR_DIR_PIN 2   // Right stepper DIR
#define STEPR_STEP_PIN 3  // Right stepper STEP
#define STEPR_DIS_PIN 4   // Right stepper DISABLE

#define STEPL_DIR_PIN 6    // Left stepper DIR
#define STEPL_STEP_PIN 7  // Left stepper STEP
#define STEPL_DIS_PIN 5    // Left stepper DISABLE

// ------------------------------------------------------
// Shared Configuration Constants
// ------------------------------------------------------
const float MAX_COLD_START_RPS = 0.00f;     // Instant-start RPS limit for motors. Reversing directions needs momentary rest at low RPS.
const float STEP_RAMP_ACCEL = 2.0f;         // Acceleration in RPS per second for steppers
const long LARGE_TEMP_COUNT = 2147483647L;  // Large value to prevent premature stop during blend decel

// ------------------------------------------------------
// Stepper-Specific Constants
// ------------------------------------------------------
const int STEPS_PER_REV = 200 * 4;  // Steps per revolution (e.g., 200 full steps * 8 for 1/8 microstepping)
const float STEP_MAX_RPS = 1.2;     // Maximum allowed RPS for steppers

// ------------------------------------------------------
// Enumerations
// ------------------------------------------------------
enum MotorState { MOTOR_IDLE,
                  MOTOR_START,
                  MOTOR_RAMP,
                  MOTOR_RUN,
                  MOTOR_DECEL,
                  MOTOR_STOP };
enum MotorID { MOTOR_R = 'R',
               MOTOR_L = 'L',
               MOTOR_B = 'B' };

// ------------------------------------------------------
// Stepper Motor State Structure
// ------------------------------------------------------
struct StepperMotor {
  volatile long steps_remaining = 0;      // Remaining steps to target
  unsigned long step_interval_us = 1000;  // Interval between steps in microseconds
  int direction = HIGH;                   // Direction pin state (HIGH or LOW)
  float current_rps = 0.0f;               // Current speed in revolutions per second
  float target_rps = 0.0f;                // Target speed in RPS
  unsigned long ramp_start_time_ms = 0;   // Timestamp when ramp started
  unsigned long last_step_time_us = 0;    // Timestamp of last step pulse
  float ramp_start_rps = 0.0f;            // RPS at start of ramp
  long blend_steps = 0;                   // Steps for blending to new command
  float blend_target_rps = 0.0f;          // Target RPS for blend
  int blend_direction = HIGH;             // Direction for blend
  bool is_blending = false;               // Flag for active blending
};

// ------------------------------------------------------
// Global Motor Instances and States
// ------------------------------------------------------
StepperMotor motor_right;
StepperMotor motor_left;
StepperMotor motor_bottom;

MotorState state_right = MOTOR_IDLE;
MotorState state_left = MOTOR_IDLE;
MotorState state_bottom = MOTOR_IDLE;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================
void Stepper_Setup();

void Motor_Start(char motor_id, long steps, float rps, int direction);
void Motor_Disable(char motor_id);
void Stepper_Timed_Step(char motor_id);
void Motor_Test();

void Stepper_Set_Direction(char motor_id, int direction);
void Stepper_Enable(char motor_id);
void Stepper_Init_Cold_Start(StepperMotor* motor);
void Stepper_Init_Ramp(StepperMotor* motor);
unsigned long Stepper_Calculate_Interval_us(float rps);
long Calculate_Decel_Steps(float rps);

void Handle_Serial_Input();
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id);
void Handle_Active_Motors();

// ======================================================
// ARDUINO ENTRY POINTS
// ======================================================
void setup() {
  Serial.begin(115200);  // High baud rate for fast communication with Python script
  Stepper_Setup();

  Motor_Disable(MOTOR_R);
  Motor_Disable(MOTOR_L);
  Motor_Disable(MOTOR_B);

  state_right = MOTOR_IDLE;
  state_left = MOTOR_IDLE;
  state_bottom = MOTOR_IDLE;

  delay(500);  // Brief delay for serial stabilization
  Serial.println("Enter 'motor degrees rps' (e.g., 'R 720 0.5', 'L -360 1.0', 'B 180 0.75'):");
}

void loop() {
  Motor_Test();  // Main loop handler for input and FSM updates
}

// ======================================================
// HARDWARE INITIALIZATION
// ======================================================

// ------------------------------------------------------
// Initialize Stepper Pins
// ------------------------------------------------------
void Stepper_Setup() {
  // Right stepper setup
  pinMode(STEPR_STEP_PIN, OUTPUT);
  pinMode(STEPR_DIR_PIN, OUTPUT);
  pinMode(STEPR_DIS_PIN, OUTPUT);
  digitalWrite(STEPR_DIS_PIN, HIGH);  // Disabled initially to save power and prevent heating

  // Left stepper setup
  pinMode(STEPL_STEP_PIN, OUTPUT);
  pinMode(STEPL_DIR_PIN, OUTPUT);
  pinMode(STEPL_DIS_PIN, OUTPUT);
  digitalWrite(STEPL_DIS_PIN, HIGH);  // Disabled initially

  // Bottom stepper setup
  pinMode(STEPB_STEP_PIN, OUTPUT);
  pinMode(STEPB_DIR_PIN, OUTPUT);
  pinMode(STEPB_DIS_PIN, OUTPUT);
  digitalWrite(STEPB_DIS_PIN, HIGH);  // Disabled initially
}

// ======================================================
// CORE CONTROL IMPLEMENTATIONS
// ======================================================

// ------------------------------------------------------
// Start stepper movement with ramping
// ------------------------------------------------------
// This function initializes the motor for a new command, handling cold start or ramp based on target RPS.
// Direction is passed separately, as it's now determined by the sign of degrees.
void Motor_Start(char motor_id, long steps, float rps, int direction) {
  if (steps <= 0 || rps <= 0.0f) {
    Serial.println("Invalid input: Steps must be positive, RPS must be positive.");
    return;
  }

  StepperMotor* motor;
  MotorState* state;
  switch (motor_id) {
    case MOTOR_R:
      motor = &motor_right;
      state = &state_right;
      break;
    case MOTOR_L:
      motor = &motor_left;
      state = &state_left;
      break;
    case MOTOR_B:
      motor = &motor_bottom;
      state = &state_bottom;
      break;
    default:
      return;
  }

  motor->steps_remaining = steps;
  motor->direction = direction;
  motor->target_rps = rps;

  Stepper_Set_Direction(motor_id, motor->direction);
  Stepper_Enable(motor_id);

  if (motor->target_rps <= MAX_COLD_START_RPS) {
    Stepper_Init_Cold_Start(motor);
    *state = MOTOR_RUN;
  } else {
    Stepper_Init_Ramp(motor);
    *state = MOTOR_RAMP;
  }

  motor->last_step_time_us = micros();  // Reset step timing
}

// ------------------------------------------------------
// Disable stepper (high-impedance coils)
// ------------------------------------------------------
void Motor_Disable(char motor_id) {
  int dis_pin;
  switch (motor_id) {
    case MOTOR_R:
      dis_pin = STEPR_DIS_PIN;
      break;
    case MOTOR_L:
      dis_pin = STEPL_DIS_PIN;
      break;
    case MOTOR_B:
      dis_pin = STEPB_DIS_PIN;
      break;
    default:
      return;
  }
  digitalWrite(dis_pin, HIGH);  // Disable to prevent coil current and heating
}

// ------------------------------------------------------
// Generate a single stepper pulse if interval elapsed
// ------------------------------------------------------
// This implements the timed stepping, ensuring precise pulse timing without blocking.
void Stepper_Timed_Step(char motor_id) {
  StepperMotor* motor;
  int step_pin;
  switch (motor_id) {
    case MOTOR_R:
      motor = &motor_right;
      step_pin = STEPR_STEP_PIN;
      break;
    case MOTOR_L:
      motor = &motor_left;
      step_pin = STEPL_STEP_PIN;
      break;
    case MOTOR_B:
      motor = &motor_bottom;
      step_pin = STEPB_STEP_PIN;
      break;
    default:
      return;
  }

  if (micros() - motor->last_step_time_us >= motor->step_interval_us) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(5);  // Pulse width: minimum required by driver (~1-2us, 5us for safety)
    digitalWrite(step_pin, LOW);
    motor->last_step_time_us = micros();
    motor->steps_remaining--;
  }
}

// ------------------------------------------------------
// Main loop: serial parsing + FSM for steppers
// ------------------------------------------------------
void Motor_Test() {
  Handle_Serial_Input();
  Handle_Stepper_FSM(&motor_right, &state_right, MOTOR_R);
  Handle_Stepper_FSM(&motor_left, &state_left, MOTOR_L);
  Handle_Stepper_FSM(&motor_bottom, &state_bottom, MOTOR_B);
  Handle_Active_Motors();
}

// ------------------------------------------------------
// Handle serial input parsing and command initiation
// ------------------------------------------------------
// Parses commands like "R -720 0.5" where negative degrees indicate reverse direction.
// Calculates steps from degrees, clamps RPS if needed, and handles blending if motor is active.
void Handle_Serial_Input() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int first = input.indexOf(' ');
    if (first == -1) {
      Serial.println("Invalid format. Use 'motor_ID degrees rps'.");
      return;
    }

    char motor_id = input.substring(0, first).charAt(0);
    if (motor_id != 'R' && motor_id != 'L' && motor_id != 'B') {
      Serial.println("Invalid motor ID. Use 'R', 'L', or 'B'.");
      return;
    }

    String rest = input.substring(first + 1);
    int second = rest.indexOf(' ');
    if (second == -1) {
      Serial.println("Invalid format. Use 'motor_ID degrees rps'.");
      return;
    }

    float steps_per_degree = (float)STEPS_PER_REV / 360.0f;  // Conversion factor for degrees to steps
    float max_allowed_rps = STEP_MAX_RPS;

    float degrees = rest.substring(0, second).toFloat();  // Degrees can be negative for direction
    float rps = rest.substring(second + 1).toFloat();
    if (rps > max_allowed_rps) {
      rps = max_allowed_rps;
      Serial.print("Warning: RPS clamped to ");
      Serial.println(rps, 3);
    } else if (rps < 0.0f) {
      rps = -rps;  // Ensure positive RPS, as direction is from degrees
      Serial.println("Warning: RPS made positive; direction from degrees.");
    }
    long steps = labs(round(degrees * steps_per_degree));  // Absolute steps, round to nearest integer
    if (steps <= 0 || rps <= 0.0f) {
      Serial.println("Invalid input: |degrees| must be >0, RPS must be >0.");
      return;
    }

    // Determine direction from sign of degrees
    int new_direction = (degrees >= 0.0f) ? HIGH : LOW;
    float new_target_rps = rps;

    // ---- Handle start from idle or blend while running ----------------------
    StepperMotor* motor;
    MotorState* state;
    switch (motor_id) {
      case MOTOR_R:
        motor = &motor_right;
        state = &state_right;
        break;
      case MOTOR_L:
        motor = &motor_left;
        state = &state_left;
        break;
      case MOTOR_B:
        motor = &motor_bottom;
        state = &state_bottom;
        break;
      default:
        return;
    }
    if (*state == MOTOR_IDLE) {
      Motor_Start(motor_id, steps, rps, new_direction);
    } else {
      if (new_direction == motor->direction) {
        // Same direction: blend speed, adjust remaining steps from now
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
        // Opposite direction: decel to zero, then blend to new command
        motor->is_blending = true;
        motor->blend_steps = steps;
        motor->blend_target_rps = new_target_rps;
        motor->blend_direction = new_direction;
        motor->steps_remaining = LARGE_TEMP_COUNT;  // Temp large count for decel
        motor->target_rps = 0.0f;
        motor->ramp_start_rps = motor->current_rps;
        motor->ramp_start_time_ms = millis();
        *state = MOTOR_DECEL;
      }
    }
  }
}

// ------------------------------------------------------
// Handle FSM for a stepper motor
// ------------------------------------------------------
// Finite State Machine to manage motor states: ramp up, run at constant speed, decel, stop.
// Handles blending for new commands during motion.
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id) {
  switch (*state) {
    case MOTOR_IDLE: break;

    case MOTOR_START: break;  // Handled in Motor_Start()

    case MOTOR_RAMP:
      {
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

    case MOTOR_RUN:
      {
        long decel_steps = Calculate_Decel_Steps(motor->current_rps);
        if (motor->steps_remaining <= decel_steps) {
          motor->target_rps = 0.0f;
          motor->ramp_start_rps = motor->current_rps;
          motor->ramp_start_time_ms = millis();
          *state = MOTOR_DECEL;
        }
        break;
      }

    case MOTOR_DECEL:
      {
        float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
        float new_rps = motor->ramp_start_rps - 2 * STEP_RAMP_ACCEL * elapsed_sec;  // Decel at twice accel rate for quicker stop
        if (new_rps < 0.0f) new_rps = 0.0f;
        if (new_rps <= motor->target_rps) {
          new_rps = motor->target_rps;
          if (motor->is_blending) {
            long decel_count = LARGE_TEMP_COUNT - motor->steps_remaining;  // Steps used in decel
            Stepper_Set_Direction(motor_id, motor->blend_direction);
            motor->steps_remaining = motor->blend_steps + decel_count;  // Adjust for overshoot
            motor->target_rps = motor->blend_target_rps;
            motor->direction = motor->blend_direction;
            motor->is_blending = false;
            // Start new motion from near-stopped state
            Stepper_Enable(motor_id);  // Ensure enabled
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
      Motor_Disable(motor_id);
      Serial.print(motor_id);
      Serial.println(" motor stopped.");
      *state = MOTOR_IDLE;
      break;
  }
}

// ------------------------------------------------------
// Handle actions for active motors (timed stepping)
// ------------------------------------------------------
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

// ======================================================
// STEPPER HELPER FUNCTIONS
// ======================================================

// ------------------------------------------------------
// Set stepper direction pin
// ------------------------------------------------------
void Stepper_Set_Direction(char motor_id, int direction) {
  int dir_pin;
  switch (motor_id) {
    case MOTOR_R:
      dir_pin = STEPR_DIR_PIN;
      break;
    case MOTOR_L:
      dir_pin = STEPL_DIR_PIN;
      break;
    case MOTOR_B:
      dir_pin = STEPB_DIR_PIN;
      break;
    default:
      return;
  }
  digitalWrite(dir_pin, direction);  // HIGH or LOW depending on wiring and desired rotation
}

// ------------------------------------------------------
// Enable stepper (low on disable pin)
// ------------------------------------------------------
void Stepper_Enable(char motor_id) {
  int dis_pin;
  switch (motor_id) {
    case MOTOR_R:
      dis_pin = STEPR_DIS_PIN;
      break;
    case MOTOR_L:
      dis_pin = STEPL_DIS_PIN;
      break;
    case MOTOR_B:
      dis_pin = STEPB_DIS_PIN;
      break;
    default:
      return;
  }
  digitalWrite(dis_pin, LOW);  // Enable coils for holding torque and stepping
}

// ------------------------------------------------------
// Initialize stepper for cold start (no ramp)
// ------------------------------------------------------
// For low speeds, start instantly at target RPS without acceleration ramp.
void Stepper_Init_Cold_Start(StepperMotor* motor) {
  motor->current_rps = motor->target_rps;
  motor->step_interval_us = Stepper_Calculate_Interval_us(motor->target_rps);
}

// ------------------------------------------------------
// Initialize stepper for ramping
// ------------------------------------------------------
// For higher speeds, start at max cold start RPS and ramp up to target.
void Stepper_Init_Ramp(StepperMotor* motor) {
  motor->current_rps = MAX_COLD_START_RPS;
  motor->step_interval_us = Stepper_Calculate_Interval_us(MAX_COLD_START_RPS);
  motor->ramp_start_time_ms = millis();
  motor->ramp_start_rps = MAX_COLD_START_RPS;
}

// ------------------------------------------------------
// Calculate step interval in microseconds
// ------------------------------------------------------
// Interval = 1e6 / (RPS * steps per rev), for precise timing between pulses.
unsigned long Stepper_Calculate_Interval_us(float rps) {
  if (rps <= 0.0f) {
    return 0xFFFFFFFFUL;  // Max value to effectively stop stepping
  }
  return 1000000UL / (rps * STEPS_PER_REV);
}

// ------------------------------------------------------
// Calculate steps needed to decelerate to zero
// ------------------------------------------------------
// Using kinematics: distance = v^2 / (2 * a), but here a_decel = 2 * accel for conservative stopping.
// Adjust if needed for your hardware's inertia and torque.
long Calculate_Decel_Steps(float rps) {
  if (rps <= 0.0f) return 0;
  float revs_decel = (rps * rps) / (2.0f * 2 * STEP_RAMP_ACCEL);
  return (long)ceil(revs_decel * STEPS_PER_REV);
}

// ======================================================