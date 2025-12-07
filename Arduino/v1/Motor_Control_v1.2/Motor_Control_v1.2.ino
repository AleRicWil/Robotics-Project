// MOTOR CONTROL - TRIPLE STEPPERS WITH FSM AND RAMPING
// Controls three stepper motors via serial commands for ID (R, L, B), signed degrees, positive RPS.
// Supports query ('Q ID') for remaining steps and RPS.
// v1.2: Added independent enable/disable commands ('E ID', 'D ID') for holding torque without motion.
// This allows enabling a motor in idle state to prevent drift, following industry practice for multi-axis synchronization.

// Pin Assignments
#define STEPB_STEP_PIN 8   // Bottom STEP
#define STEPB_DIR_PIN 9   // Bottom DIR
#define STEPB_DIS_PIN 10    // Bottom DISABLE

#define STEPR_DIR_PIN 2    // Right DIR
#define STEPR_STEP_PIN 3   // Right STEP
#define STEPR_DIS_PIN 4    // Right DISABLE

#define STEPL_DIR_PIN 6    // Left DIR
#define STEPL_STEP_PIN 7   // Left STEP
#define STEPL_DIS_PIN 5    // Left DISABLE

// Shared Constants
const float MAX_COLD_START_RPS = 0.001f;
const float STEP_RAMP_ACCEL = 1.0f;
const long LARGE_TEMP_COUNT = 2147483647L;

// Stepper Constants
const int STEPS_PER_REV = 200 * 4;
const float STEP_MAX_RPS = 1.56;

// Enumerations
enum MotorState { MOTOR_IDLE, MOTOR_START, MOTOR_RAMP, MOTOR_RUN, MOTOR_DECEL, MOTOR_STOP };
enum MotorID { MOTOR_R = 'R', MOTOR_L = 'L', MOTOR_B = 'B' };

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

// Global Instances
StepperMotor motor_right, motor_left, motor_bottom;
MotorState state_right = MOTOR_IDLE, state_left = MOTOR_IDLE, state_bottom = MOTOR_IDLE;

// Function Declarations
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
void Handle_Query(char motor_id);
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id);
void Handle_Active_Motors();

void setup() {
  Serial.begin(115200);
  Stepper_Setup();
  Motor_Disable(MOTOR_R);
  Motor_Disable(MOTOR_L);
  Motor_Disable(MOTOR_B);
  delay(500);
  Serial.println("Enter 'motor degrees rps' or 'Q motor':");
}

void loop() {
  Motor_Test();
}

// Hardware Initialization
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

// Core Controls
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
  Stepper_Enable(motor_id);

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

void Motor_Test() {
  Handle_Serial_Input();
  Handle_Stepper_FSM(&motor_right, &state_right, MOTOR_R);
  Handle_Stepper_FSM(&motor_left, &state_left, MOTOR_L);
  Handle_Stepper_FSM(&motor_bottom, &state_bottom, MOTOR_B);
  Handle_Active_Motors();
}

void Handle_Serial_Input() {
  static String input = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.startsWith("Q ")) {
        char motor_id = input.charAt(2);
        if (motor_id == 'R' || motor_id == 'L' || motor_id == 'B') {
          Handle_Query(motor_id);
        } else {
          Serial.println("Invalid query ID.");
        }
      } else if (input.startsWith("E ")) {
        char motor_id = input.charAt(2);
        if (motor_id == 'R' || motor_id == 'L' || motor_id == 'B') {
          Stepper_Enable(motor_id);
          Serial.print(motor_id);
          Serial.println(" motor enabled.");
        } else {
          Serial.println("Invalid enable ID.");
        }
      } else if (input.startsWith("D ")) {
        char motor_id = input.charAt(2);
        if (motor_id == 'R' || motor_id == 'L' || motor_id == 'B') {
          Motor_Disable(motor_id);
          Serial.print(motor_id);
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
        char motor_id = input.substring(0, first).charAt(0);
        if (motor_id != 'R' && motor_id != 'L' && motor_id != 'B') {
          Serial.println("Invalid motor ID.");
          input = "";
          return;
        }
        String rest = input.substring(first + 1);
        int second = rest.indexOf(' ');
        if (second == -1) {
          Serial.println("Invalid format.");
          input = "";
          return;
        }
        float steps_per_degree = (float)STEPS_PER_REV / 360.0f;
        float max_allowed_rps = STEP_MAX_RPS;
        float degrees = rest.substring(0, second).toFloat();
        float rps = rest.substring(second + 1).toFloat();
        if (rps > max_allowed_rps) {
          rps = max_allowed_rps;
          Serial.print("RPS clamped to ");
          Serial.println(rps, 3);
        } else if (rps < 0.0f) {
          rps = -rps;
          Serial.println("RPS made positive.");
        }
        long steps = labs(round(degrees * steps_per_degree));
        if (steps <= 0 || rps <= 0.0f) {
          Serial.println("Invalid input.");
          input = "";
          return;
        }
        int new_direction = (degrees >= 0.0f) ? HIGH : LOW;
        float new_target_rps = rps;
        StepperMotor* motor;
        MotorState* state;
        switch (motor_id) {
          case MOTOR_R: motor = &motor_right; state = &state_right; break;
          case MOTOR_L: motor = &motor_left; state = &state_left; break;
          case MOTOR_B: motor = &motor_bottom; state = &state_bottom; break;
          default: input = ""; return;
        }
        if (*state == MOTOR_IDLE) {
          Motor_Start(motor_id, steps, rps, new_direction);
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
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void Handle_Query(char motor_id) {
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
          Stepper_Enable(motor_id);
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

// Helpers
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