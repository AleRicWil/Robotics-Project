// ======================================================
// MOTOR CONTROL - DUAL STEPPERS AND DC WITH ENCODER FSM AND PID
// ======================================================

// ------------------------------------------------------
// Pin Assignments
// ------------------------------------------------------
#define STEPR_DIR_PIN 5   // Right stepper DIR
#define STEPR_STEP_PIN 6  // Right stepper STEP
#define STEPR_DIS_PIN 7   // Right stepper DISABLE (adjusted to avoid interrupt conflict)

#define STEPL_DIR_PIN 9  // Left stepper DIR
#define STEPL_STEP_PIN 10  // Left stepper STEP
#define STEPL_DIS_PIN 8   // Left stepper DISABLE

#define DC_PWM_PIN 3  // DC motor PWM speed control
#define DC_DIR_PIN 4  // DC motor direction
#define DC_FG_PIN 2   // DC motor encoder pulses (interrupt-capable pin)

// ------------------------------------------------------
// Shared Configuration Constants
// ------------------------------------------------------
const float MAX_COLD_START_RPS = 0.00f;  // Instant-start RPS limit for all motors. Reversing directions needs momentary rest at low rps.
const float STEP_RAMP_ACCEL    = 2.0f;     // Acceleration in RPS per second
const float DC_RAMP_ACCEL      = 1.0f;
const long  LARGE_TEMP_COUNT   = 2147483647L;     // Large value to prevent premature stop during blend decel

// ------------------------------------------------------
// Stepper-Specific Constants
// ------------------------------------------------------
const int STEPS_PER_REV = 200 * 8;  // Steps per revolution (e.g., 400 for half-step)
const float STEP_MAX_RPS = 15.0;

// ------------------------------------------------------
// DC-Specific Constants
// ------------------------------------------------------
const int PULSES_PER_REV = 45 * 6 * 2;            // 45:1 gearbox, 6 positions/rev, 2 pulses/position
const float DC_KP = 50.0f;                        // PID proportional gain
const float DC_KI = 15.0f;                       // PID integral gain
const float DC_KD = 0.5f;                         // PID derivative gain
const float DC_INTEGRAL_WINDUP_LIMIT = 30.0f;     // Anti-windup limit for integral term
const unsigned long DC_UPDATE_INTERVAL_MS = 50;  // Speed/PID update frequency (ms)
const int DC_MIN_PWM = 0;                         // PWM for maximum speed
const int DC_MAX_PWM = 247;                       // PWM for minimum speed (cutoff)
const float DC_MAX_RPS = 2.9;
const unsigned long DC_DEBOUNCE_US = 500;         // Debounce time for encoder pulses (us) [instead of wired RC low-pass filter]

// ------------------------------------------------------
// Global Volatiles for DC Encoder (Interrupt-Driven)
// ------------------------------------------------------
volatile unsigned long DC_Pulses = 0;
volatile unsigned long last_pulse_time_us = 0;

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
               MOTOR_D = 'D' };

// ------------------------------------------------------
// Stepper Motor State Structure
// ------------------------------------------------------
struct StepperMotor {
  volatile long   steps_remaining      = 0;
  unsigned long   step_interval_us     = 1000;
  int             direction            = HIGH;
  float           current_rps          = 0.0f;
  float           target_rps           = 0.0f;
  unsigned long   ramp_start_time_ms   = 0;
  unsigned long   last_step_time_us    = 0;
  float           ramp_start_rps       = 0.0f;
  long            blend_steps          = 0;
  float           blend_target_rps     = 0.0f;
  int             blend_direction      = HIGH;
  bool            is_blending          = false;
};

// ------------------------------------------------------
// DC Motor State Structure
// ------------------------------------------------------
struct DCMotor {
  volatile long   target_pulses        = 0;
  int             direction            = HIGH;
  float           target_rps           = 0.0f;
  float           pid_target_rps       = 0.0f;
  float           measured_rps         = 0.0f;
  int             pwm_value            = 255;  // Initial: stopped
  unsigned long   ramp_start_time_ms   = 0;
  unsigned long   last_update_time_ms  = 0;
  unsigned long   last_pulse_count     = 0;
  float           integral             = 0.0f;
  float           prev_error           = 0.0f;
  float           ramp_start_rps       = 0.0f;
  unsigned long   decel_start_pulses   = 0;
  long            blend_steps          = 0;
  float           blend_target_rps     = 0.0f;
  int             blend_direction      = HIGH;
  bool            is_blending          = false;
};

// ------------------------------------------------------
// Global Motor Instances and States
// ------------------------------------------------------
StepperMotor motor_right;
StepperMotor motor_left;
DCMotor motor_dc;

MotorState state_right = MOTOR_IDLE;
MotorState state_left = MOTOR_IDLE;
MotorState state_dc = MOTOR_IDLE;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================
void Stepper_Setup();
void DC_Setup();

void Motor_Start(char motor_id, long steps, float rps);
void Motor_Disable(char motor_id);
void Stepper_Timed_Step(char motor_id);
unsigned long Get_DC_Pulses();
void Motor_Test();

void Stepper_Set_Direction(char motor_id, int direction);
void Stepper_Enable(char motor_id);
void Stepper_Init_Cold_Start(StepperMotor* motor);
void Stepper_Init_Ramp(StepperMotor* motor);
unsigned long Stepper_Calculate_Interval_us(float rps);
long Calculate_Decel_Steps(float rps);

void DC_Reset_Encoder();
void DC_Set_Direction(int direction);
void DC_Set_Initial_PWM();
void DC_Reset_PID(DCMotor* motor);
void DC_Init_Cold_Start(DCMotor* motor);
void DC_Init_Ramp(DCMotor* motor);
bool DC_Should_Update_PID(unsigned long now_ms, DCMotor* motor);
float DC_Calculate_Measured_RPS(unsigned long delta_pulses, float delta_sec);
float DC_Compute_PID_Output(float error, float delta_sec, DCMotor* motor);
void DC_Clamp_PWM(DCMotor* motor);
void DC_Apply_PWM(int pwm_value);
void DC_Update_PID_State(float error, unsigned long curr_pulses, unsigned long now_ms, DCMotor* motor);
void DC_Print_Status(unsigned long curr_pulses);
long Calculate_Decel_Pulses(float rps);

void Handle_Serial_Input();
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id);
void Handle_DC_FSM();
void Handle_Active_Motors();

// ======================================================
// ARDUINO ENTRY POINTS
// ======================================================
void setup() {
  Serial.begin(115200);
  Stepper_Setup();
  DC_Setup();

  Motor_Disable(MOTOR_R);
  Motor_Disable(MOTOR_L);
  Motor_Disable(MOTOR_D);

  state_right = MOTOR_IDLE;
  state_left = MOTOR_IDLE;
  state_dc = MOTOR_IDLE;

  delay(500);
  Serial.println("Enter 'motor degrees rps' (e.g., 'R 720 0.5', 'L 360 -1.0', or 'D 180 2.0'):");
}

void loop() {
  Motor_Test();
}

// ======================================================
// HARDWARE INITIALIZATION
// ======================================================

// ------------------------------------------------------
// Initialize Motors
// ------------------------------------------------------
void Stepper_Setup() {
  // Right stepper
  pinMode(STEPR_STEP_PIN, OUTPUT);
  pinMode(STEPR_DIR_PIN, OUTPUT);
  pinMode(STEPR_DIS_PIN, OUTPUT);
  digitalWrite(STEPR_DIS_PIN, HIGH);  // Disabled initially

  // Left stepper
  pinMode(STEPL_STEP_PIN, OUTPUT);
  pinMode(STEPL_DIR_PIN, OUTPUT);
  pinMode(STEPL_DIS_PIN, OUTPUT);
  digitalWrite(STEPL_DIS_PIN, HIGH);  // Disabled initially
}

void DC_Setup() {
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DC_FG_PIN), countPulse, HIGH);

  // Initialize stopped and forward
  analogWrite(DC_PWM_PIN, 255);
  digitalWrite(DC_DIR_PIN, HIGH);
}

// ======================================================
// CORE CONTROL IMPLEMENTATIONS
// ======================================================

// ------------------------------------------------------
// Start motor movement with ramping and PID (for DC)
// ------------------------------------------------------
void Motor_Start(char motor_id, long steps, float rps) {
  if (steps <= 0 || rps == 0.0f) {
    Serial.println("Invalid input: Steps must be positive, RPS must be non-zero.");
    return;
  }

  if (motor_id == MOTOR_R || motor_id == MOTOR_L) {
    // Stepper handling
    StepperMotor* motor = (motor_id == MOTOR_R) ? &motor_right : &motor_left;
    MotorState* state = (motor_id == MOTOR_R) ? &state_right : &state_left;

    motor->steps_remaining = steps;
    motor->direction = (rps >= 0) ? HIGH : LOW;
    motor->target_rps = abs(rps);

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

  } else if (motor_id == MOTOR_D) {
    // DC handling
    MotorState* state = &state_dc;

    DC_Reset_Encoder();
    motor_dc.target_pulses = labs(steps);
    motor_dc.direction = (rps >= 0) ? HIGH : LOW;
    motor_dc.target_rps = fabs(rps);

    DC_Set_Direction(motor_dc.direction);
    DC_Set_Initial_PWM();
    DC_Reset_PID(&motor_dc);

    if (motor_dc.target_rps <= MAX_COLD_START_RPS) {
      DC_Init_Cold_Start(&motor_dc);
      *state = MOTOR_RUN;
    } else {
      DC_Init_Ramp(&motor_dc);
      *state = MOTOR_RAMP;
    }
  }
}

// ------------------------------------------------------
// Disable motor (steppers: high-impedance; DC: stop PWM)
// ------------------------------------------------------
void Motor_Disable(char motor_id) {
  if (motor_id == MOTOR_R || motor_id == MOTOR_L) {
    const int dis_pin = (motor_id == MOTOR_R) ? STEPR_DIS_PIN : STEPL_DIS_PIN;
    digitalWrite(dis_pin, HIGH);
  } else if (motor_id == MOTOR_D) {
    analogWrite(DC_PWM_PIN, 255);
  }
}

// ------------------------------------------------------
// Generate a single stepper pulse if interval elapsed
// ------------------------------------------------------
void 
Stepper_Timed_Step(char motor_id) {
  StepperMotor* motor = (motor_id == MOTOR_R) ? &motor_right : &motor_left;
  const int step_pin = (motor_id == MOTOR_R) ? STEPR_STEP_PIN : STEPL_STEP_PIN;

  if (micros() - motor->last_step_time_us >= motor->step_interval_us) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(5);  // Safe pulse width (min ~1.2us)
    digitalWrite(step_pin, LOW);
    motor->last_step_time_us = micros();
    motor->steps_remaining--;
  }
}

// ------------------------------------------------------
// Main loop: serial parsing + FSM for all motors
// ------------------------------------------------------
void Motor_Test() {
  Handle_Serial_Input();
  Handle_Stepper_FSM(&motor_right, &state_right, MOTOR_R);
  Handle_Stepper_FSM(&motor_left, &state_left, MOTOR_L);
  Handle_DC_FSM();
  Handle_Active_Motors();
}

// ------------------------------------------------------
// Handle serial input parsing and command initiation
// ------------------------------------------------------
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
    if (motor_id != 'R' && motor_id != 'L' && motor_id != 'D') {
      Serial.println("Invalid motor ID. Use 'R', 'L', or 'D'.");
      return;
    }

    String rest   = input.substring(first + 1);
    int    second = rest.indexOf(' ');
    if (second == -1) {
      Serial.println("Invalid format. Use 'motor_ID degrees rps'.");
      return;
    }

    float steps_per_degree;                     // degrees → steps (or pulses)
    float max_allowed_rps;
    if (motor_id == MOTOR_D) {
        steps_per_degree = (float)PULSES_PER_REV / 360.0f;   // DC encoder pulses
        max_allowed_rps  = DC_MAX_RPS;   // Example limit for DC motor (adjust as needed)
    } else {
        steps_per_degree = (float)STEPS_PER_REV / 360.0f;   // Stepper steps
        max_allowed_rps  = STEP_MAX_RPS;   // Example limit for steppers (adjust as needed)
    }
    
    long   degrees = rest.substring(0, second).toInt();   // user input
    float  rps     = rest.substring(second + 1).toFloat();
    if (fabsf(rps) > max_allowed_rps) {
        rps = (rps >= 0.0f) ? max_allowed_rps : -max_allowed_rps;
        Serial.print("Warning: RPS clamped to ");
        Serial.println(rps, 3);
    }
    long steps = (long)round(degrees * steps_per_degree);   // round to nearest integer
    if (degrees <= 0 || rps == 0.0f) {
      Serial.println("Invalid input: degrees must be >0, RPS must be non-zero.");
      return;
    }

    // ---- Handle start from idle or blend while running ----------------------
    if (motor_id == MOTOR_D) {
      if (state_dc == MOTOR_IDLE) {
        Motor_Start(motor_id, steps, rps);
      } else {
        int new_direction = (rps >= 0) ? HIGH : LOW;
        float new_target_rps = fabs(rps);
        if (new_direction == motor_dc.direction) {
          // Same direction: blend speed, start counting from now
          unsigned long curr_pulses = Get_DC_Pulses();
          motor_dc.target_pulses = curr_pulses + steps;
          motor_dc.target_rps = new_target_rps;
          float curr_pid_target = motor_dc.pid_target_rps;
          if (curr_pid_target > new_target_rps) {
            motor_dc.ramp_start_rps = curr_pid_target;
            motor_dc.ramp_start_time_ms = millis();
            state_dc = MOTOR_DECEL;
          } else if (curr_pid_target < new_target_rps) {
            motor_dc.ramp_start_rps = curr_pid_target;
            motor_dc.ramp_start_time_ms = millis();
            state_dc = MOTOR_RAMP;
          } else {
            state_dc = MOTOR_RUN;
          }
        } else {
          // Opposite direction: decel to zero, then blend to new
          motor_dc.is_blending = true;
          motor_dc.blend_steps = steps;
          motor_dc.blend_target_rps = new_target_rps;
          motor_dc.blend_direction = new_direction;
          motor_dc.decel_start_pulses = Get_DC_Pulses();
          motor_dc.target_pulses = LARGE_TEMP_COUNT;
          motor_dc.target_rps = 0.0f;
          motor_dc.ramp_start_rps = motor_dc.pid_target_rps;
          motor_dc.ramp_start_time_ms = millis();
          state_dc = MOTOR_DECEL;
        }
      }
    } else {
      StepperMotor* motor = (motor_id == MOTOR_R) ? &motor_right : &motor_left;
      MotorState* state = (motor_id == MOTOR_R) ? &state_right : &state_left;
      if (*state == MOTOR_IDLE) {
        Motor_Start(motor_id, steps, rps);
      } else {
        int new_direction = (rps >= 0) ? HIGH : LOW;
        float new_target_rps = fabs(rps);
        if (new_direction == motor->direction) {
          // Same direction: blend speed, start counting from now
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
          // Opposite direction: decel to zero, then blend to new
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
  }
}

// ------------------------------------------------------
// Handle FSM for a stepper motor
// ------------------------------------------------------
void Handle_Stepper_FSM(StepperMotor* motor, MotorState* state, char motor_id) {
  switch (*state) {
    case MOTOR_IDLE:   break;

    case MOTOR_START:  break;  // Handled in Motor_Start()

    case MOTOR_RAMP: {
      float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
      float new_rps     = motor->ramp_start_rps + STEP_RAMP_ACCEL * elapsed_sec;
      if (new_rps >= motor->target_rps) { 
        new_rps = motor->target_rps; 
        *state = MOTOR_RUN; 
      }
      if (new_rps != motor->current_rps) {
        motor->current_rps      = new_rps;
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
      float new_rps     = motor->ramp_start_rps - 2*STEP_RAMP_ACCEL * elapsed_sec;
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
          // Start new motion from stopped state
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
// Handle FSM for DC motor
// ------------------------------------------------------
void Handle_DC_FSM() {
  unsigned long now_ms = millis();

  switch (state_dc) {
    case MOTOR_IDLE:   break;
    
    case MOTOR_START:  break;
    
    case MOTOR_RAMP: {
      float elapsed_sec = (now_ms - motor_dc.ramp_start_time_ms) / 1000.0f;
      float new_pid_target = motor_dc.ramp_start_rps + DC_RAMP_ACCEL * elapsed_sec;
      if (new_pid_target >= motor_dc.target_rps) { 
        new_pid_target = motor_dc.target_rps; 
        state_dc = MOTOR_RUN; 
      }
      motor_dc.pid_target_rps = new_pid_target;

      long remaining = motor_dc.target_pulses - Get_DC_Pulses();
      long decel_pulses = Calculate_Decel_Pulses(motor_dc.pid_target_rps);
      if (remaining <= decel_pulses) {
        motor_dc.target_rps = 0.0f;
        motor_dc.ramp_start_rps = motor_dc.pid_target_rps;
        motor_dc.ramp_start_time_ms = now_ms;
        state_dc = MOTOR_DECEL;
      }
      break;
    }
    
    case MOTOR_RUN: {
      long remaining = motor_dc.target_pulses - Get_DC_Pulses();
      long decel_pulses = Calculate_Decel_Pulses(motor_dc.pid_target_rps);
      if (remaining <= decel_pulses) {
        motor_dc.target_rps = 0.0f;
        motor_dc.ramp_start_rps = motor_dc.pid_target_rps;
        motor_dc.ramp_start_time_ms = now_ms;
        state_dc = MOTOR_DECEL;
      }
      break;
    }

    case MOTOR_DECEL: {
      float elapsed_sec = (now_ms - motor_dc.ramp_start_time_ms) / 1000.0f;
      float new_pid_target = motor_dc.ramp_start_rps - 2*DC_RAMP_ACCEL * elapsed_sec;
      if (new_pid_target < 0.0f) new_pid_target = 0.0f;
      if (new_pid_target <= motor_dc.target_rps) {
        new_pid_target = motor_dc.target_rps;
        if (motor_dc.is_blending) {
          unsigned long decel_count = Get_DC_Pulses() - motor_dc.decel_start_pulses;
          DC_Set_Direction(motor_dc.blend_direction);
          DC_Reset_Encoder();
          motor_dc.target_pulses = motor_dc.blend_steps + decel_count;
          motor_dc.target_rps = motor_dc.blend_target_rps;
          motor_dc.direction = motor_dc.blend_direction;
          motor_dc.is_blending = false;
          DC_Set_Initial_PWM();
          DC_Reset_PID(&motor_dc);
          if (motor_dc.target_rps <= MAX_COLD_START_RPS) {
            motor_dc.pid_target_rps = motor_dc.target_rps;
            state_dc = MOTOR_RUN;
          } else {
            motor_dc.pid_target_rps = MAX_COLD_START_RPS;
            motor_dc.ramp_start_rps = MAX_COLD_START_RPS;
            motor_dc.ramp_start_time_ms = now_ms;
            state_dc = MOTOR_RAMP;
          }
        } else if (motor_dc.target_rps == 0.0f) {
          state_dc = MOTOR_STOP;
        } else {
          state_dc = MOTOR_RUN;
        }
      }
      motor_dc.pid_target_rps = new_pid_target;
      break;
    }
    
    case MOTOR_STOP:
      Motor_Disable(MOTOR_D);
      Serial.println("DC motor stopped.");
      state_dc = MOTOR_IDLE;
      break;
  }

  if (state_dc == MOTOR_RAMP || state_dc == MOTOR_DECEL || state_dc == MOTOR_RUN) {
    if (DC_Should_Update_PID(now_ms, &motor_dc)) {
      unsigned long curr_pulses = Get_DC_Pulses();
      unsigned long delta_pulses = curr_pulses - motor_dc.last_pulse_count;
      float delta_sec = (now_ms - motor_dc.last_update_time_ms) / 1000.0f;
      float measured_rps = DC_Calculate_Measured_RPS(delta_pulses, delta_sec);
      motor_dc.measured_rps = measured_rps;

      float error = motor_dc.pid_target_rps - measured_rps;
      float output = DC_Compute_PID_Output(error, delta_sec, &motor_dc);
      motor_dc.pwm_value -= (int)output;
      DC_Clamp_PWM(&motor_dc);
      DC_Apply_PWM(motor_dc.pwm_value);

      DC_Update_PID_State(error, curr_pulses, now_ms, &motor_dc);
    }
  }
}

// ------------------------------------------------------
// Handle actions for active motors
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

  if (state_dc == MOTOR_RAMP || state_dc == MOTOR_RUN || state_dc == MOTOR_DECEL) {
    unsigned long curr_pulses = Get_DC_Pulses();
    static unsigned long message_time = millis();
    if (millis() - message_time >= DC_UPDATE_INTERVAL_MS) {
      message_time = millis();
      DC_Print_Status(curr_pulses);
    }
    if (curr_pulses >= motor_dc.target_pulses) state_dc = MOTOR_STOP;
  }
}

// ======================================================
// STEPPER HELPER FUNCTIONS
// ======================================================

// ------------------------------------------------------
// Set stepper direction pin
// ------------------------------------------------------
void Stepper_Set_Direction(char motor_id, int direction) {
  const int dir_pin = (motor_id == MOTOR_R) ? STEPR_DIR_PIN : STEPL_DIR_PIN;
  digitalWrite(dir_pin, direction);
}

// ------------------------------------------------------
// Enable stepper (low on disable pin)
// ------------------------------------------------------
void Stepper_Enable(char motor_id) {
  const int dis_pin = (motor_id == MOTOR_R) ? STEPR_DIS_PIN : STEPL_DIS_PIN;
  digitalWrite(dis_pin, LOW);
}

// ------------------------------------------------------
// Initialize stepper for cold start (no ramp)
// ------------------------------------------------------
void Stepper_Init_Cold_Start(StepperMotor* motor) {
  motor->current_rps = motor->target_rps;
  motor->step_interval_us = Stepper_Calculate_Interval_us(motor->target_rps);
}

// ------------------------------------------------------
// Initialize stepper for ramping
// ------------------------------------------------------
void Stepper_Init_Ramp(StepperMotor* motor) {
  motor->current_rps = MAX_COLD_START_RPS;
  motor->step_interval_us = Stepper_Calculate_Interval_us(MAX_COLD_START_RPS);
  motor->ramp_start_time_ms = millis();
  motor->ramp_start_rps = MAX_COLD_START_RPS;
}

// ------------------------------------------------------
// Calculate step interval in microseconds
// ------------------------------------------------------
unsigned long Stepper_Calculate_Interval_us(float rps) {
  if (rps <= 0.0f) {
    return 0xFFFFFFFFUL;  // Maximum value to prevent stepping
  }
  return 1000000UL / (rps * STEPS_PER_REV);
}

// ------------------------------------------------------
// Update stepper ramp and check if complete
// ------------------------------------------------------
bool Stepper_Update_Ramp(StepperMotor* motor, MotorState* state) {
  float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
  float new_rps = MAX_COLD_START_RPS + STEP_RAMP_ACCEL * elapsed_sec;
  if (new_rps >= motor->target_rps) {
    new_rps = motor->target_rps;
    *state = MOTOR_RUN;
    return true;  // Ramp complete
  }
  if (new_rps != motor->current_rps) {
    motor->current_rps = new_rps;
    motor->step_interval_us = Stepper_Calculate_Interval_us(new_rps);
  }
  return false;  // Ramp ongoing
}

// ------------------------------------------------------
// Calculate steps needed to decelerate to zero
// ------------------------------------------------------
long Calculate_Decel_Steps(float rps) {
  if (rps <= 0.0f) return 0;
  float revs_decel = (rps * rps) / (2.0f * 2*STEP_RAMP_ACCEL);
  return (long)ceil(revs_decel * STEPS_PER_REV);
}

// ======================================================
// DC HELPER FUNCTIONS
// ======================================================

// ------------------------------------------------------
// Reset DC encoder pulses and timestamp
// ------------------------------------------------------
void DC_Reset_Encoder() {
  noInterrupts();
  DC_Pulses = 0;
  last_pulse_time_us = micros();
  interrupts();
}

// ------------------------------------------------------
// Set DC direction pin
// ------------------------------------------------------
void DC_Set_Direction(int direction) {
  digitalWrite(DC_DIR_PIN, direction);
}

// ------------------------------------------------------
// Set initial moderate PWM for DC start
// ------------------------------------------------------
void DC_Set_Initial_PWM() {
  motor_dc.pwm_value = 240;
  DC_Apply_PWM(motor_dc.pwm_value);
}

// ------------------------------------------------------
// Reset DC PID variables
// ------------------------------------------------------
void DC_Reset_PID(DCMotor* motor) {
  motor->integral = 0.0f;
  motor->prev_error = 0.0f;
  motor->measured_rps = 0.0f;
  motor->last_pulse_count = 0;
  motor->last_update_time_ms = millis();
}

// ------------------------------------------------------
// Initialize DC for cold start (no ramp)
// ------------------------------------------------------
void DC_Init_Cold_Start(DCMotor* motor) {
  motor->pid_target_rps = motor->target_rps;
}

// ------------------------------------------------------
// Initialize DC for ramping
// ------------------------------------------------------
void DC_Init_Ramp(DCMotor* motor) {
  motor->pid_target_rps = MAX_COLD_START_RPS;
  motor->ramp_start_time_ms = millis();
  motor->ramp_start_rps = MAX_COLD_START_RPS;
}

// ------------------------------------------------------
// Update DC ramp and check if complete
// ------------------------------------------------------
bool DC_Update_Ramp(DCMotor* motor, MotorState* state) {
  float elapsed_sec = (millis() - motor->ramp_start_time_ms) / 1000.0f;
  float new_pid_target = MAX_COLD_START_RPS + DC_RAMP_ACCEL * elapsed_sec;
  if (new_pid_target >= motor->target_rps) {
    new_pid_target = motor->target_rps;
    *state = MOTOR_RUN;
    return true;  // Ramp complete
  }
  motor->pid_target_rps = new_pid_target;
  return false;  // Ramp ongoing
}

// ------------------------------------------------------
// Check if time for DC PID update
// ------------------------------------------------------
bool DC_Should_Update_PID(unsigned long now_ms, DCMotor* motor) {
  return (now_ms - motor->last_update_time_ms >= DC_UPDATE_INTERVAL_MS);
}

// ------------------------------------------------------
// Calculate measured RPS for DC
// ------------------------------------------------------
float DC_Calculate_Measured_RPS(unsigned long delta_pulses, float delta_sec) {
  return (float)delta_pulses / (PULSES_PER_REV * delta_sec);
}

// ------------------------------------------------------
// Compute PID output for DC
// ------------------------------------------------------
float DC_Compute_PID_Output(float error, float delta_sec, DCMotor* motor) {
  motor->integral += error * delta_sec;
  if (motor->integral > DC_INTEGRAL_WINDUP_LIMIT) motor->integral = DC_INTEGRAL_WINDUP_LIMIT;
  if (motor->integral < -DC_INTEGRAL_WINDUP_LIMIT) motor->integral = -DC_INTEGRAL_WINDUP_LIMIT;
  float derivative = (error - motor->prev_error) / delta_sec;
  return DC_KP * error + DC_KI * motor->integral + DC_KD * derivative;
}

// ------------------------------------------------------
// Clamp PWM value within limits
// ------------------------------------------------------
void DC_Clamp_PWM(DCMotor* motor) {
  if (motor->pwm_value < DC_MIN_PWM) motor->pwm_value = DC_MIN_PWM;
  if (motor->pwm_value > DC_MAX_PWM) motor->pwm_value = DC_MAX_PWM;
}

// ------------------------------------------------------
// Apply PWM to DC motor
// ------------------------------------------------------
void DC_Apply_PWM(int pwm_value) {
  analogWrite(DC_PWM_PIN, pwm_value);
}

// ------------------------------------------------------
// Update DC PID state after computation
// ------------------------------------------------------
void DC_Update_PID_State(float error, unsigned long curr_pulses, unsigned long now_ms, DCMotor* motor) {
  motor->prev_error = error;
  motor->last_pulse_count = curr_pulses;
  motor->last_update_time_ms = now_ms;
}

// ------------------------------------------------------
// Print DC status for debugging
// ------------------------------------------------------
void DC_Print_Status(unsigned long curr_pulses) {
  Serial.print("DC Steps: ");
  Serial.print(curr_pulses);
  Serial.print("\tDC rps: ");
  Serial.print(motor_dc.measured_rps);
  Serial.print("DC PWM: ");
  Serial.println(motor_dc.pwm_value);
}

// ------------------------------------------------------
// Calculate pulses needed to decelerate to zero
// ------------------------------------------------------
long Calculate_Decel_Pulses(float rps) {
  if (rps <= 0.0f) return 0;
  float revs_decel = (rps * rps) / (2.0f * 2*DC_RAMP_ACCEL);
  return (long)ceil(revs_decel * PULSES_PER_REV);
}

// ------------------------------------------------------
// Safely retrieve current DC pulse count
// ------------------------------------------------------
unsigned long Get_DC_Pulses() {
  noInterrupts();
  unsigned long temp = DC_Pulses;
  interrupts();
  return temp;
}

// ======================================================
// INTERRUPT SERVICE ROUTINES
// ======================================================
void countPulse() {
  unsigned long now_us = micros();
  if (now_us - last_pulse_time_us > DC_DEBOUNCE_US) {
    DC_Pulses++;
    last_pulse_time_us = now_us;
  }
}

// ======================================================