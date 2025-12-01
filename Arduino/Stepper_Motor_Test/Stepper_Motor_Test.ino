// ======================================================
// STEPPER MOTOR CONTROL – DUAL MOTOR FSM WITH RAMPING
// ======================================================

// ------------------------------------------------------
// Pin Assignments
// ------------------------------------------------------
#define STEPR_STEP_PIN  3   // Right motor STEP
#define STEPR_DIR_PIN   4   // Right motor DIR
#define STEPR_DIS_PIN   2   // Right motor DISABLE

#define STEPL_STEP_PIN  5   // Left motor STEP
#define STEPL_DIR_PIN   6   // Left motor DIR
#define STEPL_DIS_PIN   7   // Left motor DISABLE

// ------------------------------------------------------
// Motor Configuration Constants
// ------------------------------------------------------
const int   STEPS_PER_REV = 200 / 0.5;        // Steps per rev (full-step)
const float MAX_COLD_START_RPS   = 1.0f;            // Instant-start limit
const float RAMP_ACCEL    = 5.0f;            // rps²

// ------------------------------------------------------
// Enumerations
// ------------------------------------------------------
enum MotorState { MOTOR_IDLE, MOTOR_START, MOTOR_RAMP, MOTOR_RUN, MOTOR_STOP };
enum MotorID    { MOTOR_R = 'R', MOTOR_L = 'L' };

// ------------------------------------------------------
// Per-Motor State Structure
// ------------------------------------------------------
struct StepperMotor {
  volatile long   steps_remaining      = 0;
  unsigned long   step_interval_us     = 1000;
  int             direction            = HIGH;
  float           current_rps          = 0.0f;
  float           target_rps           = 0.0f;
  unsigned long   ramp_start_time_ms   = 0;
  unsigned long   last_step_time_us    = 0;
};

// ------------------------------------------------------
// Global Instances
// ------------------------------------------------------
StepperMotor motor_right;
StepperMotor motor_left;

MotorState state_right = MOTOR_IDLE;
MotorState state_left  = MOTOR_IDLE;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================
void PololuMP6500_NEMA17_Setup();

void STEP_Motor_Start(char motor_id, long steps, float rps);
void STEP_Motor_Disable(char motor_id);
void STEP_Motor_Timed_Step(char motor_id);
void STEP_Motor_Test();

// ======================================================
// ARDUINO ENTRY POINTS
// ======================================================
void setup() {
  Serial.begin(115200);
  PololuMP6500_NEMA17_Setup();

  STEP_Motor_Disable(MOTOR_R);
  STEP_Motor_Disable(MOTOR_L);

  state_right = MOTOR_IDLE;
  state_left  = MOTOR_IDLE;

  Serial.println("Enter 'motor steps rps' (e.g., 'R 1000 0.5' or 'L 500 -1.0'):");
}

void loop() {
  STEP_Motor_Test();
}

// ======================================================
// HARDWARE INITIALISATION
// ======================================================
void PololuMP6500_NEMA17_Setup() {
  // Right motor
  pinMode(STEPR_STEP_PIN, OUTPUT);
  pinMode(STEPR_DIR_PIN,  OUTPUT);
  pinMode(STEPR_DIS_PIN,  OUTPUT);
  digitalWrite(STEPR_DIS_PIN, HIGH);   // disabled

  // Left motor
  pinMode(STEPL_STEP_PIN, OUTPUT);
  pinMode(STEPL_DIR_PIN,  OUTPUT);
  pinMode(STEPL_DIS_PIN,  OUTPUT);
  digitalWrite(STEPL_DIS_PIN, HIGH);   // disabled
}

// ======================================================
// CORE CONTROL IMPLEMENTATIONS
// ======================================================

// ------------------------------------------------------
// Start motor with optional ramp
// ------------------------------------------------------
void STEP_Motor_Start(char motor_id, long steps, float rps) {
  if (steps <= 0 || rps == 0.0f) {
    Serial.println("Invalid input: Steps must be positive, RPS must be non-zero.");
    return;
  }

  StepperMotor* motor = (motor_id == MOTOR_R) ? &motor_right : &motor_left;
  MotorState*   state = (motor_id == MOTOR_R) ? &state_right  : &state_left;

  const int step_pin = (motor_id == MOTOR_R) ? STEPR_STEP_PIN : STEPL_STEP_PIN;
  const int dir_pin  = (motor_id == MOTOR_R) ? STEPR_DIR_PIN  : STEPL_DIR_PIN;
  const int dis_pin  = (motor_id == MOTOR_R) ? STEPR_DIS_PIN  : STEPL_DIS_PIN;

  motor->steps_remaining = steps;
  motor->direction       = (rps >= 0) ? HIGH : LOW;
  motor->target_rps      = abs(rps);

  digitalWrite(dir_pin, motor->direction);
  digitalWrite(dis_pin, LOW);

  if (motor->target_rps <= MAX_COLD_START_RPS) {
    motor->current_rps      = motor->target_rps;
    motor->step_interval_us = 1000000UL / (motor->target_rps * STEPS_PER_REV);
    *state = MOTOR_RUN;
  } else {
    motor->current_rps      = MAX_COLD_START_RPS;
    motor->step_interval_us = 1000000UL / (MAX_COLD_START_RPS * STEPS_PER_REV);
    motor->ramp_start_time_ms = millis();
    *state = MOTOR_RAMP;
  }

  motor->last_step_time_us = micros();
}

// ------------------------------------------------------
// Disable driver
// ------------------------------------------------------
void STEP_Motor_Disable(char motor_id) {
  const int dis_pin = (motor_id == MOTOR_R) ? STEPR_DIS_PIN : STEPL_DIS_PIN;
  digitalWrite(dis_pin, HIGH);
}

// ------------------------------------------------------
// Emit a single step pulse
// ------------------------------------------------------
void STEP_Motor_Timed_Step(char motor_id) {
  StepperMotor* motor = (motor_id == MOTOR_R) ? &motor_right : &motor_left;
  const int step_pin  = (motor_id == MOTOR_R) ? STEPR_STEP_PIN : STEPL_STEP_PIN;

  if (micros() - motor->last_step_time_us >= motor->step_interval_us) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(5);               // safe pulse width
    digitalWrite(step_pin, LOW);
    motor->last_step_time_us = micros();
    motor->steps_remaining--;
  }
}

// ------------------------------------------------------
// Main loop: serial parsing + FSM
// ------------------------------------------------------
void STEP_Motor_Test() {
  // ---- Serial input parsing ------------------------------------------------
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int first  = input.indexOf(' ');
    if (first == -1) { Serial.println("Invalid format. Use 'motor steps rps'."); return; }

    char motor_id = input.substring(0, first).charAt(0);
    if (motor_id != 'R' && motor_id != 'L') { Serial.println("Invalid motor ID. Use 'R' or 'L'."); return; }

    String rest = input.substring(first + 1);
    int second  = rest.indexOf(' ');
    if (second == -1) { Serial.println("Invalid format. Use 'motor steps rps'."); return; }

    long  steps = rest.substring(0, second).toInt();
    float rps   = rest.substring(second + 1).toFloat();

    Serial.print("Starting motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.print(steps);
    Serial.print(" steps at ");
    Serial.print(rps);
    Serial.println(" RPS.");

    STEP_Motor_Start(motor_id, steps, rps);
  }

  // ---- FSM: Right motor ----------------------------------------------------
  switch (state_right) {
    case MOTOR_IDLE:   break;
    case MOTOR_START:  break;   // handled inside STEP_Motor_Start()
    case MOTOR_RAMP: {
        float elapsed_s = (millis() - motor_right.ramp_start_time_ms) / 1000.0f;
        float new_rps   = MAX_COLD_START_RPS + RAMP_ACCEL * elapsed_s;
        if (new_rps >= motor_right.target_rps) { new_rps = motor_right.target_rps; state_right = MOTOR_RUN; }
        if (new_rps != motor_right.current_rps) {
          motor_right.current_rps      = new_rps;
          motor_right.step_interval_us = 1000000UL / (new_rps * STEPS_PER_REV);
        }
        break;
      }
    case MOTOR_RUN:   break;
    case MOTOR_STOP:
      STEP_Motor_Disable(MOTOR_R);
      Serial.println("Right motor stopped.");
      state_right = MOTOR_IDLE;
      break;
  }

  // ---- FSM: Left motor -----------------------------------------------------
  switch (state_left) {
    case MOTOR_IDLE:   break;
    case MOTOR_START:  break;
    case MOTOR_RAMP: {
        float elapsed_s = (millis() - motor_left.ramp_start_time_ms) / 1000.0f;
        float new_rps   = MAX_COLD_START_RPS + RAMP_ACCEL * elapsed_s;
        if (new_rps >= motor_left.target_rps) { new_rps = motor_left.target_rps; state_left = MOTOR_RUN; }
        if (new_rps != motor_left.current_rps) {
          motor_left.current_rps      = new_rps;
          motor_left.step_interval_us = 1000000UL / (new_rps * STEPS_PER_REV);
        }
        break;
      }
    case MOTOR_RUN:   break;
    case MOTOR_STOP:
      STEP_Motor_Disable(MOTOR_L);
      Serial.println("Left motor stopped.");
      state_left = MOTOR_IDLE;
      break;
  }

  // ---- Step execution -------------------------------------------------------
  if (state_right == MOTOR_RAMP || state_right == MOTOR_RUN) {
    if (motor_right.steps_remaining > 0) STEP_Motor_Timed_Step(MOTOR_R);
    else                                   state_right = MOTOR_STOP;
  }
  if (state_left == MOTOR_RAMP || state_left == MOTOR_RUN) {
    if (motor_left.steps_remaining > 0) STEP_Motor_Timed_Step(MOTOR_L);
    else                                 state_left = MOTOR_STOP;
  }
}

// ======================================================