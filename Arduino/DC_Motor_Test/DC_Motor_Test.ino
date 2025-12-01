// Pin assignments and #includes
#define DC1_PWM_PIN 3  // PWM speed control
#define DC1_DIR_PIN 4  // Direction
#define DC1_FG_PIN 2    // Encoder pulses (needs 5V pull-up resistor)

// ======================================================

// Component setup
  // FIT0441 Brushless DC Motor Control with Encoder
const int PULSES_PER_REV = 45 * 6 * 2;  // 45:1 gearbox, 6 positions per motor rev, 2 pulses per position
const float PULSES_PER_RADIAN = PULSES_PER_REV / (2.0 * PI); 
volatile unsigned long DC1_Pulses = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long debounce_us = 500;
float DC1_TargetRadPerSec = 0.0;
float DC1_MeasuredRadPerSec = 0.0;
int DC1_PWM = 255;  // 0: full speed, 255: stop. Motor cuts out at DC1_Max_PWM. Not linear with output velocity
#define DC1_Min_PWM 0
#define DC1_Max_PWM 247
bool DC1_Direction = HIGH;  // HIGH = forward, LOW = reverse

void FIT0441_Setup() {
  pinMode(DC1_PWM_PIN, OUTPUT);
  pinMode(DC1_DIR_PIN, OUTPUT);
  pinMode(DC1_FG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DC1_FG_PIN), countPulse1, HIGH);

  // Initialize motor stopped for set forward
  analogWrite(DC1_PWM_PIN, 255);
  digitalWrite(DC1_DIR_PIN, HIGH);

  // User serial interface
  Serial.println(F("=== FIT0441 DC Motor with Encorder ==="));
  Serial.print(F("Encoder pin: "));
  Serial.println(DC1_FG_PIN);
}
// End of component setup

// ======================================================

// Function Declarations
void DC_Motor_Stop(int motorNum=1);
void DC_Motor_Start(int motorNum=1, int PWM=240, bool direction=HIGH);
int Get_DC_Motor_Pulses(int motorNum=1);
void DC_Motor_Speed_Test();
void DC_Motor_Displacement_Test();
// End of function declarations

// ======================================================

// Start of Arduino execution
void setup() {
  Serial.begin(115200);
  FIT0441_Setup();
}

void loop() {
  // DC_Motor_Speed_Test();
  DC_Motor_Displacement_Test();
}
// End of Arduino execution

// ======================================================

// Function Definitions
void DC_Motor_Stop(int motorNum=1) {
  if (motorNum == 1) {
    analogWrite(DC1_PWM_PIN, 255);
  }
}
void DC_Motor_Start(int motorNum=1, int PWM=240, bool direction=HIGH) {
  if (motorNum == 1) {
    // Stop motor and reset pulse counter to ensure accurate starting reference
    DC_Motor_Stop(motorNum);
    noInterrupts();
    DC1_Pulses = 0;
    interrupts();

    // Start motor with direction and speed
    digitalWrite(DC1_DIR_PIN, direction);
    analogWrite(DC1_PWM_PIN, PWM);
  }
}
int Get_DC_Motor_Pulses(int motorNum=1) {
  if (motorNum == 1) {
    noInterrupts();
    unsigned long DC1_Pulses_temp = DC1_Pulses;
    interrupts();

    return DC1_Pulses_temp;
  }
}
void DC_Motor_Speed_Test() {
  static unsigned long messageTime = 0;
  static unsigned long prevMessageTime = 0;
  static unsigned long currTime = micros();
  static unsigned long prev_DC1_Pulses = 0;
  static unsigned int pulses = 0;
  static float DC1_radsec = 0;

  // Serial Input: Motor PWM
  if (Serial.available()) {
    int commanded_PWM = Serial.parseInt();
    if (commanded_PWM != 0 || Serial.peek() == '-') {
      DC1_PWM = abs(commanded_PWM);
      analogWrite(DC1_PWM_PIN, DC1_PWM);

      DC1_Direction = (commanded_PWM >= 0) ? HIGH : LOW;
      digitalWrite(DC1_DIR_PIN, DC1_Direction);

      Serial.print("Target: ");
      Serial.println(commanded_PWM);
    }
    while (Serial.available()) Serial.read();  // Clear buffer by reading to end of stored data
  }

  // Report current speed
  currTime = micros();
  if (currTime - messageTime > 1000000) {
    messageTime = currTime;

    unsigned long DC1_Pulses_temp = Get_DC_Motor_Pulses(1);
   
    unsigned long duration_ms = messageTime - prevMessageTime;
    float duration_sec = duration_ms / 1000000.0f;
    pulses = DC1_Pulses_temp - prev_DC1_Pulses;
    float DC1_revsec = (float)pulses / (PULSES_PER_REV * duration_sec);
    DC1_radsec = DC1_revsec * 2.0f * PI;

    Serial.print("DC1 PWM: ");
    Serial.print(DC1_PWM);
    Serial.print("\tDC1 Pulses: ");
    Serial.print(pulses);
    Serial.print("\tDuration: ");
    Serial.print(duration_sec, 3);
    Serial.print("\tDC1 rev/s: ");
    Serial.println(DC1_revsec, 3);

    prevMessageTime = messageTime;
    prev_DC1_Pulses = DC1_Pulses_temp;
  }
}
void DC_Motor_Displacement_Test() {
  static int commanded_steps = 0;
  static int measured_steps = 0;
  static float measured_angle = 0.0;
  static bool running = false;  // State flag: true when motor is pursuing a target
  static bool direction = HIGH; // Motor direction: HIGH for positive, LOW for negative
  static int leg = 1;
  static int total_steps = 0;

  // Serial Input: Motor angular displacement
  if (Serial.available()) {
    float commanded_degrees = Serial.parseFloat();
    
    if (commanded_degrees != 0 || Serial.peek() == '-') {
      DC_Motor_Stop(1);
      
      float commanded_radians = commanded_degrees * PI / 180.0;
      direction = (commanded_degrees >= 0) ? HIGH : LOW;
      commanded_steps = round(fabs(commanded_radians) * PULSES_PER_RADIAN);
      
      Serial.print("\nTarget Angle: ");
      Serial.print(commanded_degrees);
      Serial.print("\tTarget Steps: ");
      Serial.println(commanded_steps);

      // Start the motor and set running state
      DC_Motor_Start(1, 200, direction);
      running = true;
    }
    while (Serial.available()) Serial.read();  // Clear buffer by reading to end of stored data
  }

  // If running, monitor pulse count periodically
  if (running) {
    measured_steps = Get_DC_Motor_Pulses(1);

    if (measured_steps >= commanded_steps) {
      DC_Motor_Stop(1);
      if (direction == HIGH) {
        total_steps += measured_steps;
      } else {
        total_steps -= measured_steps;
      }

      leg++;
      if (leg > 4) {
        running = false;
        float difference_angle = (total_steps / PULSES_PER_RADIAN) * (180.0 / PI);
        leg = 1;
        Serial.print("Measured Difference: ");
        Serial.println(difference_angle, 2);


      } else {
        direction = (leg % 2 == 1) ? HIGH : LOW;
        delay(500);
        DC_Motor_Start(1, 200, direction);
      }
    }
  }
}
// End of function definitions

// ======================================================

// Interrupt Service Routines (ISR)
void countPulse1() {
  unsigned long now = micros();
  if (now - lastPulseTime > debounce_us) {
    DC1_Pulses++;
    lastPulseTime = now;
  }
}
// ======================================================