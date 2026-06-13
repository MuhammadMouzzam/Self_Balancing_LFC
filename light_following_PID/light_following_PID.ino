#include <Wire.h>
#include <math.h>

// =====================================================
// L298N Motor Driver Pins (ESP32-S3)
// =====================================================
const int ENA = 4;
const int IN1 = 5;
const int IN2 = 6;

const int IN3 = 7;
const int IN4 = 12;
const int ENB = 13;

// =====================================================
// MPU6050 I2C Pins
// =====================================================
const int S3_SDA = 8;
const int S3_SCL = 9;

const int MPU_ADDR = 0x68;

// =====================================================
// LDR ADC Pins
// =====================================================
// Your used pins:
// Motor pins: 4,5,6,7,12,13
// I2C pins:   8,9
//
// So use free ADC pins for LDRs.
const int LDR_LEFT_PIN  = 1;
const int LDR_RIGHT_PIN = 2;

// =====================================================
// Timing
// =====================================================
unsigned long lastTime = 0;
const float dt = 0.01; // 10ms loop = 100Hz

// Lab 13 slow light-following loop
unsigned long lastLightTime = 0;
const unsigned long LIGHT_LOOP_PERIOD_MS = 100; // 10Hz
const float lightDt = 0.1;

// =====================================================
// Angle Variables
// =====================================================
float currentAngle = 0.0;

// =====================================================
// PID Gains
// SELF BALANCING PID - UNCHANGED
// =====================================================
float Kp = 45.0;
float Ki = 15.0;
float Kd = 0.2;

float integral = 0.0;
float prevError = 0.0;

const float desiredAngle = -5.0;

// =====================================================
// Light Following PID - Slow Outer Loop
// =====================================================
// Start weak so it does not disturb balance.
float Kp_light = 0.2;
float Ki_light = 0.1;
float Kd_light = 0.0;

float lightIntegral = 0.0;
float prevLightError = 0.0;

float directionControl = 0.0;

// Keep this small initially.
// Since PWM is 9-bit, max is 511. A direction limit of 40-70 is safe.
const float MAX_DIR_CONTROL = 50.0;

// LDR filtering
float L_left_filtered = 0.0;
float L_right_filtered = 0.0;

const float LDR_FILTER_ALPHA = 0.8;

// If your voltage divider is:
// 3.3V ---- LDR ---- ADC ---- 10k ---- GND
// then more light gives higher ADC reading, so keep this false.
//
// If your divider is reversed and more light gives lower ADC reading,
// set this true.
const bool INVERT_LDR_READING = false;

// =====================================================
// PWM Settings
// SELF BALANCING PWM - UNCHANGED
// =====================================================
const int PWM_FREQ = 15000;
const int PWM_RESOLUTION = 9;

const int MIN_PWM = 90;
const int MAX_PWM = 511;

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  // -----------------------------
  // Motor Pins
  // -----------------------------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // -----------------------------
  // LDR Pins
  // -----------------------------
  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  analogReadResolution(12); // ESP32 ADC: 0 to 4095

  // -----------------------------
  // PWM Setup (ESP32-S3 NEW API)
  // -----------------------------
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  // -----------------------------
  // MPU6050 Setup
  // -----------------------------
  Wire.begin(S3_SDA, S3_SCL);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(1000);

  Serial.println("Self Balancing Light Following Robot Started");
  Serial.println("Lab 13 MIMO Control: Fast Balance Loop + Slow Light Loop");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  unsigned long currentTime = millis();

  // =====================================================
  // SLOW LOOP: Light Following at 10Hz
  // This is separated from the fast balance loop.
  // =====================================================
  if (currentTime - lastLightTime >= LIGHT_LOOP_PERIOD_MS) {

    lastLightTime = currentTime;

    updateLightFollowing();
  }

  // =====================================================
  // FAST LOOP: Self Balancing at 100Hz
  // SELF BALANCING CODE BELOW IS KEPT SAME
  // =====================================================
  if (currentTime - lastTime >= 10) {

    lastTime = currentTime;

    // =================================================
    // Read MPU6050
    // =================================================
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    Wire.read() << 8 | Wire.read(); // Skip Temp

    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    // =================================================
    // Angle Calculation
    // UNCHANGED
    // =================================================
    float accAngle = (atan2(ay, ax) * 180.0 / PI);

    float gyroRate = -gz/ 131.0;

    // Complementary Filter
    // UNCHANGED
    currentAngle =
      0.9 * (currentAngle + gyroRate * dt) +
      0.1 * accAngle;

    // =================================================
    // PID Controller
    // UNCHANGED
    // =================================================
    float error = desiredAngle - currentAngle;

    integral += error * dt;

    float derivative = (error - prevError) / dt;

    float controlSignal =
      (Kp * error) +
      (Ki * integral) +
      (Kd * derivative);

    prevError = error;

    // =================================================
    // Lab 13 MIMO Integration
    //
    // Your balance output is controlSignal = u_bal.
    // Light-following output is directionControl = u_dir.
    //
    // Manual equation:
    // u_left  = u_bal + u_dir
    // u_right = u_bal - u_dir
    // =================================================
    driveMotorsMIMO(controlSignal, directionControl);

    // =================================================
    // Serial Monitor
    // =================================================
    Serial.print("Angle: ");
    Serial.print(currentAngle);

    Serial.print("   Balance Control: ");
    Serial.print(controlSignal);

    Serial.print("   Direction Control: ");
    Serial.print(directionControl);

    Serial.print("   L_left: ");
    Serial.print(L_left_filtered);

    Serial.print("   L_right: ");
    Serial.print(L_right_filtered);

    Serial.println();
  }
}

// =====================================================
// LIGHT FOLLOWING LOOP
// Slow outer loop for Lab 13
// =====================================================
void updateLightFollowing() {

  int rawLeft = analogRead(LDR_LEFT_PIN);
  int rawRight = analogRead(LDR_RIGHT_PIN);

  if (INVERT_LDR_READING) {
    rawLeft = 4095 - rawLeft;
    rawRight = 4095 - rawRight;
  }

  // Low-pass filtering to reduce LDR noise
  L_left_filtered = LDR_FILTER_ALPHA * L_left_filtered + (1.0 - LDR_FILTER_ALPHA) * rawLeft; //LDR_FILTER_ALPHA * L_left_filtered + (1.0 - LDR_FILTER_ALPHA) *

  L_right_filtered = LDR_FILTER_ALPHA * L_right_filtered + (1.0 - LDR_FILTER_ALPHA) * rawRight; //LDR_FILTER_ALPHA * L_right_filtered + (1.0 - LDR_FILTER_ALPHA) * 

  // Normalize light error so room brightness does not dominate
  float lightSum = abs(L_left_filtered - L_right_filtered) + 1.0;

  float error_light =
    (L_left_filtered - L_right_filtered)/10;

  // Optional deadband: ignore tiny differences
  if (abs(error_light) < 50.0) {
    error_light = 0.0;
  }

  lightIntegral += error_light * lightDt;

  // Prevent light integral windup
  lightIntegral = constrain(lightIntegral, -10.0, 10.0);

  float lightDerivative =
    (error_light - prevLightError) / lightDt;

  directionControl =
    (Kp_light * error_light) +
    (Ki_light * lightIntegral) +
    (Kd_light * lightDerivative);

  directionControl =
    constrain(directionControl, -MAX_DIR_CONTROL, MAX_DIR_CONTROL);

  prevLightError = error_light;

  Serial.print("Light Loop -> ");
  Serial.print("L_left: ");
  Serial.print(L_left_filtered);

  Serial.print("   L_right: ");
  Serial.print(L_right_filtered);

  Serial.print("   error_light: ");
  Serial.print(error_light, 4);

  Serial.print("   u_dir: ");
  Serial.println(directionControl);
}

// =====================================================
// MIMO MOTOR CONTROL FUNCTION
// Lab 13 Differential Motor Command
// =====================================================
void driveMotorsMIMO(float balanceControl, float dirControl) {

  float leftCommand  = balanceControl + dirControl;
  float rightCommand = balanceControl - dirControl;

  // Limit command so MIN_PWM + abs(command) does not exceed MAX_PWM
  float maxCommand = MAX_PWM - MIN_PWM;

  leftCommand  = constrain(leftCommand, -maxCommand, maxCommand);
  rightCommand = constrain(rightCommand, -maxCommand, maxCommand);

  int leftSpeed = 0;
  int rightSpeed = 0;

  if (leftCommand != 0) {
    leftSpeed = MIN_PWM + (int)abs(leftCommand);
    leftSpeed = constrain(leftSpeed, 0, MAX_PWM);
  }

  if (rightCommand != 0) {
    rightSpeed = MIN_PWM + (int)abs(rightCommand);
    rightSpeed = constrain(rightSpeed, 0, MAX_PWM);
  }

  // =================================================
  // Left Motor Direction
  // Uses same direction logic as your original code
  // =================================================
  if (leftCommand > 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  else if (leftCommand < 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  else {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    leftSpeed = 0;
  }

  // =================================================
  // Right Motor Direction
  // Uses same direction logic as your original code
  // =================================================
  if (rightCommand > 0) {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else if (rightCommand < 0) {

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else {

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    rightSpeed = 0;
  }

  // =================================================
  // Apply PWM separately
  // =================================================
  ledcWrite(ENA, leftSpeed);
  ledcWrite(ENB, rightSpeed);

  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);

  Serial.print("   Right Speed: ");
  Serial.println(rightSpeed);
}

// =====================================================
// ORIGINAL MOTOR CONTROL FUNCTION
// Kept here unchanged as backup.
// Not used in Lab 13 MIMO mode.
// =====================================================
void driveMotors(float controlSignal) {

  int motorSpeed =
  constrain(MIN_PWM + (int)abs(controlSignal), 0, MAX_PWM);

  // =================================================
  // Forward
  // =================================================
  if (controlSignal > 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // =================================================
  // Backward
  // =================================================
  else if (controlSignal < 0) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  // =================================================
  // Stop
  // =================================================
  else {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    motorSpeed = 0;
  }

  // =================================================
  // Apply PWM
  // =================================================
  ledcWrite(ENA, motorSpeed);
  ledcWrite(ENB, motorSpeed);

  Serial.print("Speed: ");
  Serial.println(motorSpeed);
}