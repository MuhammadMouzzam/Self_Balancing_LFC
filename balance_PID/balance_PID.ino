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
// Timing
// =====================================================
unsigned long lastTime = 0;
const float dt = 0.01; // 10ms loop = 100Hz

// =====================================================
// Angle Variables
// =====================================================
float currentAngle = 0.0;

// =====================================================
// PID Gains
// =====================================================
float Kp = 45.0;
float Ki = 25.0;
float Kd = 0.2;

float integral = 0.0;
float prevError = 0.0;

const float desiredAngle = -.0;

// =====================================================
// PWM Settings
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

  Serial.println("Self Balancing Robot Started");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  unsigned long currentTime = millis();

  // 100Hz loop
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
    // =================================================
    float accAngle = (atan2(ay, ax) * 180.0 / PI);

    float gyroRate = -gz/ 131.0;

    Serial.print("Angle ");
    Serial.println(accAngle);

    Serial.print("   Gyro: ");
    Serial.println(gyroRate);

    // Complementary Filter
    currentAngle =
      0.9 * (currentAngle + gyroRate * dt) +
      0.1 * accAngle;

    // =================================================
    // PID Controller
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
    // Drive Motors
    // =================================================
    driveMotors(controlSignal);

    // =================================================
    // Serial Monitor
    // =================================================
    Serial.print("   Angle: ");
    Serial.print(currentAngle);

    Serial.print("   Control: ");
    Serial.println(controlSignal);
  }
}

// =====================================================
// MOTOR CONTROL FUNCTION
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