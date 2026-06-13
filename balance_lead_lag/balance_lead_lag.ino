// =====================================================================
// Lab 15 – Lead-Lag Controller for Self-Balancing Robot
// CEP Part 4 (Final) – UET Lahore Control Systems Lab
//
// Authors      : Muhammad Mouzzam (2023-EE-111)
//                Umair Nadeem     (2023-EE-115)
// Instructors  : Dr. Habib Wajid | Prof. Hanan Tariq
//
// Hardware     : ESP32-S3 N16R8
//                MPU6050 IMU (I2C: SDA=8, SCL=9)
//                L298N Motor Driver
//                Left Motor  – ENA=4,  IN1=5, IN2=6
//                Right Motor – ENB=13, IN3=7, IN4=12
//
// Controller   : Discrete lead-lag compensator (Tustin, Ts=0.01 s)
//   Designed from Lab 14 identified model:
//     G(s) = 6.2117 / (s^2 - 10.0512)
//   Lead:  C_lead(s) = (s+3)/(s+25)
//   Lag :  C_lag(s)  = (s+0.5)/(s+0.05)
//   K = 30
//   Combined: C(s) = 30 · C_lead(s) · C_lag(s)
//           = 30(s^2 + 3.5s + 1.5) / (s^2 + 25.05s + 1.25)
//
// Discretisation (Tustin / bilinear, Ts = 0.01 s):
//   C(z) = (b0 + b1·z^-1 + b2·z^-2) / (1 + a1·z^-1 + a2·z^-2)
//
// Difference equation (2nd-order IIR):
//   u[k] = b0·e[k] + b1·e[k-1] + b2·e[k-2]
//          - a1·u[k-1] - a2·u[k-2]
//
// Computed with MATLAB/SciPy (scipy.signal.cont2discrete, 'bilinear'):
//   b0 =  27.12755145,  b1 = -53.31800383,  b2 = 26.19445139
//   a1 =  -1.77727790,  a2 =   0.77738899
// =====================================================================

#include <Wire.h>
#include <math.h>

// =====================================================================
// MOTOR DRIVER PINS
// =====================================================================
const int ENA = 4;   // PWM – left  motor
const int IN1 = 5;
const int IN2 = 6;

const int IN3 = 7;
const int IN4 = 12;
const int ENB = 13;  // PWM – right motor

// =====================================================================
// MPU6050
// =====================================================================
const int  S3_SDA   = 8;
const int  S3_SCL   = 9;
const int  MPU_ADDR = 0x68;

// =====================================================================
// TIMING
// =====================================================================
const float   Ts       = 0.01f;   // 10 ms = 100 Hz
unsigned long lastTime = 0;

// =====================================================================
// ANGLE ESTIMATION
// =====================================================================
float currentAngle = 0.0f;
const float ALPHA  = 0.9f;        // complementary filter weight

// =====================================================================
// SETPOINT
// =====================================================================
const float desiredAngle = -5.0f; // degrees (forward-lean from Lab 13)

// =====================================================================
// DISCRETE LEAD-LAG COEFFICIENTS  (Tustin, Ts = 0.01 s)
//
// C(s) = 30*(s+3)/(s+25) * (s+0.5)/(s+0.05)   [2nd-order continuous]
//
//   b (numerator)  : b0   b1   b2
//   a (denominator): 1.0  a1   a2
//
// Note: C(s) is 2nd order → C(z) is 2nd order (3 b-coefficients,
//       2 a-coefficients after normalisation).  The original ino file
//       erroneously used a 3rd-order structure; this is corrected here.
// =====================================================================
const int CTRL_ORDER = 2;   // 2nd-order IIR

// Numerator coefficients
const float b_coef[CTRL_ORDER + 1] = {
     27.12755145f,   // b0
    -53.31800383f,   // b1
     26.19445139f    // b2
};

// Denominator coefficients (a0 = 1 always)
const float a_coef[CTRL_ORDER + 1] = {
     1.00000000f,    // a0 = 1  (normalised, not used in loop)
    -1.77727790f,    // a1
     0.77738899f     // a2
};

// Controller state buffers
float e_buf[CTRL_ORDER + 1] = {0.0f};  // e[k], e[k-1], e[k-2]
float u_buf[CTRL_ORDER + 1] = {0.0f};  // u[k], u[k-1], u[k-2]

// =====================================================================
// PWM SETTINGS (ESP32-S3, LEDC peripheral)
// =====================================================================
const int PWM_FREQ       = 15000;   // Hz
const int PWM_RESOLUTION = 9;       // bits  → range 0..511
const int MIN_PWM        = 90;      // deadband compensation
const int MAX_PWM        = 511;     // full duty

// =====================================================================
// SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);

    // Motor direction pins
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    // PWM (ESP32-S3 Arduino core ≥ 2.x new API)
    ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);

    // MPU6050 initialisation
    Wire.begin(S3_SDA, S3_SCL);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);   // PWR_MGMT_1 register
    Wire.write(0x00);   // wake up (clear sleep bit)
    Wire.endTransmission(true);

    // Optional: set gyro to ±250 deg/s (register 0x1B, value 0x00)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Optional: set accelerometer to ±2g (register 0x1C, value 0x00)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    delay(200);   // allow MPU6050 to stabilise

    Serial.println("Lab 15 - Lead-Lag Self-Balancing Robot");
    Serial.println("C(s) = 30*(s+3)/(s+25)*(s+0.5)/(s+0.05)");
    Serial.println("Discrete (Tustin, 100 Hz): 2nd-order IIR");
    Serial.println("b = [27.12755, -53.31800, 26.19445]");
    Serial.println("a = [1.00000, -1.77728,   0.77739]");
    Serial.println("Time[s],Angle[deg],Error[deg],Control[u]");
}

// =====================================================================
// MAIN LOOP  (100 Hz)
// =====================================================================
void loop() {
    unsigned long now = millis();

    if (now - lastTime >= (unsigned long)(Ts * 1000.0f)) {
        lastTime = now;

        // =============================================================
        // 1. READ MPU6050  (14 bytes: 3×accel + temp + 3×gyro)
        // =============================================================
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);          // start at ACCEL_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);

        int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
        Wire.read(); Wire.read();  // skip temperature (2 bytes)
        int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

        // =============================================================
        // 2. ANGLE ESTIMATION  (complementary filter, alpha = 0.9)
        //    Tilt angle around the wheel axis uses ay and az.
        //    Gyro rate uses gy (roll rate, not gz).
        //    Adjust axes to match physical robot orientation.
        // =============================================================
        // Accelerometer angle (static, noisy)
        float accAngle = atan2f((float)ay, (float)az) * 180.0f / (float)M_PI;

        // Gyroscope rate (dynamic, drifts)
        // ±250 deg/s range → 131 LSB/(deg/s)
        float gyroRate = (float)gy / 131.0f;   // deg/s  (use gy for pitch)

        // Complementary filter
        currentAngle = ALPHA * (currentAngle + gyroRate * Ts)
                     + (1.0f - ALPHA) * accAngle;

        // =============================================================
        // 3. LEAD-LAG CONTROL LAW  (2nd-order difference equation)
        //
        //   Shift error buffer:   e[k-2]←e[k-1]←e[k]
        //   Shift control buffer: u[k-2]←u[k-1]
        //   Apply: u[k] = b0·e[k] + b1·e[k-1] + b2·e[k-2]
        //               - a1·u[k-1] - a2·u[k-2]
        // =============================================================
        float error = desiredAngle - currentAngle;

        // Shift error buffer (index 0 = newest)
        e_buf[2] = e_buf[1];
        e_buf[1] = e_buf[0];
        e_buf[0] = error;

        // Shift output buffer
        u_buf[2] = u_buf[1];
        u_buf[1] = u_buf[0];

        // Difference equation
        float u_new =
              b_coef[0] * e_buf[0]
            + b_coef[1] * e_buf[1]
            + b_coef[2] * e_buf[2]
            - a_coef[1] * u_buf[1]
            - a_coef[2] * u_buf[2];

        // Clamp to prevent actuator saturation / integral windup
        u_new = constrain(u_new,
                          -(float)(MAX_PWM - MIN_PWM),
                           (float)(MAX_PWM - MIN_PWM));

        u_buf[0] = u_new;

        // =============================================================
        // 4. DRIVE MOTORS
        // =============================================================
        driveMotors(u_new);

        // =============================================================
        // 5. SERIAL LOGGING  (CSV, import into MATLAB or Python)
        // =============================================================
        Serial.print(now / 1000.0f, 4);
        Serial.print(",");
        Serial.print(currentAngle, 4);
        Serial.print(",");
        Serial.print(error, 4);
        Serial.print(",");
        Serial.println(u_new, 4);
    }
}

// =====================================================================
// MOTOR CONTROL
//   controlSignal > 0  → forward
//   controlSignal < 0  → backward
//   controlSignal == 0 → stop
// =====================================================================
void driveMotors(float controlSignal) {

    int motorSpeed = MIN_PWM + (int)fabsf(controlSignal);
    motorSpeed = constrain(motorSpeed, 0, MAX_PWM);

    if (controlSignal > 1.0f) {
        // Forward
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    }
    else if (controlSignal < -1.0f) {
        // Backward
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }
    else {
        // Stop (deadband)
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
        motorSpeed = 0;
    }

    ledcWrite(ENA, motorSpeed);
    ledcWrite(ENB, motorSpeed);
}
