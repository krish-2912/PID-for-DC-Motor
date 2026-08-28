#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ======================================================
// GEOMETRY & KINEMATICS
// ======================================================
const float GEOM = 0.250 + 0.275; // Lx + Ly
const float WHEEL_RADIUS = 0.050; // Wheel radius in meters (Adjust to your exact mecanum wheel size)
const float MS_TO_RPM = 60.0f / (2.0f * PI * WHEEL_RADIUS);

// ======================================================
// ENCODER MATH
// ======================================================
// Accounting for the 19.2:1 gearbox ratio to match physical tachometer
const float GEAR_RATIO = 19.2f;
const float BASE_PPR = 14.0f; // Base pulses per revolution of the magnetic disk
const float CPR = BASE_PPR * GEAR_RATIO * 2.0f; // *2 for attachHalfQuad

// ======================================================
// IMU & MOTION STATE
// ======================================================
MPU6050 mpu(Wire);
const int I2C_SDA = 25;
const int I2C_SCL = 26;

float bias_gz = 0.0;
float yaw = 0.0;
float old_gz = 0.0;
float p1 = 0.7; // Smoothing factor
float p2 = 0.3; // 1 - p1
const float GYRO_THRESHOLD = 0.05;
int rotation_stationary_counter = 0;
const int ROTATION_STATIONARY_THRESHOLD = 5;

unsigned long lastUpdate = 0;
unsigned long lastPidUpdate = 0;
const uint32_t PID_SAMPLE_MS = 30;

float targetVX = 0, targetVY = 0, targetWZ = 0;
float currentVX = 0, currentVY = 0, currentWZ = 0;
const float ACCEL_STEP = 0.04; 

// ======================================================
// MOTOR CLASS DEFINITION (Bi-Directional IMC PID)
// ======================================================
class Motor {
  public:
    String name;
    int encA, encB, motF, motR;
    float K_model, tau, Kp, Ki, Kff;
    
    float integral = 0.0f;
    long lastEncCount = 0;
    float currentRPM = 0.0f;
    float filteredRPM = 0.0f;
    float setpointRPM = 0.0f;

    const float lambda = 0.1f;
    const float Kaw = 0.2f;
    const float I_MAX = 100.0f;
    const float alpha = 0.3f;
    
    ESP32Encoder encoder;

    Motor(String n, int ea, int eb, int mf, int mr, float k, float t) 
      : name(n), encA(ea), encB(eb), motF(mf), motR(mr), K_model(k), tau(t) {}

    void init() {
      pinMode(motF, OUTPUT);
      pinMode(motR, OUTPUT);
      
      encoder.attachHalfQuad(encA, encB);
      encoder.setCount(0);

      // Auto-Tune (IMC)
      Kp = tau / (K_model * lambda);
      Ki = Kp / tau;
      Kff = 1.0f / max(K_model, 0.01f);
    }

    void update(uint32_t dt_ms) {
      // 1. Calculate RPM (Handles both forward and reverse)
      long count = encoder.getCount();
      long delta = count - lastEncCount;
      float rawRPM = (float(delta) / CPR) * (60000.0f / dt_ms);
      
      filteredRPM = alpha * rawRPM + (1 - alpha) * filteredRPM;
      currentRPM = filteredRPM;
      lastEncCount = count;

      // ==========================================
      // THE FIX: Zero-Setpoint Cutoff
      // If we want to stop, clear the integral and kill power.
      // ==========================================
      if (abs(setpointRPM) < 0.1f) {
        integral = 0.0f;
        applyPWM(0);
        return; // Skip the rest of the PID calculations
      }

      // 2. PID Control
      float error = setpointRPM - currentRPM;
      
      // Deadband commented out to prevent integral stalling when close to target
      // if (abs(error) < 1.0f) error = 0; 

      if (isnan(error) || isinf(error)) {
        integral = 0;
        applyPWM(0);
        return;
      }

      error = constrain(error, -500, 500);
      float dt_s = dt_ms / 1000.0f;

      float u_ff = setpointRPM * Kff;
      integral += error * dt_s;
      integral = constrain(integral, -I_MAX, I_MAX);

      float u_pid = Kp * error + Ki * integral;
      float u = u_ff + u_pid;

      if (isnan(u) || isinf(u)) { integral = 0; u = 0; }

      // Saturation (-255 to 255 for bi-directional drive)
      float u_sat = constrain(u, -255.0f, 255.0f);

      // Anti-windup
      float windup = u_sat - u;
      windup = constrain(windup, -50, 50);
      integral += Kaw * windup;

      // Apply
      applyPWM(u_sat);
    }

    void applyPWM(float pwm) {
      int p = constrain((int)abs(pwm), 0, 255);
      // Incorporating your 15 PWM deadband safety check
      if (pwm > 15) { 
        analogWrite(motF, p); 
        analogWrite(motR, 0); 
      } else if (pwm < -15) { 
        analogWrite(motF, 0); 
        analogWrite(motR, p); 
      } else { 
        analogWrite(motF, 0); 
        analogWrite(motR, 0); 
      }
    }
};

// ======================================================
// INSTANTIATE MOTORS
// ======================================================
// (Name, EncA, EncB, MotF, MotR, K_model, Tau)
Motor flMotor("FL", 16, 17, 18, 19, 2.016f, 0.039f);
Motor frMotor("FR", 34, 35,  5, 13, 2.105f, 0.031f);
Motor blMotor("BL", 21, 22, 23,  4, 2.077f, 0.029f);
Motor brMotor("BR", 32, 33, 14, 27, 1.954f, 0.044f);

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  flMotor.init();
  frMotor.init();
  blMotor.init();
  brMotor.init();

  Wire.begin(I2C_SDA, I2C_SCL);
  if (mpu.begin() != 0) {
    while (1) { Serial.println("MPU6050 Error"); delay(1000); }
  }
  
  Serial.println("Calibrating IMU... Keep still.");
  float gz_sum = 0;
  for (int i = 0; i < 200; i++) {
    mpu.update();
    gz_sum += mpu.getGyroZ(); 
    delay(10);
  }
  bias_gz = (gz_sum / 200.0) * (PI / 180.0);
  
  yaw = 0.0;
  lastUpdate = millis();
  lastPidUpdate = millis();
}

// ======================================================
// RAMP FUNCTION
// ======================================================
float ramp(float current, float target, float step) {
  if (abs(current - target) < step) return target;
  return (current < target) ? (current + step) : (current - step);
}

// ======================================================
// MAIN LOOP
// ======================================================
void loop() {
  mpu.update();
  unsigned long currentTime = millis();
  
  // ----------------------------------------------------
  // 1. IMU Processing
  // ----------------------------------------------------
  float dt_imu = (currentTime - lastUpdate) / 1000.0;
  if (dt_imu >= 0.01) { 
    float measured_gz = (mpu.getGyroZ() * (PI / 180.0)) - bias_gz;
    float gz = (p1 * old_gz) + (p2 * measured_gz);
    old_gz = gz;

    if (abs(gz) < GYRO_THRESHOLD) {
      rotation_stationary_counter++;
    } else {
      rotation_stationary_counter = 0;
    }

    if (rotation_stationary_counter <= ROTATION_STATIONARY_THRESHOLD) {
      yaw += gz * dt_imu;
    }

    if (yaw > PI) yaw -= 2 * PI;
    if (yaw < -PI) yaw += 2 * PI;

    lastUpdate = currentTime;
  }

  // ----------------------------------------------------
  // 2. Read Serial Commands from Pi Bridge
  // ----------------------------------------------------
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      targetVX = data.substring(0, c1).toFloat();
      targetVY = data.substring(c1 + 1, c2).toFloat();
      targetWZ = data.substring(c2 + 1).toFloat();
    }
  }

  // ----------------------------------------------------
  // 3. Trajectory Ramping & Kinematics Update (25ms)
  // ----------------------------------------------------
  static unsigned long lastRamp = 0;
  if (currentTime - lastRamp > 25) { 
    currentVX = ramp(currentVX, targetVX, ACCEL_STEP);
    currentVY = ramp(currentVY, targetVY, ACCEL_STEP);
    currentWZ = ramp(currentWZ, targetWZ, ACCEL_STEP);

    // Inverse Kinematics -> Converts m/s to target RPM
    flMotor.setpointRPM = (currentVX + currentVY + (GEOM * currentWZ)) * MS_TO_RPM;
    frMotor.setpointRPM = (currentVX - currentVY - (GEOM * currentWZ)) * MS_TO_RPM;
    blMotor.setpointRPM = (currentVX - currentVY + (GEOM * currentWZ)) * MS_TO_RPM;
    brMotor.setpointRPM = (currentVX + currentVY - (GEOM * currentWZ)) * MS_TO_RPM;
    
    lastRamp = currentTime;
  }

  // ----------------------------------------------------
  // 4. Execute PID Control (30ms)
  // ----------------------------------------------------
  uint32_t dt_pid = currentTime - lastPidUpdate;
  if (dt_pid >= PID_SAMPLE_MS) {
    flMotor.update(dt_pid);
    frMotor.update(dt_pid);
    blMotor.update(dt_pid);
    brMotor.update(dt_pid);
    
    lastPidUpdate = currentTime;
  }

  // ----------------------------------------------------
  // 5. Send Telemetry to Pi (50ms)
  // ----------------------------------------------------
  static unsigned long lastSerialSend = 0;
  if (currentTime - lastSerialSend > 50) {
    Serial.print((long)flMotor.encoder.getCount()); Serial.print(",");
    Serial.print((long)frMotor.encoder.getCount()); Serial.print(",");
    Serial.print((long)blMotor.encoder.getCount()); Serial.print(",");
    Serial.print((long)brMotor.encoder.getCount()); Serial.print(",");
    Serial.println(yaw * (180.0 / PI));     
    lastSerialSend = currentTime;
  }
}