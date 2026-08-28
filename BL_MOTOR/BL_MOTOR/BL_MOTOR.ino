#include <Arduino.h>
#include <ESP32Encoder.h>

// ======================================================
// PIN CONFIG
// ======================================================
const int ENC_A  = 21;
const int ENC_B  = 22;
const int MOT_F  = 23;
const int MOT_R  = 4;

const int PWM_CH   = 0;
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// ======================================================
// ENCODER
// ======================================================
ESP32Encoder encoder;

// Slightly slower sampling → smoother RPM
const uint32_t SAMPLE_MS = 30;
const float CPR = 269.5f * 2.0f;

// ======================================================
// SYSTEM MODEL (from your identification)
// ======================================================
float K_model = 2.077f;
float tau     = 0.029f;

// Stability-focused tuning
float lambda = 0.1f;

// ======================================================
// CONTROLLER
// ======================================================
float Kp, Ki;
float Kff;
float Kaw = 0.2f;

// State
float integral = 0.0f;

// Limits
const float U_MAX = 255.0f;
const float U_MIN = 0.0f;
const float I_MAX = 100.0f;

// ======================================================
// RPM
// ======================================================
long lastEncCount = 0;
uint32_t lastSampleT = 0;

float currentRPM = 0.0f;
float filteredRPM = 0.0f;

// Low-pass filter
float alpha = 0.3f;

// Deadband
float deadband = 1.0f;

float setpointRPM = 200.0f;

// ======================================================
// MOTOR
// ======================================================
void motorPWM(float pwm) {
  digitalWrite(MOT_R, LOW);
  ledcWrite(PWM_CH, constrain((int)pwm, 0, 255));
}

// ======================================================
// AUTO-TUNE (IMC)
// ======================================================
void autoTune() {
  Kp = tau / (K_model * lambda);
  Ki = Kp / tau;

  Kff = 1.0f / max(K_model, 0.01f);

  Serial.println("\n=== AUTO TUNE ===");
  Serial.printf("Kp=%.3f Ki=%.3f Kff=%.3f\n", Kp, Ki, Kff);
}

// ======================================================
// RPM UPDATE
// ======================================================
bool updateRPM() {
  uint32_t now = millis();
  uint32_t dt = now - lastSampleT;

  if (dt < SAMPLE_MS) return false;

  long count = encoder.getCount();
  long delta = count - lastEncCount;

  float rawRPM = (float(delta) / CPR) * (60000.0f / dt);

  // Low-pass filter
  filteredRPM = alpha * rawRPM + (1 - alpha) * filteredRPM;
  currentRPM = filteredRPM;

  lastEncCount = count;
  lastSampleT = now;

  return true;
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A, ENC_B);
  encoder.setCount(0);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOT_F, PWM_CH);
  pinMode(MOT_R, OUTPUT);

  autoTune();

  lastSampleT = millis();

  Serial.println("time,sp,rpm,pwm,error,Kp,Ki");
}

// ======================================================
// LOOP
// ======================================================
void loop() {

  if (updateRPM()) {

    float dt = SAMPLE_MS / 1000.0f;
    float error = setpointRPM - currentRPM;

    // Deadband
    if (abs(error) < deadband) error = 0;

    // Safety
    if (isnan(error) || isinf(error)) {
      integral = 0;
      return;
    }

    error = constrain(error, -500, 500);

    // Feedforward
    float u_ff = setpointRPM * Kff;

    // Integral
    integral += error * dt;
    integral = constrain(integral, -I_MAX, I_MAX);

    // PI
    float u_pid = Kp * error + Ki * integral;

    float u = u_ff + u_pid;

    // NaN protection
    if (isnan(u) || isinf(u)) {
      integral = 0;
      u = 0;
    }

    // Saturation
    float u_sat = constrain(u, U_MIN, U_MAX);

    // Anti-windup
    float windup = u_sat - u;
    windup = constrain(windup, -50, 50);
    integral += Kaw * windup;

    // Apply
    motorPWM(u_sat);

    // Log
    Serial.printf("%lu,%.1f,%.2f,%.1f,%.2f,%.3f,%.3f\n",
      millis(), setpointRPM, currentRPM, u_sat, error, Kp, Ki);
  }

  // Serial setpoint
  if (Serial.available()) {
    float sp = Serial.parseFloat();
    if (sp > 0 && sp < 500) {
      setpointRPM = sp;
      integral = 0;

      Serial.printf("# New SP: %.1f\n", sp);
    }
  }
}