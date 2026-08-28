#include <Arduino.h>
#include <ESP32Encoder.h>

// ======================================================
// GLOBAL CONFIGURATION
// ======================================================
const uint32_t SAMPLE_MS = 30;
const float CPR = 269.5f * 2.0f;
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// Shared Controller Limits & Filters
const float lambda = 0.1f;
const float Kaw = 0.2f;
const float U_MAX = 255.0f;
const float U_MIN = 0.0f;
const float I_MAX = 100.0f;
const float alpha = 0.3f;
const float deadband = 1.0f;

// Shared Setpoint
float globalSetpointRPM = 100.0f;
uint32_t lastSampleT = 0;

// ======================================================
// MOTOR CLASS DEFINITION
// ======================================================
class Motor {
  public:
    String name;
    int encA, encB, motF, motR, pwmCh;
    float K_model, tau;

    // Controller parameters
    float Kp, Ki, Kff;
    float integral = 0.0f;

    // State
    long lastEncCount = 0;
    float currentRPM = 0.0f;
    float filteredRPM = 0.0f;

    ESP32Encoder encoder;

    // Constructor to initialize pins and system model per motor
    Motor(String n, int ea, int eb, int mf, int mr, int ch, float k, float t) 
      : name(n), encA(ea), encB(eb), motF(mf), motR(mr), pwmCh(ch), K_model(k), tau(t) {}

    void init() {
      // 1. Setup Encoder
      encoder.attachFullQuad(encA, encB);
      encoder.setCount(0);

      // 2. Setup Motor Pins
      ledcSetup(pwmCh, PWM_FREQ, PWM_RES);
      ledcAttachPin(motF, pwmCh);
      pinMode(motR, OUTPUT);

      // 3. Auto-Tune (IMC)
      Kp = tau / (K_model * lambda);
      Ki = Kp / tau;
      Kff = 1.0f / max(K_model, 0.01f);

      Serial.printf("%s Motor Tuned -> Kp: %.3f | Ki: %.3f | Kff: %.3f\n", name.c_str(), Kp, Ki, Kff);
    }

    void update(uint32_t dt_ms) {
      // --- Calculate RPM ---
      long count = encoder.getCount();
      long delta = count - lastEncCount;
      float rawRPM = (float(delta) / CPR) * (60000.0f / dt_ms);
      
      filteredRPM = alpha * rawRPM + (1 - alpha) * filteredRPM;
      currentRPM = filteredRPM;
      lastEncCount = count;

      // --- PID Control ---
      float error = globalSetpointRPM - currentRPM;
      if (abs(error) < deadband) error = 0;

      // Safety check
      if (isnan(error) || isinf(error)) {
        integral = 0;
        applyPWM(0);
        return;
      }

      error = constrain(error, -500, 500);
      float dt_s = dt_ms / 1000.0f;

      // Feedforward & Integral
      float u_ff = globalSetpointRPM * Kff;
      integral += error * dt_s;
      integral = constrain(integral, -I_MAX, I_MAX);

      // Compute Total U
      float u_pid = Kp * error + Ki * integral;
      float u = u_ff + u_pid;

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

      // Apply Power
      applyPWM(u_sat);
    }

    void applyPWM(float pwm) {
      digitalWrite(motR, LOW);
      ledcWrite(pwmCh, constrain((int)pwm, 0, 255));
    }

    void resetIntegral() {
      integral = 0.0f;
    }
};

// ======================================================
// INSTANTIATE MOTORS (Name, EncA, EncB, MotF, MotR, PWM_Channel, K_model, Tau)
// ======================================================
Motor flMotor("FL", 16, 17, 18, 19, 0, 2.016f, 0.039f); // Channel 0 [cite: 1, 2, 5]
Motor blMotor("BL", 21, 22, 23,  4, 1, 2.077f, 0.029f); // Channel 1 [cite: 33, 34, 37]
Motor brMotor("BR", 32, 33, 14, 27, 2, 1.954f, 0.044f); // Channel 2 [cite: 65, 66, 69]
Motor frMotor("FR", 34, 35,  5, 13, 3, 2.105f, 0.031f); // Channel 3 [cite: 97, 98, 101]

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  Serial.println("\n=== INITIALIZING OMNI BASE ===");
  flMotor.init();
  blMotor.init();
  brMotor.init();
  frMotor.init();

  lastSampleT = millis();
  Serial.println("\nSystem Ready. Enter a target RPM via Serial.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  uint32_t now = millis();
  uint32_t dt = now - lastSampleT;

  if (dt >= SAMPLE_MS) {
    flMotor.update(dt);
    blMotor.update(dt);
    brMotor.update(dt);
    frMotor.update(dt);

    lastSampleT = now;

    // Optional: Print telemetry for plotting/debugging
     Serial.printf("SP:%.1f | FL:%.1f | BL:%.1f | BR:%.1f | FR:%.1f\n", globalSetpointRPM, flMotor.currentRPM, blMotor.currentRPM, brMotor.currentRPM, frMotor.currentRPM);
  }

  // Serial Setpoint Listener
  if (Serial.available()) {
    float sp = Serial.parseFloat();
    if (sp > 0 && sp < 500) {
      globalSetpointRPM = sp;
      
      // Reset all integrals to prevent snapping when SP jumps
      flMotor.resetIntegral();
      blMotor.resetIntegral();
      brMotor.resetIntegral();
      frMotor.resetIntegral();
      
      Serial.printf("\n# NEW SHARED SETPOINT: %.1f RPM\n", sp);
    }
  }
}