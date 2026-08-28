#include <Arduino.h>
#include <ESP32Encoder.h>

// ======================================================
//  PIN & MOTOR CONFIG
// ======================================================
const int ENC_A      = 32;
const int ENC_B      = 33;
const int MOT_F      = 14;   // PWM pin (forward)
const int MOT_R      = 27;   // Direction pin

const int PWM_CH     = 0;
const int PWM_FREQ   = 20000;  // 20 kHz — inaudible
const int PWM_RES    = 8;      // 0–255

// ======================================================
//  ENCODER CONFIG
// ======================================================
ESP32Encoder encoder;
const float CPR = 269.5f * 2.0f;  // 1078 counts/rev (full quadrature)

// ======================================================
//  TIMING CONFIG
// ======================================================
const uint32_t SAMPLE_MS       = 20;    // 50 Hz data logging
const uint32_t STEP_DURATION   = 5000;  // 5s per step test
const uint32_t SETTLE_LONG     = 3000;  // 3s between step tests
const uint32_t SETTLE_SHORT    = 1500;  // 1.5s between ramp points
const uint32_t STEADY_DWELL    = 2500;  // 2.5s to reach steady state
const uint32_t DEADZONE_DWELL  = 700;   // 700ms per deadzone PWM step

// ======================================================
//  TEST PROFILES
// ======================================================

// Test 2 — Step response at 6 different PWM levels
const int STEP_PWMS[] = {60, 90, 128, 160, 200, 230};
const int NUM_STEPS   = 6;

// Test 3 — Steady-state RPM vs PWM (linearity map)
const int RAMP_PWMS[] = {30, 45, 60, 75, 90, 110, 130, 150, 170, 190, 210, 230, 255};
const int NUM_RAMP    = 13;

// ======================================================
//  STATE MACHINE
// ======================================================
enum State {
  WAIT_START,

  // Test 1 — Dead zone
  DZ_STEP,
  DZ_DWELL,

  DZ_SETTLE,

  // Test 2 — Step response
  STEP_INIT,
  STEP_RUN,
  STEP_SETTLE,

  // Test 3 — Linearity
  RAMP_INIT,
  RAMP_DWELL,
  RAMP_LOG,
  RAMP_SETTLE,

  DONE
};

State state = WAIT_START;

// ======================================================
//  RUNTIME VARIABLES
// ======================================================
float        currentRPM   = 0.0f;
long         lastEncCount = 0;
uint32_t     lastSampleT  = 0;

uint32_t     stateStart   = 0;
uint32_t     lastLogT     = 0;

int          stepIdx      = 0;
int          rampIdx      = 0;
int          scanPWM      = 0;
int          deadzonePWM  = -1;

// Rolling average for steady-state (Test 3)
float        rpmAccum     = 0.0f;
int          rpmCount     = 0;

// ======================================================
//  MOTOR & ENCODER HELPERS
// ======================================================
void motorPWM(int pwm) {
  digitalWrite(MOT_R, LOW);
  ledcWrite(PWM_CH, constrain(pwm, 0, 255));
}

void motorStop() {
  ledcWrite(PWM_CH, 0);
  digitalWrite(MOT_R, LOW);
}

void resetEncoder() {
  encoder.setCount(0);
  lastEncCount = 0;
}

void updateRPM() {
  uint32_t now = millis();
  uint32_t dt  = now - lastSampleT;
  if (dt >= SAMPLE_MS) {
    long count = encoder.getCount();
    long delta = count - lastEncCount;
    currentRPM      = (float(delta) / CPR) * (60000.0f / dt);
    lastEncCount    = count;
    lastSampleT     = now;
  }
}

// ======================================================
//  SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A, ENC_B);
  resetEncoder();

  // Motor driver
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOT_F, PWM_CH);
  pinMode(MOT_R, OUTPUT);
  motorStop();

  Serial.println(F("========================================"));
  Serial.println(F("  DC Motor System Identification Suite  "));
  Serial.println(F("========================================"));
  Serial.println(F("Tests:"));
  Serial.println(F("  1. Dead zone scan     (PWM 0 -> 255)"));
  Serial.println(F("  2. Step response      (6 PWM levels)"));
  Serial.println(F("  3. Steady-state map   (13 PWM levels)"));
  Serial.println(F(""));
  Serial.println(F("Send 's' + Enter to begin."));
}

// ======================================================
//  LOOP — STATE MACHINE
// ======================================================
void loop() {
  updateRPM();
  uint32_t now = millis();

  switch (state) {

    // --------------------------------------------------
    case WAIT_START:
      if (Serial.available() && Serial.read() == 's') {
        Serial.println(F("\n--- TEST 1: DEAD ZONE SCAN ---"));
        Serial.println(F("# Slowly ramps PWM 0->255, finds minimum PWM to spin"));
        Serial.println(F("pwm,rpm"));
        scanPWM    = 0;
        stateStart = now;
        motorPWM(scanPWM);
        state = DZ_DWELL;
        stateStart = now;
      }
      break;

    // --------------------------------------------------
    // TEST 1 — Dead Zone: apply PWM, dwell, log, increment
    // --------------------------------------------------
    case DZ_DWELL:
      if (now - stateStart >= DEADZONE_DWELL) {
        // Log this point
        Serial.printf("%d,%.2f\n", scanPWM, currentRPM);

        // First time RPM exceeds threshold → record deadzone
        if (deadzonePWM == -1 && currentRPM > 5.0f) {
          deadzonePWM = scanPWM;
          Serial.printf("# >> Deadzone threshold: PWM = %d\n", deadzonePWM);
        }

        scanPWM += 5;
        if (scanPWM > 255) {
          // Scan complete
          motorStop();
          Serial.printf("# Dead zone scan complete. Min PWM = %d\n", deadzonePWM);
          Serial.println(F("# Settling 3s..."));
          state      = DZ_SETTLE;
          stateStart = now;
        } else {
          motorPWM(scanPWM);
          stateStart = now;  // reset dwell timer
        }
      }
      break;

    case DZ_SETTLE:
      if (now - stateStart >= SETTLE_LONG) {
        Serial.println(F("\n--- TEST 2: STEP RESPONSE ---"));
        Serial.println(F("# Step applied at t=0, logs RPM for 5s each"));
        stepIdx = 0;
        state   = STEP_INIT;
      }
      break;

    // --------------------------------------------------
    // TEST 2 — Step Response
    // --------------------------------------------------
    case STEP_INIT:
      resetEncoder();
      motorPWM(STEP_PWMS[stepIdx]);
      Serial.printf("\n# Step %d/%d  |  PWM = %d\n",
                    stepIdx + 1, NUM_STEPS, STEP_PWMS[stepIdx]);
      Serial.println(F("time_ms,pwm,rpm"));
      stateStart = now;
      lastLogT   = now;
      state      = STEP_RUN;
      break;

    case STEP_RUN:
      // Log at SAMPLE_MS rate
      if (now - lastLogT >= SAMPLE_MS) {
        Serial.printf("%lu,%d,%.2f\n",
          now - stateStart, STEP_PWMS[stepIdx], currentRPM);
        lastLogT = now;
      }
      // Step duration over
      if (now - stateStart >= STEP_DURATION) {
        motorStop();
        Serial.printf("# Step %d done. Settling %lus...\n",
                      stepIdx + 1, SETTLE_LONG / 1000);
        stateStart = now;
        state      = STEP_SETTLE;
      }
      break;

    case STEP_SETTLE:
      if (now - stateStart >= SETTLE_LONG) {
        stepIdx++;
        if (stepIdx >= NUM_STEPS) {
          // All steps done
          Serial.println(F("\n--- TEST 3: STEADY-STATE LINEARITY MAP ---"));
          Serial.println(F("# Holds each PWM 2.5s, logs average RPM"));
          Serial.println(F("pwm,rpm_avg"));
          rampIdx = 0;
          state   = RAMP_INIT;
        } else {
          state = STEP_INIT;   // next step
        }
      }
      break;

    // --------------------------------------------------
    // TEST 3 — Steady-State Linearity
    // --------------------------------------------------
    case RAMP_INIT:
      motorPWM(RAMP_PWMS[rampIdx]);
      rpmAccum   = 0.0f;
      rpmCount   = 0;
      stateStart = now;
      lastLogT   = now;
      state      = RAMP_DWELL;
      break;

    case RAMP_DWELL:
      // Accumulate RPM samples in second half of dwell (steady portion)
      if (now - stateStart > STEADY_DWELL / 2) {
        if (now - lastLogT >= SAMPLE_MS) {
          rpmAccum += currentRPM;
          rpmCount++;
          lastLogT = now;
        }
      }
      if (now - stateStart >= STEADY_DWELL) {
        state = RAMP_LOG;
      }
      break;

    case RAMP_LOG: {
      float avgRPM = (rpmCount > 0) ? (rpmAccum / rpmCount) : 0.0f;
      Serial.printf("%d,%.2f\n", RAMP_PWMS[rampIdx], avgRPM);
      motorStop();
      stateStart = now;
      state      = RAMP_SETTLE;
      break;
    }

    case RAMP_SETTLE:
      if (now - stateStart >= SETTLE_SHORT) {
        rampIdx++;
        if (rampIdx >= NUM_RAMP) {
          state = DONE;
        } else {
          state = RAMP_INIT;
        }
      }
      break;

    // --------------------------------------------------
    case DONE:
      motorStop();
      Serial.println(F("\n========================================"));
      Serial.println(F("       ALL TESTS COMPLETE               "));
      Serial.println(F("========================================"));
      Serial.printf("Dead zone PWM   : %d / 255\n", deadzonePWM);
      Serial.printf("Dead zone %%     : %.1f%%\n", deadzonePWM * 100.0f / 255.0f);
      Serial.println(F("Copy this Serial output into a .txt file"));
      Serial.println(F("and run the Python analysis script."));
      // Halt
      while (true) delay(1000);
      break;
  }
}