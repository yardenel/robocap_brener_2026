#include <Arduino.h>

/*
 * teensy_l298n_motor_test.ino
 * -------------------------------------------------------------
 * Target : Teensy 4.1
 * Driver : L298N (single motor on channel A)
 *
 * Sequence (repeats):
 *   1) 5 s forward  (PWM ramps low -> full to demo variable speed)
 *   2) 1 s pause    (motor stopped)
 *   3) 5 s reverse  (PWM ramps low -> full)
 *   4) brief stop, then repeat
 *
 * Wiring (Teensy pin -> L298N):
 *   ENA (speed / PWM) -> GPIO 23   (PWM-capable pin)
 *   IN1 (dr1)         -> GPIO 13   (direction A)  NOTE: also onboard LED
 *   IN2 (dr2)         -> GPIO 41   (direction B)
 *
 * L298N direction truth table (channel A):
 *   IN1=HIGH IN2=LOW  -> forward
 *   IN1=LOW  IN2=HIGH -> reverse
 *   IN1=LOW  IN2=LOW  -> coast / stop
 * -------------------------------------------------------------
 */

// ---- Pin assignments -----------------------------------------
const uint8_t PIN_ENA = 23;   // PWM speed (ENA)
const uint8_t PIN_IN1 = 13;   // direction A (dr1)
const uint8_t PIN_IN2 = 41;   // direction B (dr2)

// ---- PWM configuration ---------------------------------------
// 8-bit resolution -> duty range 0..255.
// L298N switches slowly, so keep the frequency in the low-kHz range.
// 1 kHz is a safe, widely-used default; tune to taste.
const uint8_t  PWM_BITS    = 8;        // 0..255
const uint32_t PWM_FREQ_HZ = 1000;     // 1 kHz

// ---- Speed limits for the ramp (duty values, 0..255) ---------
const uint8_t DUTY_MIN = 60;    // minimum that reliably spins your motor under load
const uint8_t DUTY_MAX = 255;   // full speed

// ---- Timing (milliseconds) -----------------------------------
const uint32_t RUN_MS   = 5000; // 5 s per direction
const uint32_t PAUSE_MS = 1000; // 1 s pause

// ---- Direction helpers ---------------------------------------
void motorForward() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
}

void motorReverse() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
}

void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);   // 0 % duty
}

// Set speed directly (0..255)
void setSpeed(uint8_t duty) {
  analogWrite(PIN_ENA, duty);
}

/*
 * Run for durationMs while linearly ramping the PWM duty from
 * dutyStart to dutyEnd. Direction must be set before calling.
 */
void runRamped(uint32_t durationMs, uint8_t dutyStart, uint8_t dutyEnd) {
  const uint32_t t0 = millis();
  uint32_t elapsed = 0;
  while ((elapsed = millis() - t0) < durationMs) {
    float k    = (float)elapsed / (float)durationMs;          // 0.0 -> 1.0
    int   duty = dutyStart + (int)((dutyEnd - dutyStart) * k); // interpolate
    setSpeed((uint8_t)duty);
    delay(20);   // ~50 updates/sec, smooth ramp
  }
  setSpeed(dutyEnd);   // land exactly on the target
}

// ---- Setup ---------------------------------------------------
void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  analogWriteResolution(PWM_BITS);              // duty range 0..255
  analogWriteFrequency(PIN_ENA, PWM_FREQ_HZ);   // set PWM frequency on ENA

  motorStop();
  delay(500);   // settle before first cycle
}

// ---- Main loop -----------------------------------------------
void loop() {
  // 1) Forward 5 s, ramping speed up
  motorForward();
  runRamped(RUN_MS, DUTY_MIN, DUTY_MAX);

  // 2) Pause 1 s
  motorStop();
  delay(PAUSE_MS);

  // 3) Reverse 5 s, ramping speed up
  motorReverse();
  runRamped(RUN_MS, DUTY_MIN, DUTY_MAX);

  // 4) Stop briefly, then the loop repeats
  motorStop();
  delay(PAUSE_MS);
}
