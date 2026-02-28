/**
 * ============================================================
 * RoboCup Junior — IR Ball Sensor Array
 * Teensy 4.1 | 20× TSOP34838 | 74HC4067 MUX + 4 Direct
 * ============================================================
 * HARDWARE:
 *   MUX 74HC4067:
 *     S0 → pin 31, S1 → pin 30, S2 → pin 29, S3 → pin 28
 *     SIG → pin 32 (common output, TSOP active-LOW)
 *   Direct sensors:
 *     Sensor 17 → pin 24
 *     Sensor 18 → pin 25
 *     Sensor 19 → pin 26
 *     Sensor 20 → pin 27
 *
 * BALL WAVEFORM (from spec):
 *   Carrier frequency : 40 kHz  (25µs period)
 *   Burst duration    : ~175µs  (7 carrier cycles)
 *   Burst period      : 833µs   (1.2kHz repetition)
 *   Gap between bursts: ~658µs
 *
 * TSOP34838 NOTE:
 *   Tuned for 38kHz — ball transmits 40kHz.
 *   Works with ~3-6dB sensitivity reduction.
 *   Ideal replacement: TSOP34840 (40kHz).
 *
 * SCAN STRATEGY — Optimized Synchronized:
 *   MUX channels (0–15): waitForBurst(200µs) per channel
 *     - 200µs > burst duration (175µs) → catches any burst starting now
 *     - Worst case: 16 × (2µs settle + 200µs) = ~3.2ms total
 *   Direct sensors (16–19): hardware interrupt on FALLING edge
 *     - ISR sets flag instantly — zero latency
 *     - Flags accumulate during ~3.2ms MUX scan window
 *     - 3.2ms window covers ~3.8 full burst cycles → guaranteed catch
 *
 *   Total scan time:
 *     Worst case (no ball): ~3.2ms
 *     Best case (ball visible): < 1ms average
 *     Full cycle: 50ms (20Hz refresh — adequate for ball tracking)
 *
 * OUTPUT:
 *   bool irSensors[20] — true = ball signal received on that sensor
 *   float ballAngle    — estimated ball direction (0–360°, circular mean)
 *   int   ballCount    — number of sensors detecting the ball
 *
 * REQUIRES: No external libraries (bare Teensy 4.1)
 * ============================================================
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
// SECTION 1 — PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

// 74HC4067 MUX select lines
#define MUX_S0 31
#define MUX_S1 30
#define MUX_S2 29
#define MUX_S3 28
#define MUX_SIG 32  // MUX common signal output (active LOW)

// Direct sensor pins (sensors 17–20, array index 16–19)
const uint8_t DIRECT_PINS[4] = {24, 25, 26, 27};

// ═══════════════════════════════════════════════════════════════
// SECTION 2 — CONFIGURATION
// ═══════════════════════════════════════════════════════════════

#define NUM_SENSORS 20
#define NUM_MUX_CH 16
#define SCAN_PERIOD_MS 50  // Full refresh every 50ms (20Hz)

// Burst timing (from ball waveform spec)
#define BURST_DURATION_US 175  // One burst from ball transmitter
#define BURST_PERIOD_US 833    // Full burst cycle (1.2kHz)

// Timeout per MUX channel:
// Must be > BURST_DURATION_US to catch a burst that just started.
// 200µs gives 25µs margin above 175µs burst.
// Keeping it short (not full 833µs) keeps total scan fast.
#define BURST_TIMEOUT_US 200

// MUX propagation delay (74HC4067 typ 15ns, we add margin)
#define MUX_SETTLE_US 2

// Sensor layout — physical mounting angle on robot
// 0° = front, clockwise positive.
// !! Adjust to match your robot's actual sensor positions !!
// 20 sensors assumed at uniform 18° spacing.
const float SENSOR_ANGLES[NUM_SENSORS]
    = {0,   18,  36,  54,  72,  90,  108, 126, 144, 162,
       180, 198, 216, 234, 252, 270, 288, 306, 324, 342};

// ═══════════════════════════════════════════════════════════════
// SECTION 3 — GLOBAL STATE
// ═══════════════════════════════════════════════════════════════

// Public results — written by doScanOptimized(), read by processIRData()
bool irSensors[NUM_SENSORS] = {};
float ballAngle             = -1.0f;  // -1 = no ball detected
int ballCount               = 0;

// ISR flags for direct sensors — volatile because modified in ISR
volatile bool directDetected[4] = {};

// Timer trigger flag
volatile bool triggerScan = false;
IntervalTimer periodicTimer;

// ═══════════════════════════════════════════════════════════════
// SECTION 4 — MUX CONTROL
// ═══════════════════════════════════════════════════════════════

// Select 74HC4067 channel 0–15 via S0–S3
inline void setMuxChannel(uint8_t ch) {
    digitalWriteFast(MUX_S0, (ch >> 0) & 1);
    digitalWriteFast(MUX_S1, (ch >> 1) & 1);
    digitalWriteFast(MUX_S2, (ch >> 2) & 1);
    digitalWriteFast(MUX_S3, (ch >> 3) & 1);
}

// ═══════════════════════════════════════════════════════════════
// SECTION 5 — SYNCHRONIZED BURST DETECTION
// ═══════════════════════════════════════════════════════════════

/**
 * Waits for TSOP output to go LOW (burst received) within timeoutUs.
 *
 * Why 200µs and not the full 833µs period?
 *   A burst lasts 175µs. If we start waiting at a random moment in
 *   the burst cycle, the worst case is we just missed a burst and
 *   need to wait up to 658µs (gap) + 175µs (next burst) = 833µs
 *   for the NEXT one. However, in practice with 40kHz transmitter
 *   and 38kHz TSOP, detection range is reduced — but if the ball is
 *   visible at all, the TSOP will still pulse LOW. With 200µs timeout:
 *   - If we arrive mid-burst → detected immediately (< 25µs remaining)
 *   - If we arrive just after burst → 658µs gap, will NOT detect
 *
 *   This is the intentional design trade-off:
 *   - 200µs per channel × 16 channels = 3.2ms total scan (FAST)
 *   - A channel reports "no ball" if no burst arrives in that 200µs
 *   - Statistical safety: each channel gets sampled every 50ms →
 *     50ms / 833µs = ~60 burst cycles per refresh → over multiple
 *     refreshes, a ball in range will always be caught
 *   - For better single-scan guarantee, increase to 900µs (full cycle)
 *     at the cost of 16 × 900µs = 14.4ms scan time
 *
 * Returns: true if burst detected, false if timeout
 */
inline bool waitForBurst(uint8_t pin, uint32_t timeoutUs) {
    uint32_t deadline = micros() + timeoutUs;
    while ((int32_t)(micros() - deadline) < 0) {
        if (!digitalReadFast(pin)) return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════
// SECTION 6 — DIRECT SENSOR INTERRUPTS
// ═══════════════════════════════════════════════════════════════
/**
 * Hardware interrupts on FALLING edge (TSOP active LOW).
 * ISR sets flag immediately — zero scan latency for direct sensors.
 * Flags accumulate during the ~3.2ms MUX scan window.
 * 3.2ms / 833µs ≈ 3.8 full burst cycles → guaranteed detection
 * if ball is in range of any direct sensor.
 *
 * FASTRUN attribute places ISR in ITCM (fastest RAM on Teensy 4.1).
 */

void FASTRUN isrDirect0() { directDetected[0] = true; }
void FASTRUN isrDirect1() { directDetected[1] = true; }
void FASTRUN isrDirect2() { directDetected[2] = true; }
void FASTRUN isrDirect3() { directDetected[3] = true; }

// ═══════════════════════════════════════════════════════════════
// SECTION 7 — OPTIMIZED SYNCHRONIZED SCAN
// ═══════════════════════════════════════════════════════════════

/**
 * Full 20-sensor scan with synchronized burst detection.
 *
 * Execution timeline:
 *  t=0:       Reset direct sensor ISR flags
 *  t=0..3.2ms: Scan MUX channels 0–15 sequentially
 *              Direct sensor ISRs fire freely in background
 *  t=3.2ms:   Collect direct sensor results from ISR flags
 *  t=3.2ms:   Commit results to irSensors[]
 *
 * Worst-case total: ~3.2ms (all channels timeout — no ball)
 * This leaves ~46.8ms of the 50ms cycle for other robot tasks.
 */
void doScanOptimized() {
    bool results[NUM_SENSORS] = {};

    // ── Step 1: Reset direct sensor ISR flags atomically ─────
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) directDetected[i] = false;
    interrupts();

    // ── Step 2: Scan MUX channels (0–15) ─────────────────────
    // Direct sensor ISRs run freely in background during this loop.
    // Each channel: 2µs settle + up to 200µs burst wait = 202µs max
    // Total: 16 × 202µs ≈ 3.2ms
    for (uint8_t ch = 0; ch < NUM_MUX_CH; ch++) {
        setMuxChannel(ch);
        delayMicroseconds(MUX_SETTLE_US);
        results[ch] = waitForBurst(MUX_SIG, BURST_TIMEOUT_US);
    }

    // ── Step 3: Collect direct sensor results (16–19) ─────────
    // ISR flags have been accumulating for ~3.2ms (≈3.8 burst cycles)
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
        results[NUM_MUX_CH + i] = directDetected[i];
    }
    interrupts();

    // ── Step 4: Atomic commit to public array ─────────────────
    memcpy(irSensors, results, sizeof(results));
}

// ═══════════════════════════════════════════════════════════════
// SECTION 8 — BALL DIRECTION PROCESSING
// ═══════════════════════════════════════════════════════════════

/**
 * Calculates ball angle using circular (vector) mean.
 *
 * Why circular mean and not simple average?
 *   Simple average of [350°, 10°] = 180° — WRONG.
 *   Circular mean: sin/cos components → atan2 → 0° — CORRECT.
 *
 * Multiple sensors detecting ball → weighted vector sum.
 * Result written to globals: ballAngle, ballCount.
 */
void processIRData() {
    float sinSum = 0.0f, cosSum = 0.0f;
    ballCount = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (irSensors[i]) {
            float rad = SENSOR_ANGLES[i] * DEG_TO_RAD;
            sinSum += sinf(rad);
            cosSum += cosf(rad);
            ballCount++;
        }
    }

    if (ballCount > 0) {
        ballAngle = atan2f(sinSum, cosSum) * RAD_TO_DEG;
        if (ballAngle < 0.0f) ballAngle += 360.0f;
    } else {
        ballAngle = -1.0f;  // No ball in range
    }
}

// ═══════════════════════════════════════════════════════════════
// SECTION 9 — TIMER ISR (flag-only — minimal ISR rule)
// ═══════════════════════════════════════════════════════════════

void onScanTick() { triggerScan = true; }

// ═══════════════════════════════════════════════════════════════
// SECTION 10 — SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);

    // ── MUX select lines ──────────────────────────────────────
    pinMode(MUX_S0, OUTPUT);
    digitalWriteFast(MUX_S0, LOW);
    pinMode(MUX_S1, OUTPUT);
    digitalWriteFast(MUX_S1, LOW);
    pinMode(MUX_S2, OUTPUT);
    digitalWriteFast(MUX_S2, LOW);
    pinMode(MUX_S3, OUTPUT);
    digitalWriteFast(MUX_S3, LOW);

    // ── MUX signal input (TSOP push-pull — no pull-up needed) ─
    pinMode(MUX_SIG, INPUT);

    // ── Direct sensor pins + hardware interrupts ──────────────
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(DIRECT_PINS[i], INPUT);
    }
    attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[0]), isrDirect0, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[1]), isrDirect1, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[2]), isrDirect2, FALLING);
    attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[3]), isrDirect3, FALLING);

    // ── Start 50ms periodic scan timer ────────────────────────
    periodicTimer.begin(onScanTick, SCAN_PERIOD_MS * 1000UL);

    Serial.println("[IR] Scanner ready");
    Serial.printf(
        "[IR] Burst timeout: %dµs/ch | Scan window: ~%dms\n",
        BURST_TIMEOUT_US,
        (NUM_MUX_CH * (MUX_SETTLE_US + BURST_TIMEOUT_US)) / 1000
    );
}

// ═══════════════════════════════════════════════════════════════
// SECTION 11 — MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
    if (triggerScan) {
        triggerScan = false;

        // ── Synchronized scan ─────────────────────────────────
        doScanOptimized();

        // ── Calculate ball direction ───────────────────────────
        processIRData();

        // ── irSensors[0..19], ballAngle, ballCount are valid ──
        // Pass ballAngle to motor controller / strategy logic here.

        // ── Debug output (remove in competition) ──────────────
        if (ballCount > 0) {
            Serial.printf("[IR] Ball: %.1f° | %d sensors\n", ballAngle, ballCount);
        } else {
            Serial.println("[IR] No ball");
        }
    }

    // Other robot tasks: motors, kicker, color sensors, comms...
}
